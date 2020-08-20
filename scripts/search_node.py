#!/usr/bin/env python
from __future__ import division
import rospy
import yaml
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from minau.msg import SonarTargetList, SonarTarget
from minau.srv import ArmControl, SetHeadingVelocity
from etddf.srv import SetSonarSettings
from etddf.msg import SonarSettings
from ping360_sonar.msg import SonarEcho
import scipy.stats
import matplotlib.pyplot as plt
import tf
import math
import copy
import perlin


#Open up config file to get the volume we are going to patrol
# volume_file = rospy.get_param("~volume_file")
def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

sonar_range = 10
twenty_grad = (20 * np.pi/200.0)
print(twenty_grad)
grid = np.zeros((23,23))
sonar_view = {}
for x in range(20):
    sonar_angle = (-200 + 20*x) * (np.pi/200.0)
    points = []
    for i in range(23):
        for j in range(23):
            ang = np.arctan2(11-j,11-i)
            rang = np.linalg.norm([11-j,11-i])
            if rang <= sonar_range:
                if ang >= sonar_angle and ang <= sonar_angle+twenty_grad:
                    points.append([11-i,11-j,rang])
    sonar_view[20*x] = points

def normalize_velocity(v,speed):
    """Takes in the position difference vector and returns velcity vector
    Arguments:
        v {Vector3} -- 3d vector of position difference of the rov and its waypoint
        speed {float} -- the speed we want the rov to move at
    Returns:
        Vector3 -- the velocity vector of the rov
    """
    size=np.sqrt(v.x**2+v.y**2+v.z**2)
    size = size/speed
    v.x = v.x/size
    v.y = v.y/size
    v.z = v.z/size
    return v



class Volume:
    def __init__(self,vol_dict):
        self.x_range = [vol_dict['volume']['x1'],vol_dict['volume']['x2']]
        self.y_range = [vol_dict['volume']['y1'],vol_dict['volume']['y2']]
        self.z = vol_dict['volume']['z']
        self.delta = vol_dict['volume']['delta']
        

class Space:
    def __init__(self,v):
        self.volume = v
        self.x_diff = self.volume.x_range[1] - self.volume.x_range[0]
        self.y_diff = self.volume.y_range[1] - self.volume.y_range[0]
        self.x_0 = self.volume.x_range[0]
        self.y_0 = self.volume.y_range[0]
        self.z = self.volume.z
        self.plot_count = 1
        self.range = self.x_diff * self.volume.delta
        if self.y_diff > self.x_diff:
            self.range = self.y_diff * self.volume.delta
        self.discritize_space()
    def discritize_space(self):
        x_blocks = int(self.x_diff/self.volume.delta) + 1
        y_blocks = int(self.y_diff/self.volume.delta) + 1
        mean_x_1 = (self.volume.x_range[0] + (self.x_diff)/2.0) + 10.0
        mean_x_2 = self.volume.x_range[0] + (self.x_diff)/2.0
        mean_y_1 = self.volume.y_range[0] + 3*(self.x_diff)/4.0
        mean_y_2 = self.volume.x_range[0] + 1*(self.x_diff)/4.0
        std_dev = self.x_diff/10.0

        distr_1 = scipy.stats.norm(0,std_dev)
        distr_2 = scipy.stats.norm(0,std_dev)
        
        p = perlin.PerlinNoiseFactory(2,unbias=True)
        delta = 4.0/x_blocks

        self.space = np.ones((x_blocks,y_blocks))*0.5
        for i in range(x_blocks):
            for j in range(y_blocks):
                self.space[i,j] += (p(i*delta,j*delta)+1)/2.0

        self.x_cords = [self.volume.x_range[0] + self.volume.delta*i for i in range(x_blocks)]
        self.y_cords = [self.volume.y_range[0] + self.volume.delta*i for i in range(y_blocks)]

        # for i in range(len(self.x_cords)):
        #     for j in range(len(self.y_cords)):
        #         range_1 = np.linalg.norm([self.x_cords[i]-mean_x_1,self.y_cords[j]-mean_y_1])
        #         range_2 = np.linalg.norm([self.x_cords[i]-mean_x_2,self.y_cords[j]-mean_y_2])
        #         self.space[i,j] += distr_1.pdf(range_1) + distr_2.pdf(range_2)

        self.space/=self.space.sum()
        
        

class Search:
    def __init__(self,space,name,half):
        #0 for top half, 1 for bottom half
        print(name)
        self.half = half
        #name in format '/bluerov2_X'
        self.space = space
        self.need_plot = False
        self.name = name
        self.range = 10
        self.truth_yaw = None
        self.angle_start = None
        self.angle_count = 0
        self.angle_scan_begin = 0
        self.step = 1
        self.red_found = False
        self.x = None
        self.waypoint = None
        self.lawn = rospy.get_param("~lawn",False)
        self.visited_x = []
        self.visited_y = []

        self.target_pos = [-9,-6]
        #Make sure both rovs are armed
        rospy.wait_for_service(self.name+'/uuv_control/arm_control')
        self.arm_control = rospy.ServiceProxy(self.name+'/uuv_control/arm_control', ArmControl)
        resp1 = self.arm_control()
        self.prev_dist = 0
        
        #Set up services to set the velocity
        rospy.wait_for_service(self.name+'/uuv_control/set_heading_velocity')
        self.shv = rospy.ServiceProxy(self.name+'/uuv_control/set_heading_velocity', SetHeadingVelocity)

        #Subscribes to the necisary topics for the search node for each rov
        rospy.Subscriber(self.name+"/etddf/estimate"+self.name,Odometry,self.ownship_callback)
        rospy.Subscriber(self.name+"/ping_360_target",SonarTargetList,self.detections)
        rospy.Subscriber(self.name+"/pose_gt",Odometry,self.truth_callback)
        rospy.Subscriber(self.name+"/ping360_node/sonar/data",SonarEcho,self.angle_callback)
        self.first = True

    def ownship_callback(self,msg):
        self.position = msg.pose.pose.position
        (r,p,self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def angle_callback(self,msg):
        #read in from the visual publisher to know where it is looking
        #determine the angle range it has viewed
        if self.truth_yaw==None:
            return
        self.angle = msg.angle
        if (self.angle - self.angle_scan_begin)%400 >= 20:
            self.angle_count+=1
            # print(self.angle)
            self.angle_scan_begin = self.angle
            self.not_seen(self.angle)
            self.update()

    
    def update(self):
        sp = copy.deepcopy(self.space.space)
        for i in range(len(self.space.x_cords)):
            for j in range(len(self.space.y_cords)):
                sp[i][j] = self.space.space[i][j] * 0.95 + 0.05 * self.neighbor(i,j)
        self.space.space = sp/sp.sum()

    def neighbor(self,x,y):
        sum = 0
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                if x+i >= 0 and x+i < len(self.space.space) and y+j>=0 and y+j < len(self.space.space[0]):
                    sum += self.space.space[i+x,j+y]
                else:
                    sum += self.space.space[i][j]*1.03
        sum-=self.space.space[x,y]
        return sum/8.0


    def not_seen(self,angle):
        world_ang = ((angle-20) + (self.truth_yaw * 200/np.pi))%400
        world_ang/=20
        world_ang = round(world_ang)
        world_ang = int((world_ang*20)%400)
        x_idx = round(self.truth_pos.x - self.space.x_0)
        y_idx = round(self.truth_pos.y - self.space.y_0)
        for i in range(len(sonar_view[world_ang])):
            x = int(x_idx+sonar_view[world_ang][i][0])
            y = int(y_idx+sonar_view[world_ang][i][1])
            if x >= 0 and x < len(self.space.x_cords):
                if y >= 0 and y < len(self.space.y_cords):
                    self.space.space[x,y]*=(1-(0.9-0.07*sonar_view[world_ang][i][2]))

    def detections(self,msg):
        for target in msg.targets:
            # print(target.id)
            # if target.id != 'red_asset':
            #     continue
            # else:
            #     self.red_found = True
                self.spotted(target)
    
    def spotted(self,target):
        range_to = target.range_m
        bearing = target.bearing_rad
        bearing += self.truth_yaw
        bearing = normalize_angle(bearing)
        x = self.truth_pos.x + range_to*np.cos(bearing)
        y = self.truth_pos.y + range_to*np.sin(bearing)
        d = np.linalg.norm([x-self.target_pos[0],y-self.target_pos[1]])
        print(d)
        if d > 2:
            return
        self.red_found = True
        x_close = None
        y_close = None
        for i in range(len(self.space.x_cords)):
            if np.linalg.norm(self.space.x_cords[i]-x) < .5:
                x_close = i
                break
        for i in range(len(self.space.y_cords)):
            if np.linalg.norm(self.space.y_cords[i]-y) < .5:
                y_close = i
                break
        self.space.space[x_close][y_close] = 0
        self.space.space = self.space.space*0.2/(self.space.space.sum())
        self.space.space[x_close][y_close] = 0.8
        self.need_plot = True

    def truth_callback(self,msg):
        self.truth_pos = msg.pose.pose.position
        (r,p,self.truth_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def new_waypoint(self):
        if not self.lawn:
            if self.half == 0:
                dir = 1
            else:
                dir = -1
            max_x = 0
            max_y = dir
            total_max = 0
            for i in range(len(self.space.x_cords)):
                for j in range(int(len(self.space.y_cords)/2)):
                    total = 0
                    # for k in range(21):
                    #     if k+i-10 >=0 and k+i-10 < len(self.space.x_cords):
                    #         total += self.space.space[k+i-10,dir*(j+1)]
                    # for k in range(21):
                    #     if dir*(j+1) + k -10 >=0 and dir*(j+1) + k -10 <len(self.space.y_cords):
                    #         total+= self.space.space[i,dir*(j+1) + k -10]
                    # if total > total_max:
                    if self.space.space[i,dir*(j+1)] > self.space.space[max_x,max_y]:
                        max_x,max_y = i,dir*(j+1)
                        # total_max = total
            self.waypoint = [self.space.x_cords[max_x],self.space.y_cords[max_y],self.space.z]
        else:
            if self.x == None:
                self.x1 = self.space.x_cords[0] +5
                self.x2 = self.space.x_cords[-1] -5
                self.y1 = self.space.y_cords[0]
                self.y2 = self.space.y_cords[-1]
                y_diff = (self.y2-self.y1)/2
                self.moveX = True
                self.x = True
                self.y = True      #true for positive

                self.y1 = self.y1 + (y_diff)*self.half + 5
                self.y2 = self.y1 + y_diff -10
                self.waypoint = [self.x1,self.y1,self.space.z]
                return

            if self.moveX:
                if self.x:
                    self.waypoint[0] = self.x2
                    self.x = False
                else:
                    self.waypoint[0] = self.x1
                    self.x = True
                self.moveX = False
                return

            self.moveX = True
            if self.y:
                self.waypoint[1] += 15
                print(self.waypoint[1])
                print(self.y2)
                if self.waypoint[1] >= self.y2:
                    self.waypoint[1] = self.y2
                    self.y = False
                return
            self.waypoint[1] -= 15
            if self.waypoint[1] <= self.y1:
                self.waypoint[1] = self.y1
                self.y = True

    def plot(self, still_looking):
        X,Y = np.meshgrid(self.space.x_cords,self.space.y_cords)
        fig,ax=plt.subplots(1,1)
        levels = np.linspace(0,0.007,15)
        cp = ax.contourf(X, Y, self.space.space.transpose())
        fig.colorbar(cp) # Add a colorbar to a plot
        ax.set_title(self.name)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        if still_looking:
            # self.visited_x.append(self.truth_pos.x)
            # self.visited_y.append(self.truth_pos.y)
            # plt.plot(self.visited_x,self.visited_y,'bo')
            plt.plot(self.waypoint[0],self.waypoint[1],'ro')
        name = 'space_' + str(self.space.plot_count) +'.png'
        self.space.plot_count +=1
        plt.savefig(name)



    def run(self):
        rate = rospy.Rate(1)
        while self.truth_yaw==None:
            rate.sleep()
        #first waypoint
        if self.first:
            self.new_waypoint()
            self.need_plot = True
            self.first = False
        x_diff = self.waypoint[0]-self.truth_pos.x
        y_diff = self.waypoint[1]-self.truth_pos.y
        z_diff = self.waypoint[2]-self.truth_pos.z
        v = Vector3(y_diff,x_diff,-z_diff)
        v = normalize_velocity(v,0.4)
        ang = np.arctan2(x_diff,y_diff)
        dist = math.sqrt(x_diff**2+y_diff**2+z_diff**2)
        if dist < 0.5:
            # print('Reached Target ' + self.name)
            v_0 = Vector3(0,0,0)
            self.shv(0,v_0)
            if self.angle_start == None:
                print('Reached Target ' + self.name)
                print(self.angle)
                self.angle_start = self.angle_count
            elif self.angle_count>=self.angle_start+20:
                self.update()
                print('Yo!')
                self.need_plot = True
                self.new_waypoint()
                print('New Waypoint for ' + self.name+' : ')
                print(self.angle)
                print(self.waypoint)
                self.angle_start=None
        else:
            if np.linalg.norm(self.prev_dist-dist) < 0.001:
                self.arm_control()
                rate.sleep()
            self.prev_dist = dist
            self.shv(ang*(180/np.pi),v)
        # print(self.waypoint)
        # print("Dist error for " + self.name + ":" + str(dist))



def plot(s):
    X,Y = np.meshgrid(s.x_cords,s.y_cords)
    fig,ax=plt.subplots(1,1)
    levels = np.linspace(0,0.007,15)
    cp = ax.contourf(X, Y, s.space.transpose())
    fig.colorbar(cp) # Add a colorbar to a plot
    name =  'first.png'
    plt.savefig(name)

if __name__ == "__main__":
    rospy.init_node("search_node")
    volume = None
    with open('hello.yaml') as file:
        volume = yaml.load(file, Loader=yaml.FullLoader)
    v = Volume(volume)
    space = Space(v)
    plot(space)
    s3 = Search(space,'/bluerov2_3',0)
    s4 = Search(space,'/bluerov2_4',1)
    initial_time = rospy.get_rostime()
    print(initial_time)
    rate = rospy.Rate(20)
    while not (s3.red_found or s4.red_found) and (not rospy.is_shutdown()):
        if s3.need_plot:
            s3.plot(True)
            s3.need_plot = False
            continue
        if s4.need_plot:
            s4.plot(True)
            s4.need_plot = False
            continue
        s3.run()
        s4.run()
        rate.sleep()
    print "=============================================="
    s3.plot(False)
    print('Total time')
    print(rospy.get_rostime()-initial_time)