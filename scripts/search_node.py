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
import tf
import math


#Open up config file to get the volume we are going to patrol
# volume_file = rospy.get_param("~volume_file")
def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

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

class Space:
    def __init__(self,v):
        self.volume = v
        x_diff = v.x_range[1]-v.x_range[0]
        y_diff = v.y_range[1]-v.y_range[0]
        self.range = x_diff/4.
        if y_diff > x_diff:
            self.range = y_diff/4.
        self.discritize_space()
    def discritize_space(self):
        self.x_diff = self.volume.x_range[1] - self.volume.x_range[0]
        self.y_diff = self.volume.y_range[1] - self.volume.y_range[0]
        self.z = self.volume.z
        x_blocks = int(self.x_diff*4) + 1
        y_blocks = int(self.y_diff*4) + 1
        self.space = np.zeros((x_blocks,y_blocks))
        self.x_cords = [self.volume.x_range[0] + 0.25*i for i in range(x_blocks)]
        self.y_cords = [self.volume.y_range[0] + 0.25*i for i in range(y_blocks)]
        
        

class Search:
    def __init__(self,space,name):
        #name in format '/bluerov2_X'
        self.space = space
        self.name = name

        #Make sure both rovs are armed
        rospy.wait_for_service(self.name+'/uuv_control/arm_control')
        arm_control = rospy.ServiceProxy(self.name+'/uuv_control/arm_control', ArmControl)
        resp1 = arm_control()
    
        
        #Set up services to set the velocity
        rospy.wait_for_service(self.name+'/uuv_control/set_heading_velocity')
        self.shv = rospy.ServiceProxy(self.name+'/uuv_control/set_heading_velocity', SetHeadingVelocity)


        #360 sonar settings with 10 m range
        settings = SonarSettings()
        settings.range = 10
        settings.min_angle = -4
        settings.max_angle = 4

        #Set up services to set the sonar settings
        rospy.wait_for_service(self.name+'/set_sonar_settings')
        self.set_sonar = rospy.ServiceProxy(self.name+'/set_sonar_settings',SetSonarSettings)

        self.set_sonar(settings)

        #Subscribes to the necisary topics for the search node for each rov
        rospy.Subscriber(self.name+"/etddf/estimate/bluerov2_3",Odometry,self.ownship_callback)
        rospy.Subscriber(self.name+"/bluerov2_3/ping_360_target",SonarTargetList,self.detections)
        rospy.Subscriber(self.name+"/bluerov2_3/pose_gt",Odometry,self.truth_callback)
        rospy.Subscriber(self.name+"/bluerov2_3/visual_pub",LaserScan,self.angle_callback)
        self.truth_yaw = None

    def ownship_callback(self,msg):
        self.position = msg.pose.pose.position
        (r,p,self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    def inview(self,x,y,angles):
        x_diff = x-self.truth_pos.x
        y_diff = y-self.truth_pos.y
        dist = np.linalg.norm([x_diff,y_diff])
        xy_ang = np.arctan2(y_diff,x_diff)
        xy_ang -= self.truth_yaw
        xy_ang = normalize_angle(xy_ang)
        # print(xy_ang)
        # print(angles)
        # exit()
        if xy_ang-.1 > angles[-1] or xy_ang+.1 < angles[0]:
            return False
        if dist < self.range:
            return True
        return False

    def angle_callback(self,msg):
        #read in from the visual publisher to know where it is looking
        #determine the angle range it has viewed
        angle_increment = msg.angle_increment
        angles = [-np.pi+angle_increment*i for i in range(len(msg.ranges))]
        current_angles = []
        for i in range(len(msg.ranges)):
            if msg.ranges[i]!=-1:
                current_angles.append(angles[i])
        self.not_seen(current_angles)

    def not_seen(self,angles):
        for i in range(len(self.x_cords)):
            for j in range(len(self.y_cords)):
                if self.space[i][j] <= 0.01:
                    if self.inview(self.x_cords[i],self.y_cords[j],angles):
                        self.space[i][j]=-1


    def detections(self,msg):
        found = False
        for target in msg.targets:
            if target.id != 'red_asset':
                continue
            else:
                found = True
                self.spotted(target)
    def spotted(self,target):
        range_to = target.range_m
        bearing = target.bearing_rad
        bearing += self.truth_yaw
        bearing = normalize_angle(bearing)
        x = self.truth_pos.x + range_to*np.cos(bearing)
        y = self.truth_pos.y + range_to*np.sin(bearing)
        x_close = None
        y_close = None
        for i in range(len(self.x_cords)):
            if np.norm(self.x_cords[i]-x) < .25:
                x_close = i
                break
        for i in range(len(self.y_cords)):
            if np.norm(self.y_cords[i]-y) < .25:
                y_close = i
                break
        self.space[x_close][y_close] = 1

    def truth_callback(self,msg):
        self.truth_pos = msg.pose.pose.position
        (r,p,self.truth_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    def update(self):
        self.space *= .95
    def run(self):
        rate = rospy.Rate(1)
        while self.truth_yaw_3==None or self.truth_yaw_4==None:
            rate.sleep()
        x_edges = [self.x_cords[0]+0.25*self.x_diff,self.x_cords[0]+0.75*self.x_diff]
        y_edges = [self.y_cords[0]+0.25*self.y_diff,self.y_cords[0]+0.75*self.y_diff]
        z_level = self.z
        sonar_settings = SonarSettings()
        sonar_settings.range = self.range
        sonar_settings.min_angle = -4
        sonar_settings.max_angle = 4
        self.set_sonar_3(sonar_settings)
        self.set_sonar_4(sonar_settings)
        waypoints = [[x_edges[0],y_edges[0],z_level],[x_edges[1],y_edges[0],z_level],[x_edges[1],y_edges[1],z_level],[x_edges[0],y_edges[1],z_level]]
        curr_waypt_3 = 0
        curr_waypt_4 = 2
        while not rospy.is_shutdown():
            x_diff_3 = waypoints[curr_waypt_3][0]-self.truth_pos_3.x
            y_diff_3 = waypoints[curr_waypt_3][1]-self.truth_pos_3.y
            z_diff_3 = waypoints[curr_waypt_3][2]-self.truth_pos_3.z
            v_3 = Vector3(y_diff_3,x_diff_3,-z_diff_3)
            v_3 = normalize_velocity(v_3,0.4)
            ang_3 = np.arctan2(x_diff_3,y_diff_3)
            # self.shv_3(ang_3*(180/np.pi),v_3)
            dist_3 = math.sqrt(x_diff_3**2+y_diff_3**2+z_diff_3**2)
            if dist_3 < 0.5:
                curr_waypt_3 = (curr_waypt_3+1)%4
            print("Dist_3 error: "+ str(dist_3))

            x_diff_4 = waypoints[curr_waypt_4][0]-self.truth_pos_4.x
            y_diff_4 = waypoints[curr_waypt_4][1]-self.truth_pos_4.y
            z_diff_4 = waypoints[curr_waypt_4][2]-self.truth_pos_4.z
            v_4 = Vector3(y_diff_4,x_diff_4,-z_diff_4)
            v_4 = normalize_velocity(v_4,0.4)
            ang_4 = np.arctan2(x_diff_4,y_diff_4)
            # self.shv_4(ang_4*(180/np.pi),v_4)
            dist_4 = math.sqrt(x_diff_4**2+y_diff_4**2+z_diff_4**2)
            if dist_4 < 0.5:
                curr_waypt_4 = (curr_waypt_4+1)%4
            print("Dist_4 error: "+ str(dist_4))
            print('')
            np.set_printoptions(precision=3)
            print(self.space[:][:])
            print('')


            self.update()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("search_node")
    volume = None
    with open('hello.yaml') as file:
        volume = yaml.load(file, Loader=yaml.FullLoader)
    v = Volume(volume)
    space = Space(v)
    s3 = Search(space,'/bluerov2_3')
    s4 = Search(space,'/bluerov2_4')
    s3.run()
    s4.run()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()