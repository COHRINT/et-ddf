#!/usr/bin/env python
"""   Node that allows makes the blue rovs search the space   """
from __future__ import division
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from minau.msg import SonarTargetList, SonarTarget
from minau.srv import ArmControl, SetHeadingVelocity
from etddf.srv import Detection 
from std_msgs.msg import Int16
import scipy.stats
import matplotlib.pyplot as plt
import tf
import math
import copy
import perlin
import os
import space


PLOTTING = True


#Open up config file to get the volume we are going to patrol
# volume_file = rospy.get_param("~volume_file")
def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

sonar_range = 10
twenty_grad = (20 * np.pi/200.0)



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
  

class Search:
    def __init__(self,space,name,half):
        #0 for top half, 1 for bottom half
        self.half = half
        self.z = self.half-1
        #name in format '/bluerov2_X'
        self.bel = space
        self.need_plot = False
        self.name = name
        self.range = 10
        self.yaw = None
        self.angle_start = None
        self.angle_count = 0
        self.angle_scan_begin = 0
        self.step = 1
        self.red_found = 0
        self.x = None
        self.waypoint = None
        self.lawn = rospy.get_param("~lawn",False)
        self.custody_dist = rospy.get_param("~custody",1)
        self.visited_x = []
        self.visited_y = []
        self.vel = rospy.get_param("~vel",0.4)


        #Make sure both rovs are armed
        rospy.wait_for_service(self.name+'/uuv_control/arm_control')
        self.arm_control = rospy.ServiceProxy(self.name+'/uuv_control/arm_control', ArmControl)
        resp1 = self.arm_control()
        self.prev_dist = 0

        #Set up services to set the velocity
        rospy.wait_for_service(self.name+'/uuv_control/set_heading_velocity')
        self.shv = rospy.ServiceProxy(self.name+'/uuv_control/set_heading_velocity', SetHeadingVelocity)
        rospy.wait_for_service(self.name+'/red_asset_detect_angle')
        self.detection_angle = rospy.ServiceProxy(self.name+'/red_asset_detect_angle',Detection)
        
        #Subscribes to the necisary topics for the search node for each rov
        rospy.Subscriber(self.name+"/strapdown/estimate",Odometry,self.ownship_callback)
        rospy.Subscriber(self.name+"/sonar_processing/target_list",SonarTargetList,self.detections)
        rospy.Subscriber(self.name+"/sonar_angle",Int16,self.angle_callback)
        self.first = True
        self.angle = 0
        
    def ownship_callback(self,msg):
        """Takes in ownship estimate to be used as it searches the space

        Args:
            msg ([Odometry]): The ownship estimate of postion
        """
        self.position = msg.pose.pose.position
        (r,p,self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def angle_callback(self,msg):
        """Subscriber to the angle of the sonar for the blue rovs so it knows what area it is scanning and can use
        the measurement to update the bayes filter.

        Args:
            msg (Int): Takes in teh degree measurement the sonar angle is at
        """
        if self.yaw==None:
            return
        ang = msg.data + (self.yaw*180.0/np.pi)
        ang = ang % 360
        

        self.angle = int(round((ang) / 18.0))
        
        x = int(round(self.position.x))
        y = int(round(self.position.y))
        self.bel.bel = self.bel.bayes(self.bel.bel,[x,y,self.angle],self.bel.p_dynm,self.bel.p_obs)
        # self.plot_continuous()

    def detections(self,msg):
        """Looks through sonar detections and determines if one is the red asset and if so continues to the spotted procedure

        Args:
            msg (SonarTargetList): List of sonar targets that have been spotted
        """
        for target in msg.targets:
            print(target.id)
            print(target.id[:9])
            if target.id[:9] == 'red_actor':
                self.red_found = 0
                self.detection_angle(self.angle)
                self.spotted(target)
                return
        self.red_found += 1
        
    
    def spotted(self,target):
        """If the red asset was spotted the blue rov points towards it and goes forward

        Args:
            target (SonarTarget): The detection of the red asset
        """
        print('Spotted')
        print(self.name)
        range_to = target.range_m
        bearing = target.bearing_rad
        bearing += self.yaw
        bearing = normalize_angle(bearing)
        x = self.position.x + range_to*np.cos(bearing)
        y = self.position.y + range_to*np.sin(bearing)
        # print(self.waypoint)
        self.waypoint = [x,y,self.position.z]

        x_close = None
        y_close = None
        for i in range(len(self.bel.x_cords)):
            if np.linalg.norm(self.bel.x_cords[i]-x) < .5:
                x_close = i
                break
        for i in range(len(self.bel.y_cords)):
            if np.linalg.norm(self.bel.y_cords[i]-y) < .5:
                y_close = i
                break
        # Updates bayes filter
        self.bel.bel[x_close][y_close] = 0
        self.bel.bel = self.bel.bel*0.7/(self.bel.bel.sum())
        self.bel.bel[x_close][y_close] = 0.3
        self.need_plot = True

    def new_waypoint(self):
        """
        Generates a new waypoint based on beleif. If in lawn mode then it just goes back and forth along the space.
        """
        if not self.lawn:
            if self.half == 0:
                dir = 1
            else:
                dir = -1
            max_x = 0
            max_y = dir
            total_max = 0
            for i in range(len(self.bel.x_cords)):
                for j in range(int(len(self.bel.y_cords)/2)):
                    total = 0
                    # for k in range(21):
                    #     if k+i-10 >=0 and k+i-10 < len(self.bel.x_cords):
                    #         total += self.bel.bel[k+i-10,dir*(j+1)]
                    # for k in range(21):
                    #     if dir*(j+1) + k -10 >=0 and dir*(j+1) + k -10 <len(self.bel.y_cords):
                    #         total+= self.bel.bel[i,dir*(j+1) + k -10]
                    # if total > total_max:
                    if self.bel.bel[i,dir*(j+1)] > self.bel.bel[max_x,max_y]:
                        if self.waypoint == None or (self.bel.x_cords[i] != self.waypoint[0] or self.bel.y_cords[(dir*j+1)] != self.waypoint[1]):
                            max_x,max_y = i,dir*(j+1)
                        # total_max = total
            self.waypoint = [self.bel.x_cords[max_x],self.bel.y_cords[max_y],self.z]
        else:
            if self.x == None:
                self.x1 = self.bel.x_cords[0] +5
                self.x2 = self.bel.x_cords[-1] -5
                self.y1 = self.bel.y_cords[0]
                self.y2 = self.bel.y_cords[-1]
                y_diff = (self.y2-self.y1)/2
                self.moveX = True
                self.x = True
                self.y = True      #true for positive

                self.y1 = self.y1 + (y_diff)*self.half + 5
                self.y2 = self.y1 + y_diff -10
                self.waypoint = [self.x1,self.y1,self.z]
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


    def plot_continuous(self):
        """
        Plots belief of the space in real time
        """
        # print('trying to plot')
        X,Y = np.meshgrid(self.bel.x_cords,self.bel.y_cords)
        cp = plt.contourf(X, Y, self.bel.bel.transpose())
        ax.set_title('Contour Plot')
        plt.pause(0.00001)


    def run(self):
        """
        Finds diffenence in waypoint location and current location and moves rov toward waypoint
        """
        rate = rospy.Rate(1)
        while self.yaw==None:
            rate.sleep()
        #first waypoint
        if self.first:
            self.new_waypoint()
            print('New Waypoint for ' + self.name+' : ')
            # print(self.angle)
            print(self.waypoint)
            self.need_plot = True
            self.first = False
        x_diff = self.waypoint[0]-self.position.x
        y_diff = self.waypoint[1]-self.position.y
        z_diff = self.waypoint[2]-self.position.z
        v = Vector3(y_diff,x_diff,-z_diff)
        v = normalize_velocity(v,self.vel)
        ang = np.arctan2(x_diff,y_diff)
        dist = math.sqrt(x_diff**2+y_diff**2)
        # If the bluerov is close to the red asset it just points toward it and stays still
        if self.red_found <  8 and dist < self.custody_dist:
            print('I have custody!')
            v = Vector3(0,0,0)
            self.shv(ang*(180/np.pi),v)
        #if bluerov gets within 0.5 meters of waypoint a new waypoint is generated
        elif dist < 0.5:
            print(dist)
            print(self.waypoint)
            print(self.position)
            self.need_plot = True
            self.new_waypoint()
            print('New Waypoint for ' + self.name+' : ')
            # print(self.angle)
            print(self.waypoint)
            self.angle_start=None
        #else the rovs velocity is set so it moves toward the waypoint
        else:
            self.shv(ang*(180/np.pi),v)


if __name__ == "__main__":
    rospy.init_node("search_node")
    x_dim = rospy.get_param("~x")
    y_dim = rospy.get_param("~y")
    space = space.Space(x_dim,y_dim)
    s3 = Search(space,'/bluerov2_3',0)
    s4 = Search(space,'/bluerov2_4',1)
    initial_time = rospy.get_rostime()
    print(initial_time)
    rate = rospy.Rate(20)
    if PLOTTING:
        fig,ax = plt.subplots()
    while (not rospy.is_shutdown()):
        s3.run()
        s4.run()
        if PLOTTING:
            s3.plot_continuous()
        rate.sleep()
    print(rospy.get_rostime()-initial_time)

