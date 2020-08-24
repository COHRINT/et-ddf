#!/usr/bin/env python
"""
Contains class that does repeat testing with the configurations included in the command line input file. 
"""
import rospy
import sys
import csv
import pprint
import random
import os, signal
import subprocess
import time
import argparse
import yaml
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class repeatTester:
    """Automatically repeats the tests according to the configurations in the csv file"""
    def __init__(self):
        """Reads in the csv file and stores the information in variables belonging to the class to set up for the testing.
        Arguments:
            cFile {string} -- The csv file name that describes the configuations for the test.
        """
        self.data_loc = rospy.get_param("~data_loc")
        cFile = rospy.get_param("~file")
        #reads the csv in as a dictionary, removing all whitespace
        reader = csv.DictReader(
            open(cFile)
        )
        # remove leading and trailing whitespace from all values
        reader = (
            dict((k.strip(), v.strip()) for k, v in row.items() if v) for row in reader)
        self.config = []
        for row in reader:
            self.config.append(row)

        self.first_3 = [0,6,-1]
        self.first_4 = [0,-6,-1]

        #extracts data and asigns it to variables
        self.config_names=[]
        self.IMU = []
        self.Sonar = []
        self.DVL = []
        for i in range(len(self.config)):
            self.config_names.append(self.config[i]['Test_Name'])
            self.IMU.append(self.config[i]['IMU_Data'])
            self.Sonar.append(self.config[i]['Sonar_Data'])
            self.DVL.append(self.config[i]['DVL_Data'])
        self.test_group_name = self.config[0]['Test_Group_Name']
        self.num_groups = int(self.config[0]['Number_Tests_dep'])
        self.num_configs = len(self.config)
        self.total_tests = self.num_groups*self.num_configs
        self.mins_per = float(self.config[0]['Mins_Per_Test_dep'])
        self.tot_mins = self.mins_per*self.total_tests
        self.remaining_time = self.tot_mins
        self.dim_x = float(self.config[0]['Map_Dim_x_dep'])
        self.dim_y = float(self.config[0]['Map_Dim_y_dep'])
        self.dim_z = float(self.config[0]['Map_Dim_z_dep'])
        self.num_waypts = int(self.config[0]['Number_Red_WayPts_dep'])
        self.printIntro()
        self.generateWay()
    def time(self):
        """Takes the time and converts it to a format that can be printed.
        Returns:
            [String] -- a string of the time of format XXh XXmin
        """
        t_formated = str(int(self.remaining_time/60))+'h '+str(self.remaining_time%60)+'min'
        return t_formated
    def printIntro(self):
        """Simply prints the intro"""        
        #prints initial information
        print('\n\nTest groups: '+str(self.num_groups))
        print('Total tests to execute: '+str(self.total_tests))
        print('Expected Duration: '+self.time()+'\n\n')
    def waypoint(self,inside):
        waypt = [random.random()*2*self.dim_x-self.dim_x,random.random()*self.dim_y*2-self.dim_y,-1]
        if inside:
            return waypt
        if waypt[0] > 0:
            waypt[0]+= self.dim_x
        else:
            waypt[0]-= self.dim_x
        if waypt[1] > 0:
            waypt[1]+= self.dim_y
        else:
            waypt[1]-= self.dim_y
        return waypt
    def generateWay(self):
        """
        Generates the waypoints for the red asset and writes them to csv files.
        """
        #makes the waypoints into csv files that other scripts will be able to use
        self.first_red = []
        for i in range(self.num_groups):
            waypoints = []
            waypoints.append(self.waypoint(False))
            self.first_red.append(waypoints[0])
            for j in range(self.num_waypts):
                waypoints.append(self.waypoint(True))
            waypoints.append(self.waypoint(False))
            fileName = 'waypoints_'+str(i)+'.csv'
            fileName = self.data_loc + '/waypoints/'+fileName
            f = open(fileName,'w')
            for j in range(self.num_waypts+2):
                f.write(str(waypoints[j][0])+','+str(waypoints[j][1])+','+str(waypoints[j][2])+'\n')
            f.close()
    def setup_config(self,config_num):
        """
        Based on the configuration this either enables or disables different sensors.
        Arguments:
            config_num {int} -- takes in an integer of the config number
        """
        include_imu = False
        include_sonar = False
        include_dvl = False
        if (self.IMU[config_num]).lower()=='true':
            include_imu=True
        if (self.Sonar[config_num]).lower()=='true':
            include_sonar=True
        if (self.DVL[config_num]).lower()=='true':
            include_dvl = True
        #this switches to the correct directory and reads in the yaml
        os.chdir(self.data_loc)
        os.chdir("../../config")
        with open('etddf.yaml') as f:
            etddf = yaml.load(f, Loader=yaml.FullLoader)
        # print(etddf)
        if include_imu:
            # #using this dictionary format you can change any of the attributes you want
            etddf["measurement_topics"]["imu_ci"]="odometry/filtered"
        else:
            etddf["measurement_topics"]["imu_ci"]="None"
        if include_sonar:
            etddf["measurement_topics"]["sonar"]="sonar_processing/target_list"
        else:
            etddf["measurement_topics"]["sonar"]="None"
        if include_dvl:
            etddf["measurement_topics"]["dvl"] = "dvl"
        else:
            etddf["measurement_topics"]["dvl"] = "None"

        # #this writes the changes to the file
        with open('etddf_temp.yaml', 'w') as f:
            yaml.dump(etddf, f)
        os.chdir("../scripts")
        # print(etddf)
        # exit()
    
    def teleport(self, idx):
        set_model_state('bluerov2_3',self.first_3)
        set_model_state('bluerov2_4',self.first_4)
        set_model_state('red_actor_1',self.first_red[idx])

    def run(self):
        """
        This is the function that runs all the tests and bags all the data.
        """
        #this is where all the scripts and launch files are ran
        print(1)
        dirTo = self.data_loc+'/'+self.test_group_name
        os.mkdir(dirTo)
        for j in range(self.num_configs):
            print(2)
            self.setup_config(j)
            for i in range(self.num_groups):
                print('\n\nExecuting '+self.test_group_name+'/'+self.config_names[j]+' #'+str(i+1)+' ('+str((i+1)+3*j)+'/'+str(self.total_tests)+') | Time Remaining: '+self.time()+'\n\n')
                # exit()
                print('teleporing')
                self.teleport(i)
                print('uuv and etddf')
                # self.remaining_time -= self.mins_per
                # time.sleep(10)
                pos3 = 'pos3:='+str(self.first_3)
                pos4 = 'pos4:='+str(self.first_4)
                args2 = ['roslaunch','etddf','uuv_etddf.launch',pos3,pos4]
                # FNULL = open(os.devnull, 'w')
                proc2 = subprocess.Popen(args2)
                time.sleep(10)
                # # proc2 = subprocess.Popen(args2,stdout=FNULL,stderr=subprocess.STDOUT)
                # time.sleep(10)
                # # ,'/bluerov2_4/pose_gt','/bluerov2_3/etddf/estimate/network','/bluerov2_4/etddf/estimate/network'
                # bagfile_name = self.config_names[j]+'_'+str(i+1)
                # args5 = 'rosbag record -O '+bagfile_name+' /bluerov2_3/pose_gt /bluerov2_4/pose_gt /bluerov2_3/etddf/estimate/network /bluerov2_4/etddf/estimate/network /bluerov2_3/etddf/statistics /bluerov2_4/etddf/statistics'
                fileFor3 = self.data_loc+'/waypoints/'+'waypoints_'+str(i)+'.csv'
                print('waypoint')
                args3 = ['rosrun','etddf','waypoint_move.py','__ns:=red_actor_1',fileFor3]
                # # proc3 =  subprocess.Popen(args3,stdout=FNULL,stderr=subprocess.STDOUT)
                proc3 =  subprocess.Popen(args3)
                time.sleep(50)
                return
                # fileFor4 = 'waypoints_'+str(i+self.num_groups)+'.csv'
                # args4 = ['rosrun','etddf','waypoint_move.py','__ns:=bluerov2_4',fileFor4]
                # # proc4 = subprocess.Popen(args4,stdout=FNULL,stderr=subprocess.STDOUT)
                # proc4 = subprocess.Popen(args4)
                # # print(dirTo)
                # proc5 = subprocess.Popen(args5,stdin=subprocess.PIPE, shell=True, cwd=dirTo)
                # print('Hello')
                # rospy.sleep(self.mins_per*60)
                # print('Goodbye')
                # # time.sleep(30)
                # terminate_ros_node("/record")
                # # time.sleep(10)
                # proc3.terminate()
                # proc4.terminate()
                # proc2.terminate()
        print('All tests complete')
        print('Data located in data/'+self.test_group_name)

def set_model_state(model_name, pos):
    """Moves a model in gazebo
    Arguments:
        model_name {String} -- The name of what you want to move
        pos {[float,float,float]} -- Position of where you want to move it to
    """
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    rospy.wait_for_service('/gazebo/set_model_state')    
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...    
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover(ModelState(model_name, pose, Twist(), "world") )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(0.1)
                

def terminate_ros_node(s):
    """This is used to stop the bagging correctly.
    Arguments:
        s {string} -- the prefix of the process you want to stop
    """
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)



if __name__ == "__main__":
    rospy.init_node('repeat_tester')
    rt = repeatTester()
    rt.run()