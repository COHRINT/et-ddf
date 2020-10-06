#!/usr/bin/env python
"""
Contains class that does repeat testing with the configurations included in the input file. 
"""
import rospy
import sys
import csv
import pprint
import random, shutil
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
from std_msgs.msg import Bool


class repeatTester:
    """Automatically repeats the tests according to the configurations in the csv file"""
    def __init__(self):
        """Reads in the csv file and stores the information in variables belonging to the class to set up for the testing.
        Arguments:
            cFile {string} -- The csv file name that describes the configuations for the test.
        """
        self.data_loc = rospy.get_param("~data_loc")
        self.cFile = rospy.get_param("~file")
        #reads the csv in as a dictionary, removing all whitespace
        reader = csv.DictReader(
            open(self.cFile)
        )
        # remove leading and trailing whitespace from all values
        reader = (
            dict((k.strip(), v.strip()) for k, v in row.items() if v) for row in reader)
        self.config = []
        for row in reader:
            self.config.append(row)

        self.first_3 = [10,10,-1]
        self.first_4 = [-10,10,-1]
        self.first_5 = [-10,-10,-1]
        self.first_6 = [10,-10,-1]

        #extracts data and asigns it to variables
        self.config_names=[]
        self.lawn = []
        self.DVL = []
        self.custody = []
        for i in range(len(self.config)):
            self.config_names.append(self.config[i]['Test_Name'])
            self.lawn.append(self.config[i]['Lawn_Mode'])
            self.DVL.append(self.config[i]['DVL_Data'])
            self.custody.append(float(self.config[i]['Custody_Dist']))
        self.test_group_name = self.config[0]['Test_Group_Name']
        self.num_groups = int(self.config[0]['Number_Tests_dep'])
        self.num_configs = len(self.config)
        self.total_tests = self.num_groups*self.num_configs
        self.dim_x = float(self.config[0]['Map_Dim_x_dep'])
        self.dim_y = float(self.config[0]['Map_Dim_y_dep'])
        self.red_vel = float(self.config[0]['Red_Vel_dep'])
        self.blue_vel = float(self.config[0]['Blue_Vel_dep'])
        self.printIntro()
        self.generateWay()
        self.Done = False

        rospy.Subscriber("/red_actor_1/done",Bool,self.finished_callback)

    def finished_callback(self,msg):
        self.Done = True
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
        # print('Expected Duration: '+self.time()+'\n\n')
    def waypoint(self):
        """Generates random waypoint pairs so the red asset traverses the whole space

        Returns:
            [[x,y,z],[x,y,z]]: Pair of waypoints for red asset
        """
        #[1,2,3,4] --> [up,down,left,right]
        dir = random.randint(1,4)
        waypt = []
        if dir == 1:
            waypt.append([random.random()*2*self.dim_x-self.dim_x,random.random()*5+self.dim_y,-1])
            waypt.append([random.random()*2*self.dim_x-self.dim_x,-(random.random()*5+self.dim_y),-1])
            return waypt
        if dir == 2:
            waypt.append([random.random()*2*self.dim_x-self.dim_x,-(random.random()*5+self.dim_y),-1])
            waypt.append([random.random()*2*self.dim_x-self.dim_x,(random.random()*5+self.dim_y),-1])
            return waypt
        if dir == 3:
            waypt.append([-(random.random()*5+self.dim_x),random.random()*self.dim_y*2-self.dim_y,-1])
            waypt.append([(random.random()*5+self.dim_x),(2*random.random()*self.dim_y-self.dim_y),-1])
            return waypt
        if dir == 4:
            waypt.append([(random.random()*5+self.dim_x),random.random()*self.dim_y*2-self.dim_y,-1])
            waypt.append([-(random.random()*5+self.dim_x),(2*random.random()*self.dim_y-self.dim_y),-1])
            return waypt
    def generateWay(self):
        """
        Generates the waypoints for the red asset and writes them to csv files.
        """
        #makes the waypoints into csv files that other scripts will be able to use
        self.first_red = []
        for i in range(self.num_groups):
            waypoints = []
            waypt = self.waypoint()
            waypoints.append(waypt[0])
            waypoints.append(waypt[1])
            self.first_red.append(waypoints[0])
            fileName = 'waypoints_'+str(i)+'.csv'
            fileName = self.data_loc + '/waypoints/'+fileName
            f = open(fileName,'w')
            for j in range(2):
                f.write(str(waypoints[j][0])+','+str(waypoints[j][1])+','+str(waypoints[j][2])+'\n')
            f.close()
    def setup_config(self,config_num):
        """
        Based on the configuration this either enables or disables different sensors.
        Arguments:
            config_num {int} -- takes in an integer of the config number
        """
        include_dvl = False
        if (self.DVL[config_num]).lower()=='true':
            include_dvl = True
        #this switches to the correct directory and reads in the yaml
        os.chdir(self.data_loc)
        os.chdir("../../config")
        with open('etddf.yaml') as f:
            etddf = yaml.load(f, Loader=yaml.FullLoader)

        if include_dvl:
            etddf["measurement_topics"]["dvl"] = "dvl"
        else:
            etddf["measurement_topics"]["dvl"] = "None"

        # #this writes the changes to the file
        with open('etddf_temp.yaml', 'w') as f:
            yaml.dump(etddf, f)
        os.chdir("../scripts")
    
    def teleport(self, idx):
        """Teleports assets to starting positions

        Args:
            idx (group number): Changes which starting position the red asset will start at
        """
        set_model_state('bluerov2_3',self.first_3)
        set_model_state('bluerov2_4',self.first_4)
        set_model_state('bluerov2_5',self.first_5)
        set_model_state('bluerov2_6',self.first_6)
        set_model_state('red_actor_1',self.first_red[idx])

    def run(self):
        """
        This is the function that runs all the tests and bags all the data.
        """
        #this is where the bagged data will be stored
        dirTo = self.data_loc+'/'+self.test_group_name
        os.mkdir(dirTo)
        shutil.copy(self.cFile,dirTo)
        for j in range(self.num_configs):
            self.setup_config(j)
            for i in range(self.num_groups):
                print('\n\nExecuting '+self.test_group_name+'/'+self.config_names[j]+' #'+str(i+1)+' ('+str((i+1)+self.num_groups*j)+'/'+str(self.total_tests)+')\n\n')
                self.teleport(i)

                FNULL = open(os.devnull, 'w')

                pos3 = 'pos3:='+str(self.first_3)
                pos4 = 'pos4:='+str(self.first_4)
                pos5 = 'pos5:='+str(self.first_5)
                pos6 = 'pos6:='+str(self.first_6)
                args2 = ['roslaunch','etddf','uuv_etddf.launch',pos3,pos4,pos5,pos6]

                fileFor3 = self.data_loc+'/waypoints/'+'waypoints_'+str(i)+'.csv'
                args3 = ['rosrun','etddf','waypoint_move.py','__ns:=red_actor_1',fileFor3,'_vel:='+str(self.red_vel)]

                args4 = ['rosrun','etddf','search.py','_x:='+str(self.dim_x),'_y:='+str(self.dim_y),'_vel:='+str(self.blue_vel),'_lawn:='+str(self.lawn[j]),'_custody:='+str(self.custody[j])]

                bagfile_name = self.config_names[j]+'_'+str(i+1)
                args5 = 'rosbag record -O '+bagfile_name+' /bluerov2_3/pose_gt \
                                                           /bluerov2_4/pose_gt \
                                                           /bluerov2_5/pose_gt \
                                                           /bluerov2_6/pose_gt \
                                                           /red_actor_1/pose_gt \
                                                           /bluerov2_3/etddf/estimate/network \
                                                           /bluerov2_4/etddf/estimate/network \
                                                           /bluerov2_5/etddf/estimate/network \
                                                           /bluerov2_6/etddf/estimate/network \
                                                           /bluerov2_3/strapdown/estimate \
                                                           /bluerov2_4/strapdown/estimate \
                                                           /bluerov2_5/strapdown/estimate \
                                                           /bluerov2_6/strapdown/estimate \
                                                           /bluerov2_3/sonar_processing/target_list \
                                                           /bluerov2_4/sonar_processing/target_list \
                                                           /bluerov2_5/sonar_processing/target_list \
                                                           /bluerov2_6/sonar_processing/target_list'

                proc2 = subprocess.Popen(args2)
                # proc2 = subprocess.Popen(args2,stdout=FNULL,stderr=subprocess.STDOUT)
                time.sleep(10)
                # proc5 = subprocess.Popen(args5,stdin=subprocess.PIPE, shell=True, cwd=dirTo)
                # proc3 = subprocess.Popen(args3,stdout=FNULL,stderr=subprocess.STDOUT)
                # proc4 = subprocess.Popen(args4,stdout=FNULL,stderr=subprocess.STDOUT)
                proc5 = subprocess.Popen(args5,stdin=subprocess.PIPE, shell=True, cwd=dirTo)
                proc3 = subprocess.Popen(args3)
                proc4 = subprocess.Popen(args4)
                
                rate = rospy.Rate(10)
                # rospy.sleep(self.mins_per*60)
                while not self.Done:
                    rate.sleep()

                terminate_ros_node("/record")
                proc3.terminate()
                proc4.terminate()
                proc2.terminate()
                time.sleep(10)
                self.Done = False

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