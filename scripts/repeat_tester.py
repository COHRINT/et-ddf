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


class repeatTester:
    """Automatically repeats the tests according to the configurations in the csv file"""
    def __init__(self,cFile):
        """Reads in the csv file and stores the information in variables belonging to the class to set up for the testing.

        Arguments:
            cFile {string} -- The csv file name that describes the configuations for the test.
        """
        #reads the csv in as a dictionary, removing all whitespace
        reader = csv.DictReader(
            open(cFile)
        )
        # remove leading and trailing whitespace from all values
        reader = (
            dict((k.strip(), v.strip()) for k, v in row.items() if v) for row in reader)
        rows = []
        for row in reader:
            rows.append(row)
        # pprint.pprint(rows)
        #extracts data and asigns it to variables
        self.config_names=[]
        self.IMU = []
        for i in range(len(rows)):
            self.config_names.append(rows[i]['Test_Name'])
            self.IMU.append(rows[i]['IMU_Data'])
        self.test_group_name = rows[0]['Test_Group_Name']
        self.num_groups = int(rows[0]['Number_Tests_dep'])
        self.num_configs = len(rows)
        self.total_tests = self.num_groups*self.num_configs
        self.mins_per = float(rows[0]['Mins_Per_Test_dep'])
        self.tot_mins = self.mins_per*self.total_tests
        self.remaining_time = self.tot_mins
        self.dim_x = float(rows[0]['Map_Dim_x_dep'])
        self.dim_y = float(rows[0]['Map_Dim_y_dep'])
        self.dim_z = float(rows[0]['Map_Dim_z_dep'])
        self.printIntro()
        self.generateWay()
        rospy.init_node('RepeatTester')
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
    def generateWay(self):
        """
        Generates the waypoints and writes them to csv files.
        """
        #generates random waypoints for each group and robot
        self.waypoints = [[[random.random()*2*self.dim_x-self.dim_x,random.random()*self.dim_y*2-self.dim_y,-random.random()*self.dim_z]for j in range(50)]for i in range(self.num_groups*2)]
        #makes the waypoints into csv files that other scripts will be able to use
        self.first = []
        for i in range(self.num_groups*2):
            fileName = 'waypoints_'+str(i)+'.csv'
            f = open(fileName,'w')
            for j in range(50):
                if j == 0:
                    self.first.append('['+str(self.waypoints[i][j][0])+','+str(self.waypoints[i][j][1])+','+str(self.waypoints[i][j][2])+']')
                f.write(str(self.waypoints[i][j][0])+','+str(self.waypoints[i][j][1])+','+str(self.waypoints[i][j][2])+'\n')
            f.close()
    def setup_imu(self,config_num):
        """
        Based on the configuration this either enables or disables imu data.

        Arguments:
            config_num {int} -- takes in an integer of the config number
        """
        include_imu = False
        if self.IMU[config_num]=='true':
            include_imu=True
        #this switches to the correct directory and reads in the yaml
        os.chdir("../config")
        with open('etddf.yaml') as f:
            etddf = yaml.load(f, Loader=yaml.FullLoader)
        # print(etddf)
        if include_imu:
            # #using this dictionary format you can change any of the attributes you want
            etddf["measurement_topics"]["imu_ci"]="odometry/filtered"
        else:
            etddf["measurement_topics"]["imu_ci"]="odometry/filtered"
        # #this writes the changes to the file
        with open('example.yaml', 'w') as f:
            yaml.dump(etddf, f)
        os.chdir("../scripts")
        # print(etddf)
        # exit()
    

    def run(self):
        """
        This is the function that runs all the tests and bags all the data.
        """
        #this is where all the scripts and launch files are ran
        d = os.getcwd()+'/data/'+self.test_group_name
        os.mkdir(d)
        for j in range(self.num_configs):
            self.setup_imu(j)
            for i in range(self.num_groups):
                print('\n\nExecuting '+self.test_group_name+'/'+self.config_names[j]+' #'+str(i+1)+' ('+str((i+1)+3*j)+'/'+str(self.total_tests)+') | Time Remaining: '+self.time()+'\n\n')
                dirTo = os.getcwd()+'/data/'+self.test_group_name+'/'+self.config_names[j]+'_'+str(i+1)
                # print(dirTo)
                os.mkdir(dirTo)
                # exit()
                self.remaining_time -= self.mins_per
                time.sleep(10)
                pos3 = 'pos3:='+self.first[i]
                pos4 = 'pos4:='+self.first[i+self.num_groups]
                args2 = ['roslaunch','etddf','uuv_etddf.launch',pos3,pos4]
                FNULL = open(os.devnull, 'w')
                proc2 = subprocess.Popen(args2)
                # proc2 = subprocess.Popen(args2,stdout=FNULL,stderr=subprocess.STDOUT)
                time.sleep(10)
                # ,'/bluerov2_4/pose_gt','/bluerov2_3/etddf/estimate/network','/bluerov2_4/etddf/estimate/network'
                args5 = 'rosbag record /bluerov2_3/pose_gt /bluerov2_4/pose_gt /bluerov2_3/etddf/estimate/network /bluerov2_4/etddf/estimate/network'
                fileFor3 = 'waypoints_'+str(i)+'.csv'
                args3 = ['rosrun','etddf','waypoint_move.py','__ns:=bluerov2_3',fileFor3]
                # proc3 =  subprocess.Popen(args3,stdout=FNULL,stderr=subprocess.STDOUT)
                proc3 =  subprocess.Popen(args3)
                fileFor4 = 'waypoints_'+str(i+self.num_groups)+'.csv'
                args4 = ['rosrun','etddf','waypoint_move.py','__ns:=bluerov2_4',fileFor4]
                # proc4 = subprocess.Popen(args4,stdout=FNULL,stderr=subprocess.STDOUT)
                proc4 = subprocess.Popen(args4)
                # print(dirTo)
                proc5 = subprocess.Popen(args5,stdin=subprocess.PIPE, shell=True, cwd=dirTo)
                print('Hello')
                rospy.sleep(self.mins_per*60)
                print('Goodbye')
                # time.sleep(30)
                terminate_ros_node("/record")
                # time.sleep(10)
                proc3.terminate()
                proc4.terminate()
                proc2.terminate()
    print('All tests complete')
    print('Data located in data/'+self.test_group_name)
                

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
    """
    Gets the file name and creates a repeat tester object.
    """
    #make sure arguments are correct and provide help info
    parser = argparse.ArgumentParser(description='Repeat Tester')
    parser.add_argument("testFilename", type=str, help="location of test file")
    args = parser.parse_args()
    configFile = args.testFilename
    rt = repeatTester(configFile)
    rt.run()
