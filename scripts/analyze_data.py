#!/usr/bin/env python
from __future__ import division
import argparse
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def quadratureDiff(A,B):
    pos_A = A.pose.pose.position
    pos_B = B.pose.pose.position
    x_diff = pos_A.x - pos_B.x
    y_diff = pos_A.y - pos_B.y
    z_diff = pos_A.z - pos_B.z
    return np.linalg.norm([x_diff, y_diff,z_diff])

class AnalyzeData:
    def __init__(self,data):
        self.data_loc = os.getcwd()+'/'+data
        os.chdir(self.data_loc)
        self.bag_files = os.listdir('.')
        print(self.bag_files)
        self.truth_poses = [[]for i in range(len(self.bag_files))]
        self.estimate_poses = [[]for i in range(len(self.bag_files))]
        self.ratio = []
        for i in range(len(self.bag_files)):
            bag = rosbag.Bag(self.bag_files[i])
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_3/pose_gt']):
                self.truth_poses[i].append(msg)
            bag.close()
            bag = rosbag.Bag(self.bag_files[i])
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_3/etddf/estimate/bluerov2_3']):
                self.estimate_poses[i].append(msg)
            bag.close()
            self.ratio.append(len(self.truth_poses[i])//len(self.estimate_poses[i]) - 3) 
    def run(self):
        poseDiff = [[]for i in range(len(self.bag_files))]
        for k in range(len(self.bag_files)):
            i = 0
            j = 0
            timeDiff = abs(self.estimate_poses[k][0].header.stamp-self.truth_poses[k][0].header.stamp)
            num = 0
            while i < len(self.estimate_poses[k]):
                # print(num)
                num+=1
                tempTime = abs(self.estimate_poses[k][i].header.stamp-self.truth_poses[k][j].header.stamp)
                if tempTime > timeDiff:
                    poseDiff[k].append(quadratureDiff(self.estimate_poses[k][i],self.truth_poses[k][j-1]))   
                    i+=1
                    j+=self.ratio[k]
                    if i!=len(self.estimate_poses[k]):
                        timeDiff = abs(self.estimate_poses[k][i].header.stamp-self.truth_poses[k][0].header.stamp)
                else:
                    j+=1
                    timeDiff = tempTime
        # print(poseDiff)
        print(len(self.truth_poses))

        
        
        avgs = [np.average(poseDiff[0]),np.average(poseDiff[1]),np.average(poseDiff[2])]

        avgs0 = [avgs[0]for i in range(3)]
        avgs1 = [avgs[1]for i in range(3)]
        avgs2 = [avgs[2]for i in range(3)]


        ind = np.arange(3) 
        width = 0.3       
        plt.bar(ind-width, avgs0, width, label='Trial 1')
        plt.bar(ind, avgs1, width, label='Trial 2')
        plt.bar(ind + width, avgs2, width,
            label='Trial 3')
        plt.xticks(ind, ('Control','DVL','Sonar'))
        plt.legend(loc='best')
        plt.show()




            

        

#learn how to read in bag file and compare pose_gt with estimate



if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Analyze Data')
    parser.add_argument("location", type=str, help="location of data from scripts file(data/phaseX_x/)")
    args = parser.parse_args()
    data_location = args.location
    ad = AnalyzeData(data_location)
    ad.run()