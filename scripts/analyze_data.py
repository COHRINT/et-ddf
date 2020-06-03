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
        # print(self.bag_files)
        self.truth_poses3 = [[]for i in range(len(self.bag_files))]
        self.truth_poses4 = [[]for i in range(len(self.bag_files))]
        self.estimate3_poses4 = [[]for i in range(len(self.bag_files))]
        self.estimate3_poses3 = [[]for i in range(len(self.bag_files))]
        self.estimate4_poses4 = [[]for i in range(len(self.bag_files))]
        self.estimate4_poses3 = [[]for i in range(len(self.bag_files))]
        self.ratio = []
        for i in range(len(self.bag_files)):
            bag = rosbag.Bag(self.bag_files[i])
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_3/pose_gt']):
                self.truth_poses3[i].append(msg)
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_4/pose_gt']):
                self.truth_poses4[i].append(msg)
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_3/etddf/estimate/network']):
                for j in range(len(msg.assets)):
                    if msg.assets[j].name == 'bluerov2_3':
                        self.estimate3_poses3[i].append(msg.assets[j].odom)
                    else:
                        self.estimate3_poses4[i].append(msg.assets[j].odom)
            for topic, msg, t in bag.read_messages(topics=['/bluerov2_4/etddf/estimate/network']):
                for j in range(len(msg.assets)):
                    if msg.assets[j].name == 'bluerov2_3':
                        self.estimate4_poses3[i].append(msg.assets[j].odom)
                    else:
                        self.estimate4_poses4[i].append(msg.assets[j].odom)
            bag.close()
            self.ratio.append(len(self.truth_poses3[i])//len(self.estimate3_poses3[i]) - 3) 
        # print(self.ratio)
    def run(self):
        pose3Diff3 = [[]for i in range(len(self.bag_files))]
        pose3Diff4 = [[]for i in range(len(self.bag_files))]
        pose4Diff4 = [[]for i in range(len(self.bag_files))]
        pose4Diff3 = [[]for i in range(len(self.bag_files))]

        for k in range(len(self.bag_files)):
            i = 0
            j = 0
            timeDiff = abs(self.estimate3_poses3[k][0].header.stamp-self.truth_poses3[k][0].header.stamp)
            while i < len(self.estimate3_poses3[k]):
                tempTime = abs(self.estimate3_poses3[k][i].header.stamp-self.truth_poses3[k][j].header.stamp)
                if tempTime > timeDiff:
                    pose3Diff3[k].append(quadratureDiff(self.estimate3_poses3[k][i],self.truth_poses3[k][j-1]))
                    pose3Diff4[k].append(quadratureDiff(self.estimate3_poses4[k][i],self.truth_poses4[k][j-1]))   
                    i+=1
                    j+=self.ratio[k]
                    if i!=len(self.estimate3_poses3[k]):
                        timeDiff = abs(self.estimate3_poses3[k][i].header.stamp-self.truth_poses3[k][0].header.stamp)
                else:
                    j+=1
                    timeDiff = tempTime
            i = 0
            j = 0
            timeDiff = abs(self.estimate4_poses4[k][0].header.stamp-self.truth_poses4[k][0].header.stamp)
            while i < len(self.estimate4_poses4[k])-1:
                tempTime = abs(self.estimate4_poses4[k][i].header.stamp-self.truth_poses4[k][j].header.stamp)
                if tempTime > timeDiff:
                    pose4Diff3[k].append(quadratureDiff(self.estimate4_poses3[k][i],self.truth_poses3[k][j-1]))
                    pose4Diff4[k].append(quadratureDiff(self.estimate4_poses4[k][i],self.truth_poses4[k][j-1]))   
                    i+=1
                    j+=self.ratio[k]
                    if i!=len(self.estimate4_poses4[k]):
                        timeDiff = abs(self.estimate4_poses4[k][i].header.stamp-self.truth_poses4[k][0].header.stamp)
                else:
                    j+=1
                    timeDiff = tempTime
        # print(poseDiff)

        # print(len(pose3Diff3))
        # print(len(pose3Diff4))
        # print(len(pose4Diff4))
        # print(len(pose4Diff3))
        
        groups = [['control',[]]]
        for i in range(len(self.bag_files)):
            found = False
            for j in range(len(groups)):
                if groups[j][0]==self.bag_files[i][:-6]:
                    found = True
                    groups[j][1].append(i)
                    break
            if not found:
                groups.append([self.bag_files[i][:-6],[i]])
            
        
        for i in range(len(groups)):
            for j in range(len(groups[i][1])):
                idx = int(self.bag_files[groups[i][1][j]][-5])-1
                groups[i][1][j], groups[i][1][idx] = groups[i][1][idx], groups[i][1][j]
        
        

        diff33 = [[]for i in range(len(groups))]
        diff43 = [[]for i in range(len(groups))]
        diff34 = [[]for i in range(len(groups))]
        diff44 = [[]for i in range(len(groups))]

        for i in range(len(groups)):
            for j in range(len(groups[i][1])):
                diff33[i].append(np.average(pose3Diff3[groups[i][1][j]]))
                diff43[i].append(np.average(pose4Diff3[groups[i][1][j]]))
                diff34[i].append(np.average(pose3Diff4[groups[i][1][j]]))
                diff44[i].append(np.average(pose4Diff4[groups[i][1][j]]))
        
        
        
        
        



        ind = np.arange(len(groups))
        width = 0.3       
        plt.bar(ind-width, [i[0] for i in diff34], width, label='Trial 1')
        plt.bar(ind, [i[1] for i in diff34], width, label='Trial 2')
        plt.bar(ind + width, [i[2] for i in diff34], width,
            label='Trial 3')
        plt.xticks(ind, [i[0]for i in groups])
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