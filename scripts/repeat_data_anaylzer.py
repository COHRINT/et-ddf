#!/usr/bin/env python
from __future__ import division
import argparse
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import time

def quadratureDiff(A,B):
    """Finds the difference in position in length
    Arguments:
        A {Odometry} -- Either truth or estimate odometry
        B {Odometry} -- Either truth or estimate odometry
    Returns:
        int -- distance between truth and estimate position
    """
    pos_A = A.pose.pose.position
    pos_B = B.pose.pose.position
    x_diff = pos_A.x - pos_B.x
    y_diff = pos_A.y - pos_B.y
    z_diff = pos_A.z - pos_B.z
    return np.linalg.norm([x_diff, y_diff,z_diff])

class AnalyzeData:
    def __init__(self,data,x,y):
        """Reads in all the bag files and stores their data
        Arguments:
            data {string} -- location of bag files from scripts file
        """
        self.dim = [x,y]
        self.data_loc = os.getcwd()+'/'+data
        os.chdir(self.data_loc)
        self.bag_files = [i for i in os.listdir('.') if i[-3:]=='bag']
        self.truth_poses3 = [[]for i in range(len(self.bag_files))]
        self.truth_poses4 = [[]for i in range(len(self.bag_files))]
        self.truth_red = [[]for i in range(len(self.bag_files))]
        self.estimate3_poses4 = [[]for i in range(len(self.bag_files))]
        self.estimate3_poses3 = [[]for i in range(len(self.bag_files))]
        self.estimate3_red = [[]for i in range(len(self.bag_files))]
        self.estimate4_poses4 = [[]for i in range(len(self.bag_files))]
        self.estimate4_poses3 = [[]for i in range(len(self.bag_files))]
        self.estimate4_red = [[]for i in range(len(self.bag_files))]
        self.spotted3 = [[]for i in range(len(self.bag_files))]
        self.spotted4 = [[]for i in range(len(self.bag_files))]
        self.merge_spottings = [[]for i in range(len(self.bag_files))]
        self.ratio = []
        self.plot_count = 1
        # print(self.bag_files)
        # exit()
        for i in range(len(self.bag_files)):
            bag = rosbag.Bag(self.bag_files[i])
            #gets truth poses and ownship estimates
            self.truth_red[i] = self.extract_odometry('/red_actor_1/pose_gt',bag)
            self.truth_poses3[i] = self.extract_odometry('/bluerov2_3/pose_gt',bag)
            self.truth_poses4[i] = self.extract_odometry('/bluerov2_4/pose_gt',bag)
            self.estimate3_poses3[i] = self.extract_odometry('/bluerov2_3/strapdown/estimate',bag)
            self.estimate4_poses4[i] = self.extract_odometry('/bluerov2_4/strapdown/estimate',bag)

            #Gets the times we have spotted the red asset
            spotted3 = self.extract_detections('/bluerov2_3/sonar_processing/target_list',bag)
            spotted4 = self.extract_detections('/bluerov2_4/sonar_processing/target_list',bag)
            #Merges spottings together
            merged = self.merge(spotted3,spotted4)
            #Condenses these to the periods we have spotted the asset
            self.merge_spottings[i] = self.condense_merged_data(merged)

            #gets the estimate poses from etddf/estimate/network
            self.estimate3_poses4[i] = self.extract_odometry_network('/bluerov2_3/etddf/estimate/network',bag,'bluerov2_4')
            self.estimate4_poses3[i] = self.extract_odometry_network('/bluerov2_4/etddf/estimate/network',bag,'bluerov2_3')
            self.estimate3_red[i] = self.extract_odometry_network('/bluerov2_3/etddf/estimate/network',bag,'red_actor_1')
            self.estimate4_red[i] = self.extract_odometry_network('/bluerov2_4/etddf/estimate/network',bag,'red_actor_1')
            bag.close()

        #Gets the times the red asset is in the zone for each trial
        self.times_in = self.time_in_zone()
        #Gets [% of time tracked in zone, time to spot] for each trial
        self.tracking_data = [[]for i in range(len(self.bag_files))]
        for i in range(len(self.bag_files)):
            self.tracking_data[i] = self.get_tracking_data(self.merge_spottings[i],self.times_in[i])
        print(self.tracking_data)


        self.network_error_33 = [[]for i in range(len(self.bag_files))]
        self.network_error_44 = [[]for i in range(len(self.bag_files))]
        self.network_error_34 = [[]for i in range(len(self.bag_files))]
        self.network_error_43 = [[]for i in range(len(self.bag_files))]
        self.network_error_3r = [[]for i in range(len(self.bag_files))]
        self.network_error_4r = [[]for i in range(len(self.bag_files))]

        for i in range(len(self.bag_files)):
            self.network_error_33[i] = self.get_network_error(self.truth_poses3[i],self.estimate3_poses3[i])
            self.network_error_44[i] = self.get_network_error(self.truth_poses4[i],self.estimate4_poses4[i])
            self.network_error_34[i] = self.get_network_error(self.truth_poses4[i],self.estimate3_poses4[i])
            self.network_error_43[i] = self.get_network_error(self.truth_poses3[i],self.estimate4_poses3[i])
            self.network_error_3r[i] = self.get_network_error(self.truth_red[i],self.estimate3_red[i])
            self.network_error_4r[i] = self.get_network_error(self.truth_red[i],self.estimate4_red[i])
        print(self.network_error_3r)

        self.groups,self.group_names = self.get_groups()
        self.graph_network_error()

        self.graph_time()

        self.write_data()


    def graph_time(self):
        fig1, ax1 = plt.subplots(ncols=2,nrows=2,sharex='col', sharey='row',
                        gridspec_kw={'hspace': .2, 'wspace': 0})
        for i in range(len(self.groups)):
            data = []
            for j in range(len(self.groups[i])):
                data.append((self.tracking_data[self.groups[i][j]][1])/10**9)
            ax1[int(i/2),int(i%2)].hist(data,bins = 10)
            plt.xlabel('Time(s)')
            plt.ylabel('Frequency of Tests')
            ax1[int(i/2),int(i%2)].set_title(self.group_names[i])
            if int(i/2) == 1:
                ax1[int(i/2),int(i%2)].set_xlabel('Time(s)')
            if int(i%2) == 0:
                ax1[int(i/2),int(i%2)].set_ylabel('Frequency of Tests')
        fig1.suptitle('Time to Find Red Asset')
        fig1.savefig(self.data_loc+'/time_to_find.png')
        plt.close()

        fig1, ax1 = plt.subplots(ncols=2,nrows=2,sharex='col', sharey='row',
                        gridspec_kw={'hspace': .2, 'wspace': 0})
        bins = [10*i for i in range(11)]
        for i in range(len(self.groups)):
            data = []
            for j in range(len(self.groups[i])):
                data.append(self.tracking_data[self.groups[i][j]][0]*100)
            ax1[int(i/2),int(i%2)].hist(data, bins = bins)
            if int(i/2) == 1:
                ax1[int(i/2),int(i%2)].set_xlabel('Percent')
            if int(i%2) == 0:
                ax1[int(i/2),int(i%2)].set_ylabel('Frequency of Tests')
            ax1[int(i/2),int(i%2)].set_title(self.group_names[i])
        fig1.suptitle('Percent of Time Red Asset is Tracked in Zone')
        fig1.savefig(self.data_loc+'/percent_tracked.png')
        plt.close()
        fig1, ax1 = plt.subplots(ncols=2,nrows=2)

        labels = 'Found','Not Found'
        for i in range(len(self.groups)):
            data = np.array([0.0,0.0])
            for j in range(len(self.groups[i])):
                if self.tracking_data[self.groups[i][j]][0]*100 < 0.1:
                    data[1] += 1
                else:
                    data[0] += 1  
            data/=data.sum()
            data*=100
            
            ax1[int(i/2),int(i%2)].pie(data, labels=labels,autopct='%1.1f%%', shadow=True, startangle=140)
            ax1[int(i/2),int(i%2)].set_title(self.group_names[i])
        fig1.suptitle('Percent of Time Red Asset is Found with each Setting')
        fig1.savefig(self.data_loc+'/percent_found.png')
        plt.close()
        

        

    def get_groups(self):
        groups = []
        group_names = []
        for i in range(len(self.bag_files)):
            j = 5
            while self.bag_files[i][-j]!='_':
                j+=1
            found = False
            for k in range(len(group_names)):
                if group_names[k]== self.bag_files[i][:-j]:
                    groups[k].append(i)
                    found = True
                    break
            if not found:
                group_names.append(self.bag_files[i][:-j])
                groups.append([i])
        return groups,group_names
            
    def get_data(self,error,group):
        data = []
        for j in range(len(group)):
            if error[group[j]] != None:
                data.append(error[group[j]])
        return data

    def graph_network_error(self):
        bins_own = np.linspace(0,.3,11)
        bins_other = np.linspace(0,15,16)
        for i in range(len(self.groups)):
            fig1, ax1 = plt.subplots(ncols=3,nrows=2,sharex='col', sharey='row',
                        gridspec_kw={'hspace': .2, 'wspace': 0})
            data = self.get_data(self.network_error_33,self.groups[i])
            ax1[0,0].hist(data,bins = bins_own)
            ax1[0,0].set_title('3 Ownship')
            ax1[0,0].set_ylabel('Number of Tests')
            data = self.get_data(self.network_error_34,self.groups[i])
            ax1[0,1].hist(data,bins = bins_other)
            ax1[0,1].set_title('3 Est 4')
            data = self.get_data(self.network_error_3r,self.groups[i])

            ax1[0,2].hist(data,bins = bins_other)
            ax1[0,2].set_title('3 Est Red')

            data = self.get_data(self.network_error_44,self.groups[i])
            ax1[1,0].hist(data,bins = bins_own)
            ax1[1,0].set_title('4 Ownship')
            ax1[1,0].set_ylabel('Number of Tests')
            ax1[1,0].set_xlabel('Average Error(m)')
            data = self.get_data(self.network_error_43,self.groups[i])
            ax1[1,1].hist(data,bins = bins_other)
            ax1[1,1].set_title('4 Est 3')
            ax1[1,1].set_xlabel('Average Error(m)')
            data = self.get_data(self.network_error_4r,self.groups[i])
            ax1[1,2].hist(data,bins = bins_other)
            ax1[1,2].set_title('4 Est Red')
            ax1[1,2].set_xlabel('Average Error(m)')
            fig1.suptitle('Network Error for '+self.group_names[i])
            fig1.savefig(self.data_loc+'/network_error_'+self.group_names[i]+'.png')
            plt.close()


    def write_data(self):
        f = open(self.data_loc+'/data.csv','w')
        f.write('Test_Name,Time_to_Spot,Percent_Tracked,Avg_Ownship_Error3,Avg_Blue_Error3,Avg_Red_Error3,Avg_Ownship_Error4,Avg_Blue_Error4,Avg_Red_Error4\n')
        for i in range(len(self.bag_files)):
            f.write(self.bag_files[i][:-4]+','+str(self.tracking_data[i][1]/10**9)+','+str(self.tracking_data[i][0])+','+str(self.network_error_33[i])+','+str(self.network_error_34[i])+','+str(self.network_error_3r[i])+','+str(self.network_error_44[i])+','+str(self.network_error_43[i])+','+str(self.network_error_4r[i])+'\n')
        f.close()
    def get_network_error(self,truth,estimate):
        if len(estimate) == 0:
            print('Error!')
            return
        ratio = int((estimate[1][3]-estimate[0][3])/(truth[1][3]-truth[0][3]))
        ratio = max(ratio-3,0)
        error = []
        count = 0
        min_time_diff = truth[-1][3]-truth[0][3]
        for i in range(len(truth)):
            if count == len(estimate):
                break
            if abs(truth[i][3] - estimate[count][3]) > min_time_diff:
                if count+1 != len(estimate):
                    min_time_diff = abs(truth[i][3] - estimate[count+1][3])
                error.append(np.linalg.norm([truth[i-1][0]-estimate[count][0],truth[i-1][1]-estimate[count][1],truth[i-1][2]-estimate[count][2]]))
                count += 1
                i += ratio
                continue
            min_time_diff = abs(truth[i][3] - estimate[count][3])
        return np.average(error)
        


    def condense_merged_data(self,a):
        if a == []:
            return a
        times = []
        begin = a[0]
        end = -1
        for i in range(1,len(a)):
            if a[i]-a[i-1] > 5*10**9:
                times.append([begin,a[i-1]])
                begin = a[i]
        times.append([begin,a[-1]])
        return times

    def get_tracking_data(self,a,r):
        if a == []:
            return [0.0,r[1]-r[0]]
        total_time = r[1]-r[0]
        tracked_time = 0
        for i in range(len(a)):
            if a[i][1] < r[1] and a[i][0] > r[0]:
                tracked_time += a[i][1]-a[i][0]
            elif a[i][1] < r[1]:
                tracked_time += a[i][1]-r[0]
            elif a[i][0] > r[0]:
                tracked_time += r[1]-a[i][0]
            else:
                tracked_time += r[1]-r[0]
        return [float(tracked_time)/total_time,a[0][0]-r[0]]



    def merge(self, a, b):
        if len(a)==0:
            return b
        if len(b)==0:
            return a
        if a[0] <= b[0]:
            return [a[0]] + self.merge(a[1:],b)
        return [b[0]] + self.merge(a,b[1:])
    def extract_detections(self,name,bag):
        data = []
        for topic, msg, t in bag.read_messages(topics=[name]):
            for j in range(len(msg.targets)):
                if msg.targets[j].id[:9] == 'red_actor':
                    data.append(int(msg.header.stamp.secs*10**9 + msg.header.stamp.nsecs))
        return data
    
    def extract_odometry_network(self,name,bag,asset):
        data = []
        count = 0
        for topic, msg, t in bag.read_messages(topics=[name]):
            # if msg.assets != []:
            #     print(msg.assets)
            for j in range(len(msg.assets)):
                if msg.assets[j].name == asset:
                    # print(msg.assets[j].name)
                    count +=1
                    p = msg.assets[j].odom.pose.pose.position
                    data.append([p.x,p.y,p.z,int(msg.assets[j].odom.header.stamp.secs*10**9 + msg.assets[j].odom.header.stamp.nsecs)])
        return np.array(data)


    def extract_odometry(self,name,bag): 
        data = np.zeros(shape=(bag.get_message_count(name),4))
        count = 0
        for topic, msg, t in bag.read_messages(topics=[name]):
            p = msg.pose.pose.position
            data[count,:] = np.array([p.x,p.y,p.z,int(msg.header.stamp.secs*10**9 + msg.header.stamp.nsecs)])
            count+=1
        return data
    def time_in_zone(self):
        #for each bag file there will be a [time_in,time_out]
        time_in_zone = [[]for i in range(len(self.bag_files))]
        for i in range(len(self.bag_files)):
            t = [-1,-1]
            for j in range(len(self.truth_red[i])):
                if (-self.dim[0] < self.truth_red[i][j,0] < self.dim[0]) and (-self.dim[1] < self.truth_red[i][j,1] < self.dim[1]):
                    t[0] = self.truth_red[i][j,3]
                    break
            for j in range(-1,-len(self.truth_red[i]),-1):
                if (-self.dim[0] < self.truth_red[i][j,0] < self.dim[0]) and (-self.dim[1] < self.truth_red[i][j,1] < self.dim[1]):
                    t[1] = self.truth_red[i][j,3]
                    break
            time_in_zone[i] = t
        return time_in_zone

    def run(self):
        """
        Goes through data in bag file and extracts the data we want and graphs it.
        """
        pose3Diff3 = [[]for i in range(len(self.bag_files))]
        pose3Diff4 = [[]for i in range(len(self.bag_files))]
        pose4Diff4 = [[]for i in range(len(self.bag_files))]
        pose4Diff3 = [[]for i in range(len(self.bag_files))]

        for k in range(len(self.bag_files)):
            i = 0
            j = 0
            timeDiff = abs(self.estimate3_poses3[k][0].header.stamp-self.truth_poses3[k][0].header.stamp)
            while i < len(self.estimate3_poses3[k])-1 and i < len(self.estimate3_poses4[k])-1:
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
            while i < len(self.estimate4_poses4[k])-1 and i < len(self.estimate4_poses3[k])-1:
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




if __name__=="__main__":
    """
        Makes sure arguments work like they should and that you can ask for help if you don't know what the script wants.
    """
    parser = argparse.ArgumentParser(description='Analyze Data')
    parser.add_argument("location", type=str, help="location of data from scripts file(data/phaseX_x/)")
    parser.add_argument("x", type=int, help="Length in x dimension equalvelent to that put in the csv")
    parser.add_argument("y", type=int, help="Length in y dimension equalvelent to that put in the csv")
    args = parser.parse_args()
    data_location = args.location
    x = args.x
    y = args.y
    ad = AnalyzeData(data_location,x,y)