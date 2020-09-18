#!/usr/bin/env python
import argparse
import matplotlib.pyplot as plt
import time
import numpy as np
import os
import csv


class Graph:
    def __init__(self,data_loc):
        self.data_loc = os.getcwd()+'/'+data_loc
        os.chdir(self.data_loc)
        self.data_csv = self.data_loc + '/data.csv'
        #reads the csv in as a dictionary, removing all whitespace
        reader = csv.DictReader(
            open(self.data_csv)
        )
        # remove leading and trailing whitespace from all values
        reader = (
            dict((k.strip(), v.strip()) for k, v in row.items() if v) for row in reader)
        self.data = []
        for row in reader:
            self.data.append(row)

        self.groups, self.group_names = self.get_groups()

        self.graph_data()

    def get_groups(self):
        groups = []
        group_names = []
        for i in range(len(self.data)):
            j = 1
            while self.data[i]['Test_Name'][-j]!='_':
                j+=1
            found = False
            for k in range(len(group_names)):
                if group_names[k]== self.data[i]['Test_Name'][:-j]:
                    groups[k].append(i)
                    found = True
                    break
            if not found:
                group_names.append(self.data[i]['Test_Name'][:-j])
                groups.append([i])
        return groups,group_names
    
    def get_data(self,name,group):
        data = []
        for i in range(len(group)):
            if self.data[group[i]][name]!='None':
                data.append(float(self.data[group[i]][name]))
        return data
    
    def get_percent_found(self,group):
        x = np.zeros(2)
        for i in range(len(group)):
            if float(self.data[group[i]]['Percent_Tracked']) == 0.0:
                x[0]+=1
            else:
                x[1]+=1
        x/=x.sum()
        x*=100
        return x


    def graph_data(self):
        bins = 10
        for i in range(len(self.groups)):
            fig1, ax1 = plt.subplots(ncols=3,nrows=3,gridspec_kw={'hspace': 1, 'wspace': 1})

            data = self.get_data('Avg_Ownship_Error3',self.groups[i])
            ax1[0,0].hist(data,bins = bins)
            ax1[0,0].set_title('3 Ownship')
            ax1[0,0].set_ylabel('Frequency')
            ax1[0,0].set_xlabel('Error(m)')

            data = self.get_data('Avg_Blue_Error3',self.groups[i])
            ax1[0,1].hist(data,bins = bins)
            ax1[0,1].set_title('3 Est 4')
            ax1[0,1].set_ylabel('Frequency')
            ax1[0,1].set_xlabel('Error(m)')

            data = self.get_data('Avg_Red_Error3',self.groups[i])
            ax1[0,2].hist(data,bins = bins)
            ax1[0,2].set_title('3 Est Red')
            ax1[0,2].set_ylabel('Frequency')
            ax1[0,2].set_xlabel('Error(m)')

            data = self.get_data('Avg_Ownship_Error4',self.groups[i])
            ax1[1,0].hist(data,bins = bins)
            ax1[1,0].set_title('4 Ownship')
            ax1[1,0].set_ylabel('Frequency')
            ax1[1,0].set_xlabel('Error(m)')

            data = self.get_data('Avg_Blue_Error4',self.groups[i])
            ax1[1,1].hist(data,bins = bins)
            ax1[1,1].set_title('4 Est 3')
            ax1[1,1].set_ylabel('Frequency')
            ax1[1,1].set_xlabel('Error(m)')

            data = self.get_data('Avg_Red_Error4',self.groups[i])
            ax1[1,2].hist(data,bins = bins)
            ax1[1,2].set_title('4 Est Red')
            ax1[1,2].set_ylabel('Frequency')
            ax1[1,2].set_xlabel('Error(m)')
            
            
            data = self.get_data('Time_to_Spot',self.groups[i])
            ax1[2,0].set_title('Time to Spot')
            ax1[2,0].set_ylabel('Frequency')
            ax1[2,0].set_xlabel('Time(s)')
            ax1[2,0].hist(data,bins = bins)

            data = self.get_data('Percent_Tracked',self.groups[i])
            ax1[2,1].set_title('% Tracked')
            ax1[2,1].set_ylabel('Frequency')
            ax1[2,1].set_xlabel('%')
            ax1[2,1].hist(data,bins = bins)

            labels = 'Not Found','Found'
            data = self.get_percent_found(self.groups[i])
            ax1[2,2].set_title('% Found')
            ax1[2,2].pie(data, labels=labels,autopct='%1.1f%%', shadow=True, startangle=140)


            fig1.suptitle('Test Statistics for '+self.group_names[i])
            fig1.savefig(self.data_loc+'/all_data_'+self.group_names[i]+'.png')


            


            plt.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze Data')
    parser.add_argument("location", type=str, help="location of data from scripts file(data/phaseX_x/)")
    args = parser.parse_args()
    data_location = args.location


    g = Graph(data_location)
