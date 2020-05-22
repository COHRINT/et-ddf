import yaml
import os
import csv


#next create a csv that you can parse through to get all the information you need
rowNum = 1
rows = []
with open('scenarios.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        rows.append(row)

start_pos = "["
red_names = "["
blue_names = "["
blue_pos = "["

#starting position
for i in range(4):
    start_pos += rows[rowNum][i] + ','

start_pos = start_pos[:-1]+']'
#red team names
i = 4
while rows[rowNum][i]!='blue_names':
    red_names+=rows[rowNum][i]+','
    i+=1
i+=1
if(len(red_names)!=1):
    red_names=red_names[:-1]+']'
else:
    red_names='[]'
#blue team names
numBlue = 0
while rows[rowNum][i]!='blue_poses':
    blue_names+=rows[rowNum][i] +','
    numBlue+=1
    i+=1
i+=1
if len(blue_names)!=1:
    blue_names=blue_names[:-1]+']'
else:
    blue_names=[]
#blue team positions
if numBlue!=0:
    blue_pos+='['
else:
    blue_pos+='],'
for j in range(numBlue):
    for k in range(4):
        blue_pos+=rows[rowNum][i]+','
        i+=1
    blue_pos=blue_pos[:-1]+'],['
blue_pos= blue_pos[:-2]+']'
print(start_pos)
print(red_names)
print(blue_names)
print(blue_pos)

#this switches to the correct directory and reads in the yaml
# os.chdir("../config")


# with open('example.yaml') as f:
#     etddf = yaml.load(f, Loader=yaml.FullLoader)

# # print(etddf)

# #using this dictionary format you can change any of the attributes you want
# etddf["default_starting_pose"]["x"]=5.0
# etddf["default_starting_pose"]["y"]=5.0
# etddf["default_starting_pose"]["z"]=5.0

# # print(etddf)

# #this writes the changes to the file
# with open('example.yaml', 'w') as f:
#     yaml.dump(etddf, f)

# #switches to the correct directory
# os.chdir("../launch")

# #these are the attributes that you want to change
# start_pos = "[0,9,0,0]"
# red_names = "[]"
# blue_names = "[]"
# blue_pos = "[]"
# config_file = "$(find etddf)/config/example.yaml"

# #puts the attributes in the format for the file and writes to it
# newFile = '''<launch>
#     <arg name="my_name" default="bluerov2_3" />
#     <arg name="blue_team_names" default="'''+blue_names+'''" />
    
#     <!-- not required, list of lists, index of outer 
#     list corresponds to index in blue_team_names -->
#     <arg name="blue_team_poses" default="'''+blue_pos+'''" /> 
    
#     <arg name="red_team_names" default="'''+red_names+'''" />
#     <arg name="config_file" default="'''+config_file+'''" />
#     <arg name="starting_pose" default="'''+start_pos+'''" /> <!-- [x,y,z, yaw_rad] -->
    
#     <node name="etddf" pkg="etddf" type="etddf_node.py" output="screen">
#         <param name="my_name" value="$(arg my_name)" />
#         <rosparam param="blue_team_poses" subst_value="True">$(arg blue_team_poses)</rosparam> 
#         <rosparam param="blue_team_names" subst_value="True">$(arg blue_team_names)</rosparam> 
#         <rosparam param="red_team_names" subst_value="True">$(arg red_team_names)</rosparam> 
#         <rosparam param="starting_pose" subst_value="True">$(arg starting_pose)</rosparam> 
#         <rosparam command="load" file="$(arg config_file)" />  
#         <rosparam command="load" file="$(find etddf)/config/measurements.yaml" />
#     </node>

# </launch>'''

# f = open('example.launch','w')
# f.writelines(newFile)
