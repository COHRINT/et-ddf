from __future__ import division
import subprocess
import time
import os
import signal



### ToDo: make it so the simulation can reset, figure out how to do the rexrov when it is moving to reset it
### look into how you could read all the situations from a csv

print('helooe')
args1 = ['roslaunch','etddf','scenario.sitl.ocean.launch']
proc1 = subprocess.Popen(args1)

time.sleep(40)
x = 0
while x < 3:
    args2 = ['roslaunch','etddf','uuv_etddf.launch']
    proc2 = subprocess.Popen(args2)

    time.sleep(20)

    args3 = ['rosrun','etddf','border_patrol_node.py','__ns:=bluerov2_3','waypt_i:=1']
    args4 = ['rosrun','etddf','border_patrol_node.py','__ns:=bluerov2_4']
    proc3 = subprocess.Popen(args3)
    proc4 = subprocess.Popen(args4)

    time.sleep(25)
    proc4.terminate()
    proc3.terminate()
    proc2.terminate()
    time.sleep(15)
    print('\n\nTrying to teleport\n\n')
    args5 = ['rosrun','etddf','teleport.py']
    proc5 = subprocess.Popen(args5)
    x+=1
time.sleep(30)
proc1.terminate()