#!/usr/bin/env python
from __future__ import division
import subprocess
import time
import os
import signal

time.sleep(20)

# subp.call("source ~/minau/devel/setup.bash",shell=True)
proc1 = subprocess.Popen('roslaunch etddf uuv_etddf.launch',stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 


time.sleep(5)

# proc3 = subprocess.Popen('rosrun etddf border_patrol_node.py __ns:=bluerov2_3',stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
# proc4 = subprocess.Popen('rosrun etddf border_patrol_node.py __ns:=bluerov2_4',stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
# proc5 = subprocess.Popen('rosrun etddf enemyMove.py __ns:=rexrov2',stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

time.sleep(10)
print('hello')
#proc3.kill()


###use this to kill instead. try to get all these launch and moving files started in the same file
###by end of the day try to get it so you can basically just reset the program so it does the same stuff over again

#os.killpg(os.getpgid(proc1.pid), signal.SIGTERM)
