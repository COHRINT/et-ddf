#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from ping360_sonar.srv import SetSonarSettings
from ping360_sonar.msg import SonarSettings
from std_msgs.msg import Int16




rospy.init_node("sonar_scanner")
sonar_range = rospy.get_param("~sonar_range",default=10)
angles_per_scan = 18

timing_eq = [0.54166667,7.91666667]      #equation for how long it takes to scan full 360([m,b])

scan_time = timing_eq[0]*sonar_range + timing_eq[1]
scan_time = scan_time * (angles_per_scan/360.0)

names = ['/bluerov2_3/','/bluerov2_4/']
services = []

for name in names:
    services.append(rospy.ServiceProxy(name+"ping360_node/sonar/set_sonar_settings",SetSonarSettings))

#for this node I am going to have the rovs have the same scan angle

pub = rospy.Publisher("/sonar_angle",Int16,queue_size=10)

i = 0
settings = SonarSettings()
settings.range = 10
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    settings.min_angle_deg = i * angles_per_scan
    pub.publish(Int16(settings.min_angle_deg))
    settings.max_angle_deg = ((i+1)%(int(360/angles_per_scan))) * angles_per_scan

    for service in services:
        service(settings)

    i = (i+1)%(int(360/angles_per_scan))
    rospy.sleep(scan_time)





