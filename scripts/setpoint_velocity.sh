#!/bin/bash

rosservice call /bluerov2_3/uuv_control/arm_control "{}"
rosservice call /bluerov2_3/uuv_control/set_heading_velocity "{heading: 0.0, velocity: {x: -0.1, y: 0.0, z: 0.0 }}"
