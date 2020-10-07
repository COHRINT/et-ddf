#!/bin/bash

rosservice call /bluerov2_3/uuv_control/arm_control "{}"
rosservice call /bluerov2_3/uuv_control/set_heading_depth "{heading: 0.0, depth: -0.05}"
