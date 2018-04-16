#!/bin/bash
rostopic pub /rover/cmd_vel geometry_msgs/Twist -- "[$1, 0.0, 0.0]" "[0, 0, 0]" -1

