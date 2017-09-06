#!/bin/bash

[ -z "$ROS_HOSTNAME" ] && export ROS_HOSTNAME=$(hostname).local
. /home/slamdunk/slamdunk_catkin_ws/devel/setup.bash
exec rosrun slamdunk_pprz slamdunk2udp.py
