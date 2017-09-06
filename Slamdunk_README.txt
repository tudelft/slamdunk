Observations/Explanations/Recommendations on the code related to autonomous navigation using Slamdunk.

1. How it works and what it does

  a. Inside the Slamdunk

    The Slamdunk is running a SLAM algorithm and publishes a lot of information via ROS topics. A Python code of my conception is also running automatically, when the Slamdunk has finished booting (it's running a ubuntu 14.04 kernel). The code does the following things :

    - Opens a UDP and a Serial connection with the drone's autopilot (searching autonomously for the IP address of the drone).

    - Subscribes to a position topic (/pose) and a quality topic (/quality)
        -> Transforms it into a Paparazzi message (REMOTE_GPS_LOCAL, 'datalink')

        -> Sends the GPS message via the UDP connection, thus giving the drone a 3D Fix

    - Subscribes to a depth map topic (/depth_map/image)
        -> Gets the mean distance to the walls/obstacles in the left and right part of the field of view and sends it via the Serial connection, in IMCU_LR_MEAN_DIST, 'intermcu'


  b. In Paparazzi

    - A module Slamdunk reads the incoming IMCU_LR_MEAN_DIST, computes it into an OBSACLE_AVOIDANCE message, sent via Abi
	    -> The flight plan gets the pieces of information and reacts to those

    - The module Gps_datalink processes the REMOTE_GPS_MESSAGE (it has been slightly modified)

    - A flight plan ("egg_slamming.xml") designed for simple Guided mode has been created

    - An airframe ("tudelft_bebop2_indi_opti.xml") has been adapted to use GPS heading and both Gps_datalink and Slamdunk module

    - The messages REMOTE_GPS_LOCAL and IMCU_LR_MEAN_DIST have been added, both in pprzlink and in conf/messages.xml


2. Warnings/Obsrevations

    - Do not start the engines of the drone using the block "start engine" of the flight plan ! In Guided mode, the drone tries to stabilize itself the very moment the motors are turned on, which results in the drone trying to do a barrel roll on the ground (which is obviously bound to fail, and turns the drone upside down).

    - To restart the Slamdunk, turn it off by pressing the power button long a few seconds, then UNPLUG and REPLUG the power cables, and finally press the power button again.

    - The Slamdunk has 4 leds to communicate its state. It's blinking green+red (at the same time, resulting in a reddish green, aka orange) when booting. When the IP of the drone is found, it blinks green once, during 1.5s. When the GPS signal is sent, so you can start flying in guided mode, it blinks blue three times 0.5s.

    - The ID of the drone has to be 10, since it's written inside the Slamdunk's python code.

    - The flight plan block Go Forward is not working yet. Only the Hover block has been tested.

    - The pprzros version inside the Slamdunk is pretty old, and should be update (rosupd2 would then become rosudp in the python program too)
