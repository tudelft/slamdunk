#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic



import rospy, rospkg
import sys
from threading import Thread
from geometry_msgs.msg import PoseStamped
from slamdunk_msgs.msg import QualityStamped
from sensor_msgs.msg import Image
import message_filters
import math
import time
import numpy as np
from datetime import datetime
from subprocess import call
import traceback
import cv2
from cv_bridge import CvBridge, CvBridgeError


from scapy.all import srp1,Ether,ARP,conf

rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')
sys.path.append(PPRZROS_BASE + '/src/pprzros')

import rosudp2
import rosserial
from pprzlink.message import PprzMessage

min_velocity_samples = 4
drone_id = 10

class Light(object):
    def __init__(self):
        self.hasStopped = True

    def blink(self, n, timer, leds, sleepy="True"):
        timer /= float(1000)
        for i in range(n):
            for led in leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("1")
            time.sleep(timer)
            for led in leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("0")
            if sleepy:
                time.sleep(timer)
        return 0

class ThreadLight(Thread):

    def __init__(self, n, timer, leds, sleepy="True"):
        Thread.__init__(self)
        self.n = n
        self.timer = timer
        self.leds = leds
        self.sleepy = sleepy

    def run(self):
        self.timer /= float(1000)
        for i in range(self.n):
            for led in self.leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("1")
            time.sleep(self.timer)
            for led in self.leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("0")
            if self.sleepy:
                time.sleep(self.timer)
        return 0

class Drone(object):
    address = ''
    id = 0
    Ry = 10.8 #cm
    Rp = 5.7 #cm
    Rr = 11.1 #cm
    yaw_cst = - 1.232 #rad
    pitch_cst = 0.716 #rad
    roll_cst = -1.17 #rad

    heading = 0 #rad
    pitch = 0 #rad
    roll = 0 #rad
    x = 0
    y = 0
    z = 0
    qx = 0
    qy = 0
    qz = 0
    qw = 1
    vel_x = 0
    vel_y = 0
    vel_z = 0
    speed_x = 0
    speed_y = 0
    speed_z = 0
    vel_samples = 0
    vel_transmit = 0

    def __init__(self, id):
        self.id = id

    def hasMoved(self, x, y, z, qx, qy, qz, qw):
        return (self.x != x or self.y != y or self.z != z or self.qx != qx
                or self.qy != qy or self.qz != qz or self.qw != qw)


def str_to_bool(string): return string == "True"

def normRadAngle(x):
    while x > math.pi : x -= 2 * math.pi
    while x < -math.pi : x += 2 * math.pi
    return x

def quaternionToEulerianAngle(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z

def xy_yaw_correction():
    theta = normRadAngle(drone.heading + drone.yaw_cst)
    L = math.fabs(2 * drone.Ry * math.sin(theta/2))

    alpha = (math.pi - math.fabs(theta))/2
    D = drone.Ry * math.cos(theta)
    gamma = math.asin(D/drone.Ry)
    beta = normRadAngle(alpha - gamma)

    dx = - L * math.cos(beta) if theta > 0 else L * math.cos(beta)
    dy = L * math.sin(beta) - drone.Ry

    return(dx, dy)

def yz_pitch_correction():
    theta = normRadAngle(drone.pitch + drone.pitch_cst)
    L = math.fabs(2 * drone.Rp * math.sin(theta/2))

    alpha = (math.pi - math.fabs(theta))/2
    D = drone.Rp * math.cos(theta)
    gamma = math.asin(D/drone.Rp)
    beta = normRadAngle(alpha - gamma)

    dy = - L * math.cos(beta) if theta > 0 else L * math.cos(beta)
    dz = L * math.sin(beta) - drone.Rp

    return(dy, dz)

def xz_roll_correction():
    theta = normRadAngle(drone.roll + drone.roll_cst)
    L = math.fabs(2 * drone.Rr * math.sin(theta/2))

    alpha = (math.pi - math.fabs(theta))/2
    D = drone.Rr * math.cos(theta)
    gamma = math.asin(D/drone.Rr)
    beta = normRadAngle(alpha - gamma)

    dx = - L * math.cos(beta) if theta > 0 else L * math.cos(beta)
    dz = L * math.sin(beta) - drone.Rr

    return(dx, dz)

def callback(pose, quality):
    dx_yaw, dy_yaw = xy_yaw_correction()
    dy_pitch, dz_pitch = yz_pitch_correction()
    dx_roll, dz_roll = xz_roll_correction()

    new_x = - pose.pose.position.y * 100 + dx_yaw + dx_roll
    new_y = pose.pose.position.x * 100 + dy_yaw + dy_pitch
    new_z = pose.pose.position.z * 100 + dz_pitch + dz_roll

    new_qx = pose.pose.orientation.x
    new_qy = pose.pose.orientation.y
    new_qz = pose.pose.orientation.z
    new_qw = pose.pose.orientation.w

    # Differentiate the position to get the speed
    if drone.hasMoved(new_x, new_y, new_z, new_qx, new_qy, new_qz, new_qw):
        drone.vel_x += new_x - drone.x
        drone.vel_y += new_y - drone.y
        drone.vel_z += new_z - drone.z
        drone.vel_samples += 1

    drone.x = new_x
    drone.y = new_y
    drone.z = new_z
    drone.qx = new_qx
    drone.qy = new_qy
    drone.qz = new_qz
    drone.qw = new_qw

    # Calculate the derivative of the sum to get the correct velocity
    drone.vel_transmit += 1
    if drone.vel_samples >= min_velocity_samples:
        sample_time = drone.vel_transmit/freq_transmit
        drone.speed_x = drone.vel_x / sample_time
        drone.speed_y = drone.vel_y / sample_time
        drone.speed_z = drone.vel_z / sample_time
        drone.vel_x = 0
        drone.vel_y = 0
        drone.vel_z = 0
        drone.vel_samples = 0
        drone.vel_transmit = 0

    # Normalized rotations in rad
    rotations = quaternionToEulerianAngle(drone.qx, drone.qy, drone.qz, drone.qw)

    drone.roll, drone.pitch, yaw = map(normRadAngle, rotations)
    drone.heading = - yaw

    # Generating tow
    tow = np.uint32((int(time.strftime("%w"))-1) * (24 * 60 * 60 * 1000)
                    + int(time.strftime("%H")) * (60 * 60 * 1000)
                    + int(time.strftime("%M")) * (60 * 1000)
                    + int(time.strftime("%S")) * 1000
                    + int(datetime.now().microsecond / 1000))

    try:
        message = PprzMessage("datalink", "REMOTE_GPS_LOCAL")
        message.set_values([np.uint8(drone.id),
                        np.int32(drone.x),
                        np.int32(drone.y), # x and y are swapped /!\
                        np.int32(drone.z),
                        np.int32(drone.speed_x),
                        np.int32(drone.speed_y),
                        np.int32(drone.speed_z),
                        np.uint32(tow),
                        np.int32(drone.heading  * 10000000)])

        if (quality.quality.value != 0) and (quality.quality.value != 4):
            if lights.hasStopped:
                light_data = ThreadLight(3, 400, ["front:left:blue", "front:right:blue", "rear:left:blue", "rear:right:blue"])
                light_data.start()
                lights.hasStopped = False
            udp.interface.send(message, drone.id, drone.address)

        elif not lights.hasStopped:
            light_problem = ThreadLight(1, 1500, ["front:left:red", "front:right:red", "rear:left:red", "rear:right:red"], sleepy="False")
            light_problem.start()
            lights.hasStopped = True
    except KeyError as e:
        pass

def callback_depth(depth):

    image_buffer = drone.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    mean_left = 0
    mean_right = 0
    matrix = depth.height * depth.width

    for row in range(depth.height):
        for col in range(depth.width):
            pix = image_buffer[row][col]
            if pix < 1000:
                if(col < depth.width/2):
                    mean_left += pix
                else:
                    mean_right += pix

    mean_left /= matrix * 0.5
    mean_right /= matrix * 0.5

    message = PprzMessage("intermcu", "IMCU_LR_MEAN_DIST")
    message.set_values([np.float(mean_left),
                        np.float(mean_right)])

    serial.interface.send(message, drone.id)


def listener():
    pose_sub = message_filters.Subscriber('/pose', PoseStamped)
    qual_sub = message_filters.Subscriber('/quality', QualityStamped)
    rospy.Subscriber('/depth_map/image', Image, callback_depth)
    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, qual_sub], 1, 1)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    freq_transmit = 30.
    udp = rosudp2.RosUdpMessagesInterface()
    serial = rosserial.RosSerialMessagesInterface(device='/dev/ttyTHS1', baudrate="921600", msg_class="intermcu")
    drone = Drone(drone_id)
    drone.bridge = CvBridge()
    lights = Light()

    # Search drone's IP address (7.5s runtime)
    conf.verb = 0
    ans = srp1(Ether(dst="ff:ff:ff:ff:ff:ff")/ARP(pdst='192.168.43.0/27'), timeout=0.5, iface='usb1', inter=0.1)
    try:
        drone.address = ans[0][1].sprintf(r"%ARP.psrc%")
        light_connection = ThreadLight(1, 1500, ["front:left:green", "front:right:green", "rear:left:green", "rear:right:green"])
        light_connection.run()
    except Exception as e:
        lights.blink(2, 1000, ["front:left:red", "front:right:red", "rear:left:red", "rear:right:red"])
        traceback.print_exc()
        exit(1)

    listener()
