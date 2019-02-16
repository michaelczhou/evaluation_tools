#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import tf
import rosbag
import roslib
import sys
from nav_msgs.msg import Odometry
import numpy as np
from numpy import *
from sensor_msgs.msg import Imu
from receive_gps.msg import rtkGPS  #gps msg 文件
from sensor_msgs.msg import ChannelFloat32

import TF

import sys
reload(sys)
sys.setdefaultencoding("utf-8")

BAG_NAME = '/media/ywl/samsungT5/newgps/dache_2019-01-22-20-14-17SyncSystemTime.bag'
OUT_FILE = '/home/ywl/ros_ws/catkin_getdata/traj/gpsout.txt'
OUT_VELODYNE_STAMP = '/home/ywl/ros_ws/catkin_getdata/traj/velodyne_packets.txt'
VELODYNE_TOPIC = 'velodyne_packets'
#若已经进行修正过,则将一下两个值设为0
offset = 0.7356#0.75714 #gps中心和激光中心的偏移量
offsetyaw = 0#0.36717 #gps中心和激光中心的偏移角度,x方向为基准轴,向右为正,向左为负

if __name__ == "__main__":

    GLOBAL_TIME = "global"

    bag = rosbag.Bag(BAG_NAME)
    #mytf=TF.TF();
    #outdir = "/home/csc105/catkin_ws/src/process_data/save/"
    first_gps=False
    init_gps=[0,0,0]
    count=0
    with open(OUT_FILE, 'w') as record:
        with open(OUT_VELODYNE_STAMP, 'w') as vstamp:
            #print(msg)
            #if hasattr(msg, 'header'):
            for topic, msg, time in bag.read_messages():
                if VELODYNE_TOPIC in str(topic):
                    vstamp.write(str(msg.header.stamp))
                    vstamp.write('\n')
                if "rtkGPS" in str(topic):
                    count+=1
                    if first_gps == False:
                        if msg.vaild_flag == True:
                            # print "a"
                            # print [msg.north_meter,msg.east_meter,msg.yaw_rad]
                            yaw = msg.yaw_rad*math.pi/180.0 + offsetyaw
                            msg.north_meter += math.cos(yaw) * offset
                            msg.east_meter += math.sin(yaw) * offset
                            # print [msg.north_meter, msg.east_meter, msg.yaw_rad]
                            init_gps[0] = msg.north_meter
                            init_gps[1] = msg.east_meter
                            init_gps[2] = msg.yaw_rad
                            init_height = msg.height_meter
                            record.write(str(msg.header.stamp))
                            record.write(' 0 0 0 '+str(-msg.yaw_rad)+'\n')
                            first_gps = True
                            #record.write('\n')
                    else:
                        if msg.vaild_flag == True:
                            #print "a"
                            #print [msg.north_meter, msg.east_meter, msg.yaw_rad]
                            yaw = msg.yaw_rad*math.pi/180.0 + offsetyaw
                            msg.north_meter += math.cos(yaw) * offset
                            msg.east_meter += math.sin(yaw) * offset
                            gpsheight=msg.height_meter-init_height
                            #gpsheight-=init_height
                            outx=msg.north_meter-init_gps[0]
                            outy=-(msg.east_meter-init_gps[1])

                            #nowGPS=[msg.north_meter,msg.east_meter,msg.yaw_rad]
                            #print [msg.north_meter, msg.east_meter, msg.yaw_rad]
                            #print(nowGPS)
                            #tfGPS=mytf.rtkGPStoBaseLink(nowGPS,init_gps)
                            #with open('gps.txt', 'a') as record:
                            record.write(str(msg.header.stamp))
                            record.write(' ')
                            record.write(str(outx)+' '+str(outy)+' '+str(gpsheight)+' '+str(-msg.yaw_rad))
                            record.write('\n')
                        else:
                            record.write(str(msg.header.stamp))
                            record.write(' -1 -1 -1 -1\n')


    print('save done')
    print(count)
