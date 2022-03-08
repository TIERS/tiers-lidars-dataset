#!/usr/bin/python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

from matplotlib import pyplot as plt


os0_stamp = []
os1_stamp = []
avia_stamp = []
horizon_stamp = []
mocap_stamp = []

topics = []
topics_start_stamp = [] 
 

topics_frame_ls = ["/avia/livox/lidar", "/avia/livox/imu", "/livox/imu", "/livox/lidar", \
            "/os_cloud_node/points", 
            "/os_cloud_node/imu",\
            "/os_cloud_nodee/points",\
            "/os_cloud_nodee/imu"\
              ]

frame_ls  = ["avia_frame", "avia_frame", "horizon_frame", "horizon_frame", \
            "os0_sensor",  \
            "os0_imu",\
            "os1_sensor", \
            "os1_imu"
            ]

  
def change_frame_id(inbag,outbag, topics_ls, frame_ls): 
    outbag = rosbag.Bag(outbag,'w')
    for topic, msg, t in rosbag.Bag(inbag,'r').read_messages(): 

        # Chages the frame_id
        if topic in topics_frame_ls:
            id = topics_frame_ls.index(topic)
            if msg._has_header:
                print(id, topic,  msg.header.frame_id, "->", frame_ls[id])
                msg.header.frame_id = frame_ls[id] 
        
        print(topic, msg.header.frame_id, msg.header.stamp)
        outbag.write(topic, msg, t)
    rospy.loginfo('Closing output bagfile and exit...')
    outbag.close()

if __name__ == "__main__":
    folder_path = "./IROS2022_Dataset/" 
    in_bag =  folder_path + "st_square_2022-02-08-23-14-55.bag" 
    reframed_bag = folder_path + "forest02_square.bag"  
    
    print("=> Processing frame_id and timestamp alignment ")
    change_frame_id(in_bag, reframed_bag, topics_frame_ls, frame_ls ) 

 
