# This script is used to align the time of two topics in a rosbag file.
# The time of the first topic is used to align the time of the second topic.
# The aligned rosbag file is saved as a new file.
# You can reference https://wiki.ros.org/rosbag/Cookbook
# The script is written by Yixin.
# just run the script in the terminal: python rostopic_align.py

#!/usr/bin/env python3

#from importlib import reload
import rospy
import rosbag
import sys

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')
bag_name = '2025-04-04-23-57-18.bag' 
out_bag_name = 'gnss_leo_msg_0405.bag' 
# Please change the path to your own absolute path
# add by Yixin
dst_dir = '/home/gao-yixin/下载/软件/Dropbox/GraphGNSSLib_LEO/src/global_fusion/dataset/2021_0521_0607/' 

with rosbag.Bag(dst_dir+out_bag_name, 'w') as outbag:
    stamp = None
    for topic, msg, t in rosbag.Bag(dst_dir+bag_name).read_messages():
        if topic == '/leo_raw_publisher_node/LEOPsrCarRov1':
            stamp = msg.header.stamp
        elif topic == '/leo_raw_publisher_node/LEOPsrCarStation1' and stamp is not None: 
            outbag.write(topic, msg, stamp)
            continue
        outbag.write(topic, msg, msg.header.stamp)

print("finished")
