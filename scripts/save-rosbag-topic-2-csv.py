#!/usr/bin/env python3

import sys
import os
import csv
import rosbag
import rospy

##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# rosrun your_package:
# rosrun wheelchair_experiments room-name-timestamps.py /home/tomos/ros/wheelchair/catkin_ws/src/wheelchair_dump/dump/bags/ env1-run2_2021-11-02-17-12-24.bag 

# ##################

filename = sys.argv[2]
directory = sys.argv[1]
print("Reading the rosbag file")
if not directory.endswith("/"):
  directory += "/"
extension = ""
if not filename.endswith(".bag"):
  extension = ".bag"
bag = rosbag.Bag(directory + filename + extension)

# Create directory with name filename (without extension)
results_dir = directory + filename[:-4] + "_results"
if not os.path.exists(results_dir):
  os.makedirs(results_dir)

substopic = '/wheelchair_robot/user/room_name'
print("Writing robot joint state data to CSV")

with open(results_dir +"/"+filename+'_room-names.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time', 'name'])
  # Get all message on the ros topics topic
  for topic, msg, t in bag.read_messages(topics=[substopic]):
    # Only write to CSV if the message is for our robot
    #if msg.name[0] == "robot_elbow_joint":
    p = msg.data
    data_writer.writerow([t.secs, p[0]])

print("Finished creating csv file!")
bag.close()