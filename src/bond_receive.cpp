#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "std_msgs/String.h" //for room name topic
#include "sensor_msgs/PointCloud2.h"
#include "bondcpp/bond.h"

#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;


/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "bond_receive");
    ros::NodeHandle n;
    ros::Rate rate(10.0);

bond::Bond bond("/wheelchair_robot/bond/sandpit", "sandpit-bond");
  bond.start();
  // ... do things ...

  fprintf(stdout, "Bond started\n");
  //for (size_t i = 0; i < 2000000000; i++) {} 
  //fprintf(stdout, "Breaking bond\n");
  //bond.breakBond();
  //fprintf(stdout, "Bond stopped\n");

    while(ros::ok()) {
        

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
