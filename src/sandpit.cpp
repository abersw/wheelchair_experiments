#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "std_msgs/String.h" //for room name topic

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
    ros::init(argc, argv, "objects_context");
    ros::NodeHandle n;
    ros::Rate rate(10.0);

    int t = 1;
    for (int i = 0; i < t; i++) {
        cout << "i is " << i << endl;
    }

    while(ros::ok()) {
        
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
