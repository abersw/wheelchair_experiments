#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include "std_msgs/String.h" //for room name topic
#include "sensor_msgs/PointCloud2.h"
#include "bondcpp/bond.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>

#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

ros::Publisher *pub_ptr;

static const int DEBUG_getResolutionOnStartup = 1;

bool gotResolution = 0; //var flag for successfully getting camera resolution
int imageHeight = 0; //var to store height of rectified image pointcloud
int imageWidth = 0; //var to store width of rectified image pointcloud

//get resolution of rectified pointcloud image
void getResolutionOnStartup(const sensor_msgs::PointCloud2::ConstPtr& dpth) {
    imageHeight = dpth->height; //get height of pointcloud image
    imageWidth = dpth->width; //get width of pointcloud image
    if (DEBUG_getResolutionOnStartup) {
        cout << imageHeight << "x" << imageWidth << "\n"; //print out height and width if debug flag is true
    }
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_parameter) {
    if (gotResolution == 0) {
        getResolutionOnStartup(cloud_parameter); //get pointcloud image size
        gotResolution = 1;
    }
    sensor_msgs::PointCloud2 newCloud = *cloud_parameter;

    float X = 0;
    float Y = 0;
    float Z = 0;

    //get position of point in rectified image array, corresponding with pointcloud
    //int arrayPosition = detectedObjects[isObject].centerY*dpth->row_step + detectedObjects[isObject].centerX*dpth->point_step;
    double chopSize = (20.0 / 100.0) * double(imageWidth);
    double leftChop = chopSize;
    double rightChop = imageWidth - chopSize;
    double interquartileChop = imageWidth - leftChop - rightChop;
    cout << "chop size " << chopSize << endl;
    cout << "first 20 percent " << leftChop << endl;
    cout << "last 20 percent " << rightChop << endl;
    cout << "interquartileChop" << interquartileChop << endl;

    /*//int arrayPosition = detectedObjects[isObject].centerY*dpth->row_step + detectedObjects[isObject].centerX*dpth->point_step;
    for (int imgX = 0; imgX < leftChop; imgX++) { //run from left to right of first chop
        for (int imgY = 0; imgY < imageHeight; imgY++) {

            newCloud.height = row;
            newCloud.width = col;
            newCloud.points.resize(cloud.height * cloud.width);

            int arrayPosition = imgY * newCloud.row_step + imgX * newCloud.point_step;

            int arrayPosX = arrayPosition + newCloud.fields[0].offset; // X has an offset of 0
            int arrayPosY = arrayPosition + newCloud.fields[1].offset; // Y has an offset of 4
            int arrayPosZ = arrayPosition + newCloud.fields[2].offset; // Z has an offset of 8

            memcpy(&X, &newCloud.data[arrayPosX], sizeof(float)); //add value from depth point to X
            memcpy(&Y, &newCloud.data[arrayPosY], sizeof(float)); //add value from depth point to Y
            memcpy(&Z, &newCloud.data[arrayPosZ], sizeof(float)); //add value from depth point to Z

            newCloud.data[arrayPosX] = -1;
            newCloud.data[arrayPosY] = -1;
            newCloud.data[arrayPosZ] = -1;
        }
    }*/
    //pub_ptr->publish(newCloud);





/*
float depthThreshold = 0.5;
float threshold2 = depthThreshold*depthThreshold;

// Container for original & filtered data
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
pcl::PCLPointCloud2 cloud_filtered;

// Convert to PCL data type
pcl_conversions::toPCL(*cloud_parameter, *cloud);

// Perform the actual filtering
for (int p=0; p<cloud->points.size(); ++p)
{
// find the squared distance from the origin.
float pointDepth2 = (cloud->points[p].x * cloud->points[p].x) +
                    (cloud->points[p].y * cloud->points[p].y) + 
                    (cloud->points[p].z * cloud->points[p].z));

// remove point if it's within the threshold range
if (pointDepth2 < threshold2)
{
    cloud->points[p] = cloud->points[cloud->points.size()-1];
    cloud->points.resize(cloud->points.size()-1);
    --p;
}
}

pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud (cloudPtr);
sor.setLeafSize (0.1, 0.1, 0.1);
sor.filter (cloud_filtered);

// Convert to ROS data type
sensor_msgs::PointCloud2 output;
pcl_conversions::fromPCL(cloud_filtered, output);

// Publish the data
pub.publish (output);
*/



pcl::PointCloud<pcl::PointXYZRGB> cloud_in; //original pc as pcl version

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//STEP 0 Convert sensor_msgs to pcl
pcl::fromROSMsg(newCloud, cloud_in);

int i = 0, nr_points = (int) cloud_in.size ();
//cout << nr_points << endl;

}

/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "ros_sandpit");
    ros::NodeHandle n;
    ros::Rate rate(10.0);

    int t = 1;
    for (int i = 0; i < t; i++) {
        cout << "i is " << i << endl;
    }

    ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("/zed/zed_node/point_cloud/cloud_registered", 100, callback);
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("wheelchair_robot/point_cloud_filter",100);
    pub_ptr = &pub;

std::string id = "sandpit-bond";
// Sends id to B using a service or action
bond::Bond bond("/wheelchair_robot/bond/sandpit", id);
bond.start();
if (!bond.waitUntilFormed(ros::Duration(3.0)))
{
ROS_ERROR("ERROR!");
return false;
}
// ... do things with B ...
//bond.waitUntilBroken(ros::Duration(-1.0));
//printf("B has broken the bond\n");

    while(ros::ok()) {
        

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
