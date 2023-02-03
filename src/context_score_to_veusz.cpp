#include "tof_tool/tof_tool_box.h"
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

//ImportFileCSV('1DLF-object-scores-data/0-potted plant.csv', headermode='1st', linked=True, numericlocale='en_GB', dsprefix='0-potted plant.csv', dssuffix='0-potted plant.csv')

/*
Add('xy', name='14-bottle', autoadd=False)
To('14-bottle')
Set('xData', '14-bottle.csvduration14-bottle.csv')
Set('yData', '14-bottle.csvcontext score14-bottle.csv')
Set('hide', False)
Set('key', '14-bottle')
Set('xAxis', 'x')
Set('yAxis', 'y')
Set('labels', '')
Set('scalePoints', [])
Set('PlotLine/steps', 'left')
Set('PlotLine/interpType', 'linear')
Set('PlotLine/width', '3pt')
Set('PlotLine/style', 'solid')
Set('Label/posnVert', 'top')
Set('Label/size', '14pt')
To('..')
*/

static const int DEBUG_trackingFileToArray = 1;
static const int DEBUG_populateObjectsToTrack = 1;

TofToolBox *tofToolBox;

//context data to save
struct TrackingObjects {
    int object_id;
    string object_name;
    float object_confidence; //object confidence from dnn

    double object_timestamp; //should be saved in seconds .toSec()

    //context info
    int times_trained; //real times trained
    double times_trained_val; //actual value used for calculating object weighting

    //context data
    int object_detected; //times object has been detected

    double object_weighting; //object weighting result
    double object_uniqueness; //object uniqueness result
    double object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env
};
static const int totalObjectsTracked = 1100;
static const long totalObjectsTrackedCaptured = 10000;
//[0] contains object id and name [1] instances detected
struct TrackingObjects trackingObjects[totalObjectsTracked][totalObjectsTrackedCaptured];
int totalTrackingObjectsList = 0;

//contains instances of object found, uses order from trackingObjects
int totalTrackingObjectsCaptured[totalObjectsTracked];

string trackingObjectsListRaw[10000];
int totalTrackingObjectsListRaw = 0;

std::string PARAM_dataset_name;
std::string wheelchair_experiments_loc; //location of wheelchair_dump package
std::string experiments_loc = "/docs/objects-to-track/"; //location of list of objects to tack
std::string experiments_output_loc = "/docs/objects-to-track-output/";
std::string experiments_loc_file;

ofstream FILE_WRITER;

void trackingFileToArray() {
    int counter = 0;
    std::ifstream file(experiments_loc_file);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // using printf() in all tests for consistency
            if (DEBUG_trackingFileToArray) {
                cout << "complete line from file is " << line << endl;
            }
            size_t colon_pos = line.find(':');
            string str1 = line.substr(0, colon_pos);
            string str2 = line.substr(colon_pos+1);
            trackingObjectsListRaw[counter] = str1;
            counter++;
            trackingObjectsListRaw[counter] = str2;
            counter++;
        }
        file.close();
    }
    else {
        cout << "something went wrong opening the file" << endl;
    }
    totalTrackingObjectsListRaw = counter;
    if (DEBUG_trackingFileToArray) {
        for (int i = 0; i < counter; i++) {
            cout << trackingObjectsListRaw[i] << endl;
        }
    }
}

void populateObjectsToTrack() {
    if (DEBUG_populateObjectsToTrack) {
        cout << "total tracking objects list raw " << totalTrackingObjectsListRaw << endl;
    }
    int pos = 0;
    int counter = 0;
    for (int i = 0; i < totalTrackingObjectsListRaw; i++) {
        if (pos == 0) {
            //trackingObjectsList[counter].object_id = std::stoi(trackingObjectsListRaw[i]);
            trackingObjects[counter][0].object_id = std::stoi(trackingObjectsListRaw[i]);
            pos++;
        }
        else if (pos == 1) {
            //trackingObjectsList[counter].object_name = trackingObjectsListRaw[i];
            trackingObjects[counter][0].object_name = trackingObjectsListRaw[i];
            pos = 0;
            totalTrackingObjectsCaptured[counter] = 0;
            counter++;
        }
        else {
            if (DEBUG_populateObjectsToTrack) {
                cout << "something went wrong during allocation" << endl;
            }
        }
    }
    totalTrackingObjectsList = counter;

    if (DEBUG_populateObjectsToTrack) {
        cout << "total objects to track is " << totalTrackingObjectsList << endl;
        for (int i = 0; i < totalTrackingObjectsList; i++) {
            cout << trackingObjects[i][0].object_id << ":" << trackingObjects[i][0].object_name << endl;
        }
    }
}

void buildImportPaths() {
    tofToolBox->printSeparator(0);
    cout << "add to top of veusz file" << endl;
    tofToolBox->printSeparator(0);
    cout << "total objects to track" << totalTrackingObjectsList << endl;
    for (int isObject = 0; isObject < totalTrackingObjectsList; isObject++) {
        std::string isID = to_string(trackingObjects[isObject][0].object_id);
        std::string isName = trackingObjects[isObject][0].object_name;
        std::string importLine;
        importLine = "ImportFileCSV('" + PARAM_dataset_name  + "-object-scores-data/" +
                    isID + "-" + isName + ".csv'," +
                    " headermode='1st', linked=True, numericlocale='en_GB', dsprefix='" +
                    isID + "-" + isName + ".csv'," +
                    " dssuffix='" + isID + "-" + isName + ".csv')";

        cout << importLine << endl;
        FILE_WRITER << importLine << "\n";
    }
}
/*
Add('xy', name='14-bottle', autoadd=False)
To('14-bottle')
Set('xData', '14-bottle.csvduration14-bottle.csv')
Set('yData', '14-bottle.csvcontext score14-bottle.csv')
Set('hide', False)
Set('key', '14-bottle')
Set('xAxis', 'x')
Set('yAxis', 'y')
Set('labels', '')
Set('scalePoints', [])
Set('PlotLine/steps', 'left')
Set('PlotLine/interpType', 'linear')
Set('PlotLine/width', '3pt')
Set('PlotLine/style', 'solid')
Set('Label/posnVert', 'top')
Set('Label/size', '14pt')
To('..')
*/
void buildXY() {
    std::string xyLine;
    for (int isObject = 0; isObject < totalTrackingObjectsList; isObject++) {
        std::string isID = to_string(trackingObjects[isObject][0].object_id);
        std::string isName = trackingObjects[isObject][0].object_name;
        xyLine = "Add('xy', name='" + isID + "-" + isName + "', autoadd=False)" + "\n" +
                "To('" + isID + "-" + isName + "')" + "\n" +
                "Set('xData', '" + isID + "-" + isName + ".csvduration" + isID + "-" + isName + ".csv')" + "\n" +
                "Set('yData', '" + isID + "-" + isName + ".csvcontext score" + isID + "-" + isName + ".csv')" + "\n" +
                "Set('hide', False)" + "\n" +
                "Set('key', '" + isID + "-" + isName + "')" + "\n" +
                "Set('xAxis', 'x')" + "\n" +
                "Set('yAxis', 'y')" + "\n" +
                "Set('labels', '')" + "\n" +
                "Set('scalePoints', [])" + "\n" +
                "Set('PlotLine/steps', 'left')" + "\n" +
                "Set('PlotLine/interpType', 'linear')" + "\n" +
                "Set('PlotLine/width', '3pt')" + "\n" +
                "Set('PlotLine/style', 'solid')" + "\n" +
                "Set('Label/posnVert', 'top')" + "\n" +
                "Set('Label/size', '14pt')" + "\n" +
                "To('..')" + "\n";

        cout << xyLine << endl;
        FILE_WRITER << xyLine << "\n";
    }
}



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

    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    wheelchair_experiments_loc = tofToolBox->doesPkgExist("wheelchair_experiments");//check to see if dump package exists
    
    if (n.getParam("/wheelchair_robot/context/track_name", PARAM_dataset_name)) {
        ROS_INFO("Got param: %s", PARAM_dataset_name.c_str());
        experiments_loc_file = wheelchair_experiments_loc + experiments_loc + PARAM_dataset_name + ".txt";
        cout << "experiments file is located at " << experiments_loc_file << endl;
        
        std::string outputListLoc = wheelchair_experiments_loc + experiments_output_loc + PARAM_dataset_name + ".txt";
        tofToolBox->createFile(outputListLoc); //check to see if file is present, if not create a new one

        FILE_WRITER.open(outputListLoc);
        trackingFileToArray();
        cout << "finished populating" << endl;
        populateObjectsToTrack();
        buildImportPaths();
        buildXY();
        //objectsToTrack = 1;
        FILE_WRITER.close();
    }
    else {
        ROS_ERROR("Failed to get param '/wheelchair_robot/context/track_name'");
        //objectsToTrack = 0;
    }

    return 0;
}
