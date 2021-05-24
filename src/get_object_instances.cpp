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

static const bool DEBUG_doesPkgExist = 0;
static const bool DEBUG_roomsDacopToStruct = 0;
static const bool DEBUG_objectLocationsCallbackDictionary1 = 0;
static const bool DEBUG_objectLocationsCallbackDictionary2 = 1;

struct Objects {
    int object_id;
    string object_name;

    int room_id;
    string room_name;
};
int totalObjectsFileStruct = 0;
struct Objects objectsFileStruct[100000]; //array for storing all object and room data

//struct will store single object names and the instances inside the entire environment
struct ObjectDictionary {
    std::string object_name; //object name
    std::string room_name;
    int instances; //instances of object in environment
};
struct ObjectDictionary objectDictionary[1000]; //struct for storing data needed to calc uniqueness of objects
int totalObjectDictionaryStruct = 0; //total list of objects used to calc uniqueness

//list of file locations
std::string wheelchair_dump_loc;
std::string dump_dacop_loc = "/dump/dacop/";
std::string rooms_dacop_name = "rooms.dacop"; //file with objects associated with rooms
std::string rooms_dacop_loc; //full path for rooms dacop file
std::string rooms_list_name = "rooms.list"; //file with list of rooms
std::string rooms_list_loc; //full path for rooms list

//function for printing space sizes
void printSeparator(int spaceSize) {
	if (spaceSize == 0) {
		printf("--------------------------------------------\n");
	}
	else {
		printf("\n");
		printf("--------------------------------------------\n");
		printf("\n");
	}
}

/**
 * Does the wheelchair_dump package exist in the workspace?
 * If it's missing, close down the node safely
 */
std::string doesPkgExist(std::string pkg_name) {
    std::string getPkgPath;
	if (ros::package::getPath(pkg_name) == "") {
		cout << "FATAL:  Couldn't find package " << pkg_name << "\n";
		cout << "FATAL:  Closing node. \n";
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
		ros::shutdown();
		exit(0);
	}
    else {
        getPkgPath = ros::package::getPath(pkg_name);
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
    }
    return getPkgPath;
}

/**
 * Take room dacop file and add it to struct array for processing later 
 *
 * @param parameter fileName is the path of the room.dacop file
 */
void roomsDacopToStruct(std::string fileName) {
    if (DEBUG_roomsDacopToStruct) {
        cout << "DEBUG_roomsDacopToStruct" << endl;
    }
    //object_id, object_name, room_id, room_name
    std::string objectsDelimiter = ","; //delimiter character is a comma
    ifstream FILE_READER(fileName); //open file
    int objectNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << fileName << " file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            int lineSection = 0; //var for iterating through serialised line
            int pos = 0; //position of delimiter
            std::string token;
            while ((pos = line.find(objectsDelimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                line.erase(0, pos + objectsDelimiter.length());
                //deserialise the line section below:
                if (lineSection == 0) { //if first delimiter
                    objectsFileStruct[objectNumber].object_id = std::stoi(token); //convert object id string to int
                }
                else if (lineSection == 1) { //if second delimiter
                    objectsFileStruct[objectNumber].object_name = token; //get object name
                }
                else if (lineSection == 2) { //if third delimiter
                    objectsFileStruct[objectNumber].room_id = std::stoi(token); //convert room id string to int
                }
                lineSection++; //go to next delimiter
            }
            objectsFileStruct[objectNumber].room_name = line; //get end of line - room name
            if (DEBUG_roomsDacopToStruct) {
                cout << 
                objectsFileStruct[objectNumber].object_id << "," <<
                objectsFileStruct[objectNumber].object_name << "," <<
                objectsFileStruct[objectNumber].room_id << "," <<
                objectsFileStruct[objectNumber].room_name << endl;
            }
            objectNumber++; //go to next object line
        }
    }
    totalObjectsFileStruct = objectNumber; //set total number of objects in struct to the lines(obstacles) counted
}

void getObjectInstances() {
    //create and add object names to object dictionary struct
    for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) {
        //run through all objects
        int objectMatched = 0;
        std::string getObjName = objectsFileStruct[isContext].object_name;
        if (totalObjectDictionaryStruct == 0) {
            objectDictionary[0].object_name = getObjName; //set object name in first element in full objects struct
            totalObjectDictionaryStruct++; //add 1 to total objects in dictionary
        }
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
            std::string getObjDictName = objectDictionary[isDict].object_name;
            if (getObjName == getObjDictName) {
                objectMatched = 1;
            }
            //set objects back to 0
            objectDictionary[isDict].instances = 0;
        }
        if (objectMatched) {
            //if object is already in struct, don't add anything
        }
        else {
            //add object name to struct
            objectDictionary[totalObjectDictionaryStruct].object_name = getObjName;
            objectDictionary[totalObjectDictionaryStruct].instances = 0;
            totalObjectDictionaryStruct++;
        }
    }
    //print out list of objects
    if (DEBUG_objectLocationsCallbackDictionary1) {
        printSeparator(1);
        cout << "pre-instance calculations, total size of struct is " << totalObjectDictionaryStruct << endl;
        for (int isDet = 0; isDet < totalObjectDictionaryStruct; isDet++) {
            cout << objectDictionary[isDet].object_name << ":" << objectDictionary[isDet].instances << endl;
        }
        printSeparator(1);
    }

    //get object instances and assign to object dictionary struct
    for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) { //iterate through object dictionary
        std::string getObjDictName = objectDictionary[isDict].object_name; //get object name from dictionary
        printSeparator(1);
        cout << "total objects in dictionary is " << totalObjectDictionaryStruct << endl;
        cout << "object from dict is " << getObjDictName << endl;
        for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) { //iterate through object struct
            std::string getObjName = objectsFileStruct[isContext].object_name; //get object name from main struct
            if (DEBUG_objectLocationsCallbackDictionary1) {
                cout << "total objects in context is " << totalObjectsFileStruct << endl;
                cout << "total objects in struct is " << totalObjectsFileStruct << endl;
                cout << "object from context is " << getObjName << endl;
            }
            if (getObjDictName == getObjName) { //if object name in dictionary and main struct are equal
            cout << "found instance" << endl;
                objectDictionary[isDict].instances++; //add 1 to object instances
            }
            else {
                //don't do anything if match not found between dictionary and main object struct
            }
        }
    }
    //print out list and instances of objects
    if (DEBUG_objectLocationsCallbackDictionary2) {
        printSeparator(1);
        cout << "total number of objects are:" << endl;
        printSeparator(0);
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
            cout << objectDictionary[isDict].object_name << ":" << objectDictionary[isDict].instances << endl;
        }
        printSeparator(0);
    }

    //get objects and room count - e.g. 9 tvs in studio

}

int main(int argc, char** argv) {
    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    roomsDacopToStruct(rooms_dacop_loc);

    ros::init(argc, argv, "get_object_instances");
    ros::NodeHandle n;

    getObjectInstances();

    return 0;
}