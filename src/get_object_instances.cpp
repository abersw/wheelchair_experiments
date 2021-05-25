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
static const bool DEBUG_roomListToStruct = 1;
static const bool DEBUG_getObjectInstances1 = 0;
static const bool DEBUG_getObjectInstances2 = 1;
static const bool DEBUG_getObjectInstances3 = 0;
static const bool DEBUG_getObjectInstances4 = 1;

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

struct ObjectDictionary objectDictionaryInstances[1000]; //struct for storing data needed to calc uniqueness of objects
int totalObjectDictionaryInstancesStruct = 0; //total list of objects used to calc uniqueness

struct Rooms {
    int room_id;
    string room_name;

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
int totalRoomsFileStruct = 0;
//id,name,pointx,pointy,pointz,quatx,quaty,quatz,quatw
struct Rooms roomsFileStruct[1000];

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

/**
 * Take room list file and add it to struct array for processing later 
 *
 * @param parameter fileName is the path of the room.list file
 */
void roomListToStruct(std::string fileName) {
    if (DEBUG_roomListToStruct) {
        cout << "DEBUG_roomListToStruct" << endl;
    }
    //id, room name
    std::string objectsDelimiter = ","; //delimiter character is comma
	ifstream FILE_READER(fileName); //open file
    int roomNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << fileName << " file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            int lineSection = 0; //var for iterating through serialised line
            int pos = 0; //position of delimiter in line
            std::string token;
            while ((pos = line.find(objectsDelimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                //std::cout << token << std::endl;
                line.erase(0, pos + objectsDelimiter.length());
                //deserialise the line sections below:
                if (lineSection == 0) { //if first delimiter
                    roomsFileStruct[roomNumber].room_id = std::stoi(token); //convert room id string to int
                }
                else if (lineSection == 1) {
                    roomsFileStruct[roomNumber].room_name = token;
                }
                else if (lineSection == 2) {
                    roomsFileStruct[roomNumber].point_x = std::stof(token); //set transform point x
                }
                else if (lineSection == 3) {
                    roomsFileStruct[roomNumber].point_y = std::stof(token); //set transform point y
                }
                else if (lineSection == 4) {
                    roomsFileStruct[roomNumber].point_z = std::stof(token); //set transform point z
                }
                else if (lineSection == 5) {
                    roomsFileStruct[roomNumber].quat_x = std::stof(token); //set rotation to quaternion x
                }
                else if (lineSection == 6) {
                    roomsFileStruct[roomNumber].quat_y = std::stof(token);//set rotation to quaternion y
                }
                else if (lineSection == 7) {
                    roomsFileStruct[roomNumber].quat_z = std::stof(token); //set rotation to quaternion z
                }
                lineSection++; //move to next delimiter
            }
            roomsFileStruct[roomNumber].quat_w = std::stof(line); //set end of line to quat w

            if (DEBUG_roomListToStruct) {
                cout << 
                roomsFileStruct[roomNumber].room_id << "," << 
                roomsFileStruct[roomNumber].room_name << "," <<

                roomsFileStruct[roomNumber].point_x << "," <<
                roomsFileStruct[roomNumber].point_y << "," <<
                roomsFileStruct[roomNumber].point_z << "," <<

                roomsFileStruct[roomNumber].quat_x << "," <<
                roomsFileStruct[roomNumber].quat_y << "," <<
                roomsFileStruct[roomNumber].quat_z << "," <<
                roomsFileStruct[roomNumber].quat_w << endl;
            }
            roomNumber++; //go to next room line
        }
    }
    totalRoomsFileStruct = roomNumber; //set total number of rooms in struct to the lines(rooms) counted
    if (DEBUG_roomListToStruct) {
        cout << "total rooms in list are " << totalRoomsFileStruct << endl;
    }
}

void getObjectInstances() {
    for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) {
        //run through all objects
        int objectMatched = 0;
        std::string getObjName = objectsFileStruct[isContext].object_name;
        if (totalObjectDictionaryInstancesStruct == 0) {
            objectDictionaryInstances[0].object_name = getObjName; //set object name in first element in full objects struct
            totalObjectDictionaryInstancesStruct++; //add 1 to total objects in dictionary
        }
        for (int isDict = 0; isDict < totalObjectDictionaryInstancesStruct; isDict++) {
            std::string getObjDictName = objectDictionaryInstances[isDict].object_name;
            if (getObjName == getObjDictName) {
                objectMatched = 1;
            }
            //set objects back to 0
            objectDictionaryInstances[isDict].instances = 0;
        }
        if (objectMatched) {
            //if object is already in struct, don't add anything
        }
        else {
            //add object name to struct
            objectDictionaryInstances[totalObjectDictionaryInstancesStruct].object_name = getObjName;
            objectDictionaryInstances[totalObjectDictionaryInstancesStruct].instances = 0;
            totalObjectDictionaryInstancesStruct++;
        }
    }
    //print out list of objects
    if (DEBUG_getObjectInstances3) {
        printSeparator(1);
        cout << "pre-instance calculations, total size of struct is " << totalObjectDictionaryInstancesStruct << endl;
        for (int isDet = 0; isDet < totalObjectDictionaryInstancesStruct; isDet++) {
            cout << objectDictionaryInstances[isDet].object_name << ":" << objectDictionaryInstances[isDet].instances << endl;
        }
        printSeparator(1);
    }

    //get object instances and assign to object dictionary struct
    for (int isDict = 0; isDict < totalObjectDictionaryInstancesStruct; isDict++) { //iterate through object dictionary
        std::string getObjDictName = objectDictionaryInstances[isDict].object_name; //get object name from dictionary
        //printSeparator(1);
        //cout << "total objects in dictionary is " << totalObjectDictionaryStruct << endl;
        //cout << "object from dict is " << getObjDictName << endl;
        for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) { //iterate through object struct
            std::string getObjName = objectsFileStruct[isContext].object_name; //get object name from main struct
            //cout << "total objects in context is " << totalObjectContextStruct << endl;
            //cout << "total objects in struct is " << totalObjectsFileStruct << endl;
            //cout << "object from context is " << getObjName << endl;
            if (getObjDictName == getObjName) { //if object name in dictionary and main struct are equal
                //cout << "found instance" << endl;
                objectDictionaryInstances[isDict].instances++; //add 1 to object instances
            }
            else {
                //don't do anything if match not found between dictionary and main object struct
            }
        }
    }
    //print out list and instances of objects
    if (DEBUG_getObjectInstances4) {
        for (int isDict = 0; isDict < totalObjectDictionaryInstancesStruct; isDict++) {
            cout << objectDictionaryInstances[isDict].object_name << ":" << objectDictionaryInstances[isDict].instances << endl;
        }
    }
    cout << "end of instances" << endl;
    printSeparator(0);
}

void getObjectRooomInstances() {
    for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) {
        //run through all objects
        int objectMatched = 0;
        std::string getObjName = objectsFileStruct[isContext].object_name;
        std::string getRoomName = objectsFileStruct[isContext].room_name;
        if (totalObjectDictionaryStruct == 0) {
            objectDictionary[0].object_name = getObjName; //set object name in first element in full objects struct
            objectDictionary[0].room_name = getRoomName;
            totalObjectDictionaryStruct++; //add 1 to total objects in dictionary
        }
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
            std::string getObjDictName = objectDictionary[isDict].object_name;
            std::string getRoomDictName = objectDictionary[isDict].room_name;
            if ((getObjName == getObjDictName) && (getRoomName == getRoomDictName)) {
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
            objectDictionary[totalObjectDictionaryStruct].room_name = getRoomName;
            objectDictionary[totalObjectDictionaryStruct].instances = 0;
            totalObjectDictionaryStruct++;
        }
    }
    //print out list of objects
    if (DEBUG_getObjectInstances1) {
        printSeparator(1);
        cout << "pre-instance calculations, total size of struct is " << totalObjectDictionaryStruct << endl;
        for (int isDet = 0; isDet < totalObjectDictionaryStruct; isDet++) {
            cout << objectDictionary[isDet].object_name << ":" << objectDictionary[isDet].room_name << ":" << objectDictionary[isDet].instances << endl;
        }
        printSeparator(1);
    }

    //get object instances and assign to object dictionary struct
    for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) { //iterate through object dictionary
        std::string getObjDictName = objectDictionary[isDict].object_name; //get object name from dictionary
        std::string getRoomDictName = objectDictionary[isDict].room_name;
        //printSeparator(1);
        //cout << "total objects in dictionary is " << totalObjectDictionaryStruct << endl;
        //cout << "object from dict is " << getObjDictName << endl;
        for (int isContext = 0; isContext < totalObjectsFileStruct; isContext++) { //iterate through object struct
            std::string getObjName = objectsFileStruct[isContext].object_name; //get object name from main struct
            std::string getRoomName = objectsFileStruct[isContext].room_name;
            //cout << "total objects in context is " << totalObjectContextStruct << endl;
            //cout << "total objects in struct is " << totalObjectsFileStruct << endl;
            //cout << "object from context is " << getObjName << endl;
            if ((getObjDictName == getObjName) && (getRoomDictName == getRoomName)) { //if object name in dictionary and main struct are equal
                //cout << "found instance" << endl;
                objectDictionary[isDict].instances++; //add 1 to object instances
            }
            else {
                //don't do anything if match not found between dictionary and main object struct
            }
        }
    }
    //print out list and instances of objects
    if (DEBUG_getObjectInstances2) {
        for (int isDict = 0; isDict < totalObjectDictionaryStruct; isDict++) {
            cout << objectDictionary[isDict].object_name << ":" << objectDictionary[isDict].room_name << ":" << objectDictionary[isDict].instances << endl;
        }
    }
}

int main(int argc, char** argv) {
    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    rooms_dacop_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_dacop_name; //concatenate vars to create location of rooms dacop file
    roomsDacopToStruct(rooms_dacop_loc);
    
    rooms_list_loc = wheelchair_dump_loc + dump_dacop_loc + rooms_list_name; //concatenate vars to create location of rooms list
    roomListToStruct(rooms_list_loc);

    ros::init(argc, argv, "get_object_instances");
    ros::NodeHandle n;

    getObjectInstances();
    getObjectRooomInstances();

    return 0;
}