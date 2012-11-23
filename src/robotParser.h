#ifndef _Robot
#define _Robot

#include "robot.h"

// Parse sensor data
void parseSensor(Robot::MultiBody* mb, std::ifstream & ifs_sensor);

// method to write the multibody into the urdf format.
void writeRos   (const Robot::MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName);


// Reset the multibody parsed.
void cleanupStaticData();

// Create a robot based on the given files.
int createRobot();

// Create a romeo robot based on the given files (quite specific).
int createRomeo();

#endif

