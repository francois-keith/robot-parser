#ifndef WRITE_AMELIF_H
#define WRITE_AMELIF_H

#include "robot.h"

#include <string>
#include <map>
#include <vector>

#include <fstream>

// main method

// method to write the multibody into the amelif format.
void writeAmelif(const Robot::MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName);


// tools method.

// write a Mb into Amelif format.
void writeXmlFile(const Robot::MultiBody* mb, const std::string & xmlfile, const std::string & robotName,
		const std::map<std::string,std::vector<std::string> > & bodyNameMap);

// write a Body into Amelif format.
void writeAmelif(const Robot::Body*  b, std::ofstream & xml, const std::map<std::string,std::vector<std::string> > & bodyNameMap,
		const std::string & vrmlPath);

// write a Joint into Amelif format.
void writeAmelif(const Robot::Joint* j, std::ofstream & xml);

// write the wrl files corresponding to the bounding volumes
//  of the two feets.
void createBoundingVolumeFiles(const std::string & xmlfile, const Robot::MultiBody* mb);

#endif
