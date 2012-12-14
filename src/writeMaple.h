#ifndef _WRITE_MAPLE_H_
#define _WRITE_MAPLE_H_

#include "robot.h"

#include <fstream>
#include <sstream>

//writeMaple
void writeMaple(
	const Robot::MultiBody* mb,
	const std::string & mapleFolder,
	const std::string & robotName,
	bool useToes
	);

// write the LagrangianModel data
void writeLagrangianFile(const std::string & lagModelFile,
		int numJoints,
		int numContactBodies);

void writeMaple (const Robot::Body* b, std::ofstream & outDynamicMaple);
void writeMaple (const Robot::Joint* j, std::stringstream & data_kinematic);

//declaration of a useful method to build lagrangian data file.
void writeLagrangianFile(const std::string & lagModelFile,
		int numJoints,
		int numContactBodies);

//declaration of useful methods to build Additional data file.
void createAdditionnalData (
	const std::string & addDataFile,
	const std::string & robotName,
	const std::string & version,
	int numContactBodies,
	bool useToes
);

void createPrefixForAdditionnalData (std::ofstream & outAddMaple,
	int numContactSolids,
	const std::string & robotName,
	const std::string & version
);

void createFootTagForAdditionnalData(std::ofstream & outAddMaple,
	int startingIndex, const std::string & lbName,
	double rear, double front, double left, double right, double height
);

void createDisplayTagForAdditionnalData(std::ofstream & outAddMaple,
		const std::vector<std::string> & nameList, int startingIndexAddData);

#endif // _WRITE_MAPLE_H_
