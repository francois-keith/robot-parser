#ifndef WRITE_OPENHRP
#define WRITE_OPENHRP

namespace Robot
{
	class MultiBody;
	class Body;
	class Joint;
}

//-- method to write the whole wrl file (for openHRP)
// the whole robot is contained in one wrl.
void writeOpenHRP(const Robot::MultiBody* mb, const std::string & openhrpFolder,
		const std::string & robotNameForHeader,
		const std::string & robotName, bool useToes);
// sub method to write a body
void writeOpenHRP (const Robot::Body* body, const std::string & space, std::ofstream & out);
// sub method to write a joint
void writeOpenHRP (const Robot::Joint* joint, const std::string & space, std::ofstream & out);

// Write the header for the (sot v1)
void writeRomeoHeader(
	const std::string & openhrpFolder,
	const std::string & robotNameForHeader,
	const std::string & robotName,
	const Robot::MultiBody* mb,
	const std::map<unsigned, std::string> & limbMap,
	std::vector<double> & initialPosition
);

// Write the initial configuration (sot v1)
void writeInitConfig(
	const std::string & openhrpFolder,
	const std::string & robotName,
	const Robot::MultiBody* mb,
	const std::map<unsigned, std::string> & limbMap,
	std::vector<double> & initialPosition
);

// write the file LinkJointRank (sot v1)
void writeOpenHRPLinkJointRank (
	const Robot::MultiBody* mb,
	const std::string & openhrpFolder, const std::string & robotName);

// write the file Specificities (sot v1)
void writeOpenHRPSpecificities (
	const Robot::MultiBody* mb,
	const std::string & openhrpFolder, const std::string & robotName, bool useToes);

void writeFootPosition(std::ofstream & out,
	double rear, double front, double left, double right, double height	);

void writeHandPosition(std::ofstream & out);
//

// Write the specificities in python format (sot v2.5)
void writeRobotSpecificPython(
	const std::string & openhrpFolder,
	const std::string & robotNameForHeader,
	const std::string & robotName,
	const Robot::MultiBody* mb,
	const std::map<unsigned, std::string> & limbMap,
	const std::vector<double> & initialPosition
);

#endif //WRITE_OPENHRP
