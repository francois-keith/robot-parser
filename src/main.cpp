#include "robotParser.h"
#include "../data/romeo/footDimensions.h"
#include "tools.h"

#include "writeAmelif.h"
#include "writeMaple.h"
#include "writeOpenHRP.h"

using namespace Robot;
using namespace std;

void cleanupStaticData()
{
	// cleanup
	Body::bodyGlobalId_ = (0);
	Body::bodyNameToId_.clear();
	Body::bodyNameToVRML_.clear();
	Body::bodyNameToVRML_BV_.clear();

	Joint::jointGlobalId_ = (0);
	Joint::jointNameToId_.clear();
	Joint::anonymousJointId_ = 0;
	// end cleanup
}

int createRobot(const std::string & fileName, const std::string & robotName, bool useFingers)
{
	std::cout  << std::endl << " ** Creating the model for "<< fileName <<" ** useFingers " << useFingers << std::endl;
	cleanupStaticData();

	DECLARE_IFSTREAM(file_wrl,         ("../data/"+robotName+"/VRMLmain.wrl").c_str());
	DECLARE_IFSTREAM(file_jointLimits, ("../data/"+robotName+"/jointLimits.dat").c_str());

	// if the wrml file is not found, stop right there.
	if (!file_wrl)
		return 1;

	//parsing
	MultiBody * mb = new MultiBody(robotName);
	mb->isRomeo_ = (robotName.size() > 5 && robotName.substr(0,5) == "romeo");
	mb->parseWRL(file_wrl, file_jointLimits);

	if (mb->isRomeo_)
		loadRomeoFeetData(mb);

	// What do we do with the fingers?
	std::map<int,Joint*>::iterator it;
	for(it = mb->jointMap_.begin(); it != mb->jointMap_.end(); )
	{
		// if the id is negative (unofficial joint, we add it at the end
		//  after the other joints.
		if (it->first < 0)
		{
			Joint * joint = it->second;
			int idInit = joint->id_;
			joint->id_ = Joint::jointGlobalId_ - idInit;

			Joint::jointNameToId_[joint->name_] = joint->id_;
			mb->jointMap_[joint->id_] = joint;
			++it;

			if (! useFingers)
				joint->type_ = "fixed";

			mb->jointMap_.erase(idInit);
		}
		else
		{
			++it;
		}
	}


	// write data for amelif.
	std::string amelifFolder (robotName+"_af/data/");
	writeAmelif(mb, amelifFolder, fileName, robotName);

	// write data for the sot.
	string openHRPFolder(robotName+"_sot/data/");
	writeOpenHRP(mb, openHRPFolder, fileName, robotName, useFingers);

	// write data for the sot.
	string urdfFolder(robotName+"_urdf/"+robotName+"_description/");
	writeRos(mb, urdfFolder, fileName, robotName);

	delete mb;
	return 0;
}

// main
int main ()
{
	createRobot("robot",       "robot", true);
	createRobot("robot_small", "robot", false);

//	createAllRomeo();

	return 0;
}

