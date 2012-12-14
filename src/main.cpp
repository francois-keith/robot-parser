#include "robotParser.h"
#include "footDimensions.h"
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
	DECLARE_IFSTREAM(file_torqueLimits, ("../data/"+robotName+"/torqueLimits.dat").c_str());
	DECLARE_IFSTREAM(file_TagToBody,    ("../data/"+robotName+"/tag_to_bodyname.txt").c_str());

	// if the wrml file is not found, stop right there.
	if (!file_wrl)
		return 1;

	//parsing
	MultiBody * mb = new MultiBody(robotName);
	mb->isRomeo_ = (robotName.size() > 5 && robotName.substr(0,5) == "romeo");
	mb->parseWRL(file_wrl);
	if(file_jointLimits)
		mb->parseJointLimits(file_jointLimits);
	if(file_torqueLimits)
		mb->parseTorqueLimits(file_torqueLimits);
	if(file_TagToBody)
		mb->parseTags(file_TagToBody);

	loadFeetData(fileName, mb);

	// Finalize the parsing
	//update the outer - inner body
	int numJoints = mb->nbJoints();
	for (int i=0; i<numJoints; ++i)
	{
		bool valid = true;
		std::map<int,Joint*>::iterator it = mb->jointMap_.find(i);
		Joint* j = it->second;
		assert(j!=0x0  && "Null joint");

		// update the inner, outer body info with the ids
		std::map<std::string,int>::iterator itb = Body::bodyNameToId_.find(j->innerBodyName_);
		if ( itb != Body::bodyNameToId_.end() )
			j->innerBodyId_=itb->second;
		else
		{
			j->innerBodyId_= -1;
			std::cerr << "Unable to find inner body " << j->innerBodyName_ << " for the joint " << j->name_ << std::endl;
		}

		std::map<std::string,int>::iterator it2 = Body::bodyNameToId_.find(j->outerBodyName_);
		if ( it2 != Body::bodyNameToId_.end() )
			j->outerBodyId_=it2->second;
		else
		{
			j->outerBodyId_= -1;
			std::cerr << "Unable to find outer body " << j->outerBodyName_ << " for the joint " << j->name_ << std::endl;
		}




		// check existence of inner Body
		if (mb->bodyMap_.find(j->innerBodyId_) != mb->bodyMap_.end() )
		{
			j->innerBody_ = mb->bodyMap_[j->innerBodyId_];
			mb->bodyMap_[j->innerBodyId_]->outerJointList_.push_back(j);
		}
		else
			valid = false;

		// check existence of outer Body
		if (mb->bodyMap_.find(j->outerBodyId_) != mb->bodyMap_.end() )
			j->outerBody_ = mb->bodyMap_[j->outerBodyId_];
		else
			valid = false;

		// if the joint is not valid, remove it
		if ( valid == false )
		{
			std::cerr << "/!\\ The joint " << j->name_ << " cannot be considered as valid  "
					<< "(in: " << j->innerBodyId_ << ",out: " << j->outerBodyId_ <<	")" << std::endl;
			mb->jointMap_.erase(i);
		}
	}

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

	// write data for Humans.
	std::string mapleFolder = (robotName+"_maple/src/MapleCodeGeneration/");
	writeMaple(mb, mapleFolder, robotName, useFingers);

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

