#include "robotParser.h"
#include "tools.h"

#include "writeAmelif.h"
#include "writeMaple.h"
#include "writeOpenHRP.h"

#include <cstdlib>

using namespace Robot;
using namespace std;

int createRomeo(bool useToes, bool useHands, const std::string & afName, const std::string & mapleName)
{
	cleanupStaticData();

	DECLARE_IFSTREAM(file_bodies, "../doc/romeo/hardware_romeo_mass.html");
	DECLARE_IFSTREAM(file_joints, "../doc/romeo/hardware_romeo_joint.html");
	DECLARE_IFSTREAM(file_links,  "../doc/romeo/hardware_romeo_kinematic.html");
	DECLARE_IFSTREAM(file_motor,  "../doc/romeo/hardware_romeo_motor.html");
	DECLARE_IFSTREAM(file_sensor,  "../data/camera.txt");
	DECLARE_IFSTREAM(file_bodies2, "../doc/romeo/hands_link.xml");
	DECLARE_IFSTREAM(file_joints2, "../doc/romeo/hands_joint.xml");

	if (!file_bodies || !file_joints || !file_links)
		return 1;

	//parsing
	MultiBody * mb = new MultiBody(afName);
	while(mb->parseBody(file_bodies, useToes)) {}
	if(useHands)
		while(mb->parseBody2(file_bodies2, useToes)) {}

	parseSensor(mb, file_sensor);
	file_sensor.close();

	while(mb->parseJoint(file_joints, useToes)){}
	if(useHands)
		while(mb->parseJoint2(file_joints2, useToes)){}

	while(mb->parseLink(file_links, useToes)){}
	file_bodies.close();
	file_bodies2.close();
	file_joints.close();
	file_joints2.close();
	file_links.close();

	mb->parseMotor(file_motor);
	file_motor.close();



	//update the outer - inner body
	int numJoints = mb->nbJoints();
	for (int i=0; i<numJoints; ++i)
	{
		bool valid = true;
		std::map<int,Joint*>::iterator it = mb->jointMap_.find(i);
		Joint* j = it->second;

		// update the inner, outer body info with the ids


		//patch
		// the joint TrunkYaw is modified: it goes from the waist to the chest now
		if (j->name_ == "TrunkYaw")
		{
			j->staticXYZ_ *= -1.;
			std::string tmp = j->outerBodyName_;
			j->outerBodyName_ = j->innerBodyName_;
			j->innerBodyName_ = tmp;
		}
		//end patch

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
//		else
//		{
//			std::cout << j->name_
//					<< " in  (" << j->innerBodyName_ << ", " << j->innerBodyId_ << ") "
//					<< " out (" << j->outerBodyName_ << ", " << j->outerBodyId_ << ") "
//					<< std::endl;
//		}
	}

	mb->computeGazePosition();

	// express all the camera in the frame of the gaze.
	for (std::map<std::string,Camera*>::iterator it = mb->cameraMap_.begin();
			it != mb->cameraMap_.end(); ++it)
	{
		it->second->pos_ -= mb->gazePosition_;
	}

	int numCameraJoints = mb->cameraJointMap_.size();
	for (std::map<int,Joint*>::iterator it = mb->cameraJointMap_.begin();
			it != mb->cameraJointMap_.end(); ++it)
	{
		Joint* j = it->second;
		std::cout << " j->name_ " << j->name_;
		std::string side = j->name_.substr(0,1);
		// update the inner, outer body info with the ids

		if(j->name_.size() >=7 && j->name_.substr(1,6) == "EyeYaw")
		{
			j->innerBodyName_ = "gaze";
			j->outerBodyName_ = side+"EyePitch_link";
			if(side == "L")
			{
				j->staticXYZ_ = mb->cameraMap_["CameraLeftEye"]->pos_;
			}
			else if(side == "R")
			{
				j->staticXYZ_ = mb->cameraMap_["CameraRightEye"]->pos_;
			}
		}
		else if(j->name_.size() >=9 && j->name_.substr(1,8) == "EyePitch")
		{
			j->innerBodyName_ = side+"EyePitch_link";
			if(side == "L")
				j->outerBodyName_ = "CameraLeftEye";
			else
				j->outerBodyName_ = "CameraRightEye";
		}
		else
		{
			std::cerr << " /!\\ Issue during the camera joint handling : joint " << j->name_ << " not found" << std::endl;
		}
	}
	mb->cameraMap_.erase("CameraLeftEye");
	mb->cameraMap_.erase("CameraRightEye");

	// write data for amelif.
	std::string amelifFolder ("romeo_af/data/xml/");
	writeAmelif(mb, amelifFolder, afName, mapleName);

	// write data for Humans.
	std::string mapleFolder = ("romeo_maple/"+mapleName+"/src/MapleCodeGeneration/");
	system( ("mkdir -p " + mapleFolder).c_str() );
	writeMaple(mb, mapleFolder, mapleName, useToes);

	// write data for the sot.
	string openHRPFolder("romeo_sot/data/");
	writeOpenHRP(mb, openHRPFolder, afName, mapleName, useToes);

	// write data for the sot.
	string urdfFolder("romeo_urdf/romeo_description/");
	writeRos(mb, urdfFolder, afName, mapleName);

	delete mb;
	return 0;
}


// main
int createAllRomeo ()
{
	std::cout << " ** Creating the model with the toes ** " << std::endl;
	createRomeo(true, false, "romeo", "Romeo");
	std::cout << std::endl;

	std::cout << " ** Creating the model without toes ** " << std::endl;
	createRomeo(false, false, "romeo_notoes", "RomeoNoToes");
	std::cout << std::endl;

	std::cout << " ** Creating the model with the toes and fingers ** " << std::endl;
	createRomeo(true, true, "romeo_hands", "RomeoHands");
	std::cout << std::endl;

	std::cout << " ** Creating the model with the toes and fingers ** " << std::endl;
	createRomeo(false, true, "romeo_hands_notoes", "RomeoHandsNoToes");

	return 0;
}

