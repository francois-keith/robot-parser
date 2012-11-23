#include "robotParser.h"
#include "tools.h"

using namespace Robot;
using namespace std;

int	Body::bodyGlobalId_ = (0);
std::map<std::string,int>         Body::bodyNameToId_;
std::map<std::string,std::vector<std::string> > Body::bodyNameToVRML_;
std::map<std::string,std::vector<std::string> > Body::bodyNameToVRML_BV_;
std::map<std::string,std::string> Body::bodyNameToURDF_;

int	Joint::jointGlobalId_ = (0);
int	Joint::anonymousJointId_ = (0);
std::map<std::string,int>         Joint::jointNameToId_;

Motor::Motor()
: motor_()
, model_()
, noLoadSpeed_(0)
, torqueCst_(0)
{}


MultiBody::MultiBody(const std::string & robotName)
: bodyMap_( )
, jointMap_( )
, gazePosition_(0,0,0)
, cameraJointGlobalId_(0)
, cameraJointMap_()
, cameraJointNameToId_()
, version_("0")

, ankle_rearBound_(0)
, ankle_frontBound_(0)
, ankle_rightBound_(0)
, ankle_leftBound_(0)
, ankle_Height_(0)

, toe_rearBound_(0)
, toe_frontBound_(0)
, toe_rightBound_(0)
, toe_leftBound_(0)
, toe_Height_(0)
{
	Body::bodyGlobalId_ = (0);
	Joint::jointGlobalId_ = (0);
	Body::bodyNameToId_.clear();
	Body::bodyNameToVRML_.clear();
	Body::bodyNameToVRML_BV_.clear();
	Body::bodyNameToURDF_.clear();

	getBodyNameToURDF(robotName);
	getBodyNameToVRML(robotName);
	ifstream versionFile ("../data/version.txt");
	if (! versionFile)
	{
		std::cerr << "The version of the robot is unknown" <<std::endl;
		return;
	}
	versionFile >> version_;
	versionFile.close();
}


MultiBody::~MultiBody()
{
	std::map<int,Body*>::iterator it;
	for (it=bodyMap_.begin(); it!= bodyMap_.end(); ++it)
	{
		delete it->second;
	}
	bodyMap_.clear();

	std::map<int,Joint*>::iterator it2;
	for (it2=jointMap_.begin(); it2!= jointMap_.end(); ++it2)
	{
		delete it2->second;
	}
	jointMap_.clear();
}

Joint::Joint(): id_(-1)
, name_("")
, positionMin_(0)
, positionMax_(0)
, speedMax_(0)
, torqueMax_(0)

, outerBody_(0x0)
, outerBodyId_(-1)
, outerBodyName_("")

, innerBody_(0x0)
, innerBodyId_(-1)
, innerBodyName_("")

, type_("revolute")
, axis_(0,0,0)
, staticXYZ_(0,0,0)
, staticRPY_(0,0,0)
, rotorInertia_(0)
{}

int MultiBody::nbBodies(void) const
{
	return bodyMap_.size();
}

int MultiBody::nbJoints(void) const
{
	return jointMap_.size();
}

void MultiBody::getBodyNameToVRML(const std::string & robotName)
{
	DECLARE_IFSTREAM(in, ("../data/"+robotName+"/bodynametovrml.txt").c_str());
	string line;
	int pos;
	while(getline(in, line))
	{
		istringstream smallData(line, ios_base::in);
		std::string bodyName, vrml, tmp;
		smallData >> bodyName >> tmp;
		while ( smallData >> vrml )
			Body::bodyNameToVRML_[bodyName].push_back(vrml);
	}
	in.close();

	DECLARE_IFSTREAM(in2, ("../data/"+robotName+"/bodynametovrml_BV.txt").c_str());
	while(getline(in2, line))
	{
		istringstream smallData(line, ios_base::in);
		std::string bodyName, vrml, tmp;
		smallData >> bodyName >> tmp;
		while ( smallData >> vrml )
			Body::bodyNameToVRML_BV_[bodyName].push_back(vrml);
	}
	in2.close();
}

void MultiBody::getBodyNameToURDF(const std::string & robotName)
{
	DECLARE_IFSTREAM(in, ("../data/"+robotName+"/bodynametourdf.txt").c_str());
	string line;
	int pos;
	while(getline(in, line))
	{
		istringstream smallData(line, ios_base::in);
		std::string bodyName, urdf, tmp;
		smallData >> bodyName >> tmp >> urdf;
		Body::bodyNameToURDF_[bodyName]=(urdf);
	}
	in.close();
}

//parsers
bool MultiBody::parseBody (ifstream & ifs, bool useToes)
{
	string line;
	bool endoffile= true;
	bool moveon = false;
	while (moveon == false)
	{
		while ( (endoffile = getline(ifs, line) ) && (line.find("<div class=\"section\"") == string::npos) ){}

		if ( ! endoffile )
		{
			return false;
		}
		endoffile = getline(ifs, line);

		if (line.find("<h3>") != string::npos)
			moveon = true;
	}

	++Body::bodyGlobalId_;

	Body *body = new Body();
	body->ParseName(line);
	if (body->name_=="Romeo"){
		--Body::bodyGlobalId_;
		return parseBody (ifs, useToes);
	}


	// get themass
	while ( getline(ifs, line) && (line.find("text{Mass} =")) == string::npos){}
	body->mass_ = parseTableElement<double>(line, 0, "text{Mass} = ", "\"/></p>");

	body->ParseCom(ifs);
	body->ParseInertia(ifs);
	body->id_ = Body::bodyGlobalId_;
	
	// patch for the waist 
	if (body->name_=="TrunkYaw")
	{
		body->id_ = 0;
		--Body::bodyGlobalId_;
	}

	// patch for the name of the foot
	if ( body->name_ == "RFoot (first prototype)")
		body->name_ = "RAnkleRollBasic";

	// the bodies LAnkleRollBasic and RAnkleRollBasic are not taken into account for now
	if ( useToes)
	{
		// For the model with the toes, the following body should not be taken into account
		if ( (body->name_ == "RAnkleRollBasic") || (body->name_ == "LAnkleRollBasic") )
		{
			--Body::bodyGlobalId_;
			delete body;
		}
		else
		{
			Body::bodyNameToId_[body->name_]=body->id_;
			bodyMap_[body->id_] = body;
		}
	}
	else
	{
		// For the model without toes, the following bodies should not be taken into account
		if ( (body->name_ == "RAnkleRoll") || (body->name_ == "LAnkleRoll")
			|| (body->name_ == "RToePitch") || (body->name_ == "LToePitch") )
		{
			--Body::bodyGlobalId_;
			delete body;
		}
		else
		{
			Body::bodyNameToId_[body->name_]=body->id_;
			bodyMap_[body->id_] = body;
		}

	}

	return true;
}


bool MultiBody::parseBody2 (ifstream & ifs, bool useToes)
{
	string line;
	bool endoffile= true;
	bool moveon = false;
	while (moveon == false)
	{
		while ( (endoffile = getline(ifs, line) ) && (line.find("<link name") == string::npos) ){}
		if ( ! endoffile )
		{
			return false;
		}
		moveon = true;
	}
	++Body::bodyGlobalId_;

	Body *body = new Body();
	// parse name
	body->name_ = parseTableElement<std::string>(line, "", "<link name=\"", "\">");
	std::string tmp0 = body->name_.substr(body->name_.size()-4,body->name_.size());
	if (tmp0 == "Link")
		body->name_ = body->name_.substr(0, body->name_.size()-4);

	body->id_ = Body::bodyGlobalId_;
	bodyMap_[body->id_] = body;
	Body::bodyNameToId_[body->name_]=body->id_;

	return true;
}

bool MultiBody::parseJoint (ifstream & ifs_joints, bool useToes)
{
	string line;
	bool endoffile= true;
	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<td>") == string::npos) ){}
	if ( ! endoffile )
		return false;

	++Joint::jointGlobalId_;

	Joint * joint = new Joint;
	joint->ParseName(line);

	joint->ParseAxis(ifs_joints);
	joint->ParseRange(ifs_joints);
	getline(ifs_joints, line);

	joint->id_ = Joint::jointGlobalId_;

	if ( (joint->name_ == "LEyeYaw") || (joint->name_ == "REyeYaw")
      || (joint->name_ == "LEyePitch") || (joint->name_ == "REyePitch")
      )
	{
		--Joint::jointGlobalId_;

		joint->id_ =MultiBody::cameraJointGlobalId_;
		cameraJointMap_[joint->id_] = joint;
		cameraJointNameToId_[joint->name_] = joint->id_;
		++MultiBody::cameraJointGlobalId_;
		return true;
	}

	// patch !
	if(useToes == false)
	{
		if ( (joint->name_ == "RToePitch") || (joint->name_ == "LToePitch") )
		{
			--Joint::jointGlobalId_;
			delete joint;
			return true;
		}
	}
	// end patch


	// the joint TrunkYaw is modified: it goes from the waist to the chest now 
	if (joint->name_ == "TrunkYaw")
	{
		joint->id_ = 0;
		--Joint::jointGlobalId_;
	}

	jointMap_[joint->id_] = joint;
	Joint::jointNameToId_[joint->name_] = joint->id_;

	return true;
}

bool MultiBody::parseJoint2 (ifstream & ifs_joints, bool useToes)
{
	string line;
	bool endoffile= true;
	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<joint name") == string::npos) ){}

	if ( ! endoffile )
		return false;
	++Joint::jointGlobalId_;

	Joint * joint = new Joint;
	joint->name_ = parseTableElement<std::string>(line, "", "<joint name=\"", "\" type");
	std::string tmp0 = joint->name_.substr(joint->name_.size()-4,joint->name_.size());
	if (tmp0 == "Link")
		joint->name_ = joint->name_.substr(0, joint->name_.size()-4);

	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<parent link") == string::npos) ){}
	joint->innerBodyName_  = parseTableElement<std::string>(line, "", "<parent link=\"", "\"/>");

	std::string tmp1 = joint->innerBodyName_.substr(joint->innerBodyName_.size()-4,joint->innerBodyName_.size());
	if (tmp1 == "Link")
		joint->innerBodyName_ = joint->innerBodyName_.substr(0, joint->innerBodyName_.size()-4);

	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<child link") == string::npos) ){}
	joint->outerBodyName_  = parseTableElement<std::string>(line, "", "<child link=\"", "\"/>");

	std::string tmp2 = joint->outerBodyName_.substr(joint->outerBodyName_.size()-4,joint->outerBodyName_.size());
	if (tmp2 == "Link")
		joint->outerBodyName_ = joint->outerBodyName_.substr(0, joint->outerBodyName_.size()-4);

	if( joint->innerBodyName_  == "RWristPitchLink")
		joint->innerBodyName_  = "RWristPitch";

	if( joint->innerBodyName_  == "LWristPitchLink")
		joint->innerBodyName_  = "LWristPitch";

	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<origin xyz=") == string::npos) ){}
	joint->staticXYZ_ = parseTableElement<afbase::vector3d>(line, afbase::vector3d(), "<origin xyz=\"", "\" rpy");
	joint->staticRPY_ = parseTableElement<afbase::vector3d>(line, afbase::vector3d(), "rpy=\"", "\"/>");

	while ( (endoffile = getline(ifs_joints, line) ) && (line.find("<axis xyz=") == string::npos) ){}
	joint->axis_ = parseTableElement<afbase::vector3d>(line, afbase::vector3d(), "<axis xyz=\"", "\"/>");

	joint->positionMin_ = 0;
	joint->positionMax_ = 61;


	joint->id_ = Joint::jointGlobalId_;

	jointMap_[joint->id_] = joint;
	Joint::jointNameToId_[joint->name_] = joint->id_;

	return true;
}

bool MultiBody::parseLink(ifstream & ifs_links, bool useToes)
{
	string line;
	bool endoffile= true;
	while ( (endoffile = getline(ifs_links, line) ) && (line.find(" To ") == string::npos) ){}
	if ( ! endoffile )
		return false;


	std::string innerBodyName;
	std::string outerBodyName;
	parseTableElement2(line, innerBodyName, outerBodyName, "<td>", " To ", "</td>");
	std::map<std::string, int>::iterator it = Joint::jointNameToId_.find(outerBodyName);

	// patch !
	if(useToes == false)
	{
		if ( (outerBodyName == "RToePitch") || (outerBodyName== "LToePitch") )
			return true;
	}
	// end patch

	if (it == Joint::jointNameToId_.end())
	{
		std::cerr << "/!\\ Issue during the link parsing : joint not found "
			<< "   jointName " << outerBodyName
			<< "   ( " << innerBodyName << " -> " << outerBodyName << " ) "
			<< std::endl;
		return true;
	}
	Joint *joint = jointMap_[it->second];
	joint->ParseStaticsParameters(ifs_links);

	// patch !
	if(useToes == false)
	{
		if ( (joint->name_ == "RToePitch") || (joint->name_ == "LToePitch") )
		{
			return true;
		}
	}
	// end patch

	if(useToes == false)
	{
		if(outerBodyName == "RAnkleRoll")
			outerBodyName = "RAnkleRollBasic";
		else if(outerBodyName == "LAnkleRoll")
			outerBodyName = "LAnkleRollBasic";
	}

	joint->innerBodyName_ = innerBodyName;
	joint->outerBodyName_ = outerBodyName;

	return true;
}


void MultiBody::parseMotorList(ifstream & ifs_motor)
{
	string line;
	bool endoffile= true;

	// parse the first line containing the name of the motors
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tr ") == string::npos) ){}

	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("</tr>") == string::npos) )
	{
		Robot::Motor* mt = new Robot::Motor();
		mt->motor_ = parseTableElement<std::string>(line, "", "<th class=\"head\">", "</th>");
		motorVec_.push_back(mt);
	}

	// Parse model name
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tr ") == string::npos) ){}
	for (unsigned indexMotor=0; indexMotor<motorVec_.size(); ++indexMotor)
	{
		getline(ifs_motor, line);
		Robot::Motor * mt = motorVec_[indexMotor];
		mt->model_ = parseTableElement<std::string>(line, "");
	}

	// Parse noLoadSpeed_
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tr ") == string::npos) ){}
	for (unsigned indexMotor=0; indexMotor<motorVec_.size(); ++indexMotor)
	{
		getline(ifs_motor, line);
		Robot::Motor * mt = motorVec_[indexMotor];
		mt->noLoadSpeed_ = parseTableElement<double>(line, 0);
	}

	// Parse torque constant
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tr ") == string::npos) ){}
	for (unsigned indexMotor=0; indexMotor<motorVec_.size(); ++indexMotor)
	{
		getline(ifs_motor, line);
		Robot::Motor * mt = motorVec_[indexMotor];
		mt->torqueCst_ = parseTableElement<double>(line, 0);
	}

//	for(unsigned i=0; i<motorVec_.size(); ++i)
//	{
//		std::cout << "Motor " << std::endl;
//		std::cout << "   motor_ " << motorVec_[i]->motor_ << std::endl;
//		std::cout << "   model_ " << motorVec_[i]->model_ << std::endl;
//		std::cout << "   noLoadSpeed_ " << motorVec_[i]->noLoadSpeed_ << std::endl;
//		std::cout << "   torqueCst_ " << motorVec_[i]->torqueCst_ << std::endl;
//		std::cout << std::endl;
//	}
}

double MultiBody::getTorqueOfMotor(const std::string & name)
{
	for(unsigned i=0; i<motorVec_.size(); ++i)
		if(motorVec_[i]->motor_ == name)
			return motorVec_[i]->torqueCst_;

	return 0.;
}



bool MultiBody::parseMotor(ifstream & ifs_motor)
{
	string line;
	bool endoffile= true;

	parseMotorList(ifs_motor);


	// Skip useless info
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("Max Velocity and Max Torque") == string::npos) ){}
	if ( ! endoffile )
		return false;

	// Go to the interesting table.
	while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tbody ") == string::npos) ){}
	if ( ! endoffile )
		return false;

	// Parse all the joints.
	while ( endoffile )
	{
		while ( (endoffile = getline(ifs_motor, line) ) && (line.find("<tr ") == string::npos) ) {}
		if ( ! endoffile )
			return false;

		// get the joint name
		std::string jointName = parseTableElement<std::string>(line, "");

		// motor name
		getline(ifs_motor, line);
		std::string motorModel = parseTableElement<std::string>(line, "");

		// motor reduction
		getline(ifs_motor, line);
		double motorReduction = parseTableElement<double>(line, 0);

		//  max vel
		getline(ifs_motor, line);
		double maxVel = parseTableElement<double>(line, 0);

		bool jointFound = false;
		double ampere = 3.0;

		// cf mail de Cyrille Collette
		// The articulations HipRoll/HipPitch/AnkleRoll/AnklePitch sont couplées par un différentiel.
		// Il faut que tu doubles les couples maximums pour ces articulations car les 2 moteurs peuvent contribuer.

		if (jointName == "HipRoll" || jointName == "HipPitch" || jointName == "AnkleRoll" || jointName == "AnklePitch")
		{
			ampere *= 2;
		}

		// Get the corresponding joint(s)
		{
			std::map<std::string, int>::iterator it = Joint::jointNameToId_.find(jointName);
			if (it != Joint::jointNameToId_.end())
			{
				Joint *joint = jointMap_[it->second];
				joint->speedMax_  = maxVel;
				double torqueConstant = getTorqueOfMotor(motorModel);
				joint->torqueMax_ = ampere * torqueConstant * motorReduction;
				jointFound=true;
			}
		}
		{
			std::map<std::string, int>::iterator it = Joint::jointNameToId_.find("L"+jointName);
			if (it != Joint::jointNameToId_.end())
			{
				Joint *joint = jointMap_[it->second];
				joint->speedMax_  = maxVel;
				double torqueConstant = getTorqueOfMotor(motorModel);
				joint->torqueMax_ = ampere * torqueConstant * motorReduction;
				jointFound=true;
			}
		}
		{
			std::map<std::string, int>::iterator it = Joint::jointNameToId_.find("R"+jointName);
			if (it != Joint::jointNameToId_.end())
			{
				Joint *joint = jointMap_[it->second];
				joint->speedMax_  = maxVel;
				double torqueConstant = getTorqueOfMotor(motorModel);
				joint->torqueMax_ = ampere * torqueConstant * motorReduction;
				jointFound=true;
			}
		}
	}

	return true;
}

void MultiBody::computeGazePosition()
{
	std::map<std::string,Camera*>::const_iterator itL = cameraMap_.find("CameraLeftEye");
	std::map<std::string,Camera*>::const_iterator itR = cameraMap_.find("CameraRightEye");

	if (itL == cameraMap_.end())
	{
		std::cerr <<  " Left Eye camera not found" << std::endl;
		return;
	}
	if (itR == cameraMap_.end())
	{
		std::cerr <<  " Left Right camera not found" << std::endl;
		return;
	}

	const Camera *camLeftEye_  = itL->second;
	const Camera *camRightEye_ = itR->second;

	gazePosition_ = camLeftEye_->pos_ + camRightEye_->pos_;
	gazePosition_ *= 0.5;
}

Body::Body()
: outerJointList_()
, id_(0)
, name_("")
, mass_(0)
, com_(0,0,0)
, inertia_(0,0,0, 0,0,0, 0,0,0)
{}

//Body parsers
void Body::ParseName (ifstream & ifs)
{
	string line;
	while ( (line.find("<h3>")) == string::npos)
		getline(ifs, line);

	int found1=line.find("<h3>");
	int found2=line.find("<a class");

	name_ = line.substr (found1,found2 - found1); // "generalities"
}

void Body::ParseName (const std::string & line)
{
	int found1=line.find("<h3>");
	int found2=line.find("<a class");

	name_ = line.substr (found1+4,found2 - found1-4); // "generalities"
}

void Body::ParseCom (ifstream & ifs)
{
	string line;
	getline(ifs, line);

	while ( (line.find("text{CoM(S)}")) == string::npos)
		getline(ifs, line);

	for (int i=0; i <3;  ++i)
	{
		getline(ifs, line);
		istringstream smallData(line, ios_base::in);
		smallData >> com_(i);
	}
}

void Body::ParseInertia (ifstream & ifs)
{
	string line;
	getline(ifs, line);

	while ( (line.find("left[I_o(S)")) == string::npos)
		getline(ifs, line);

	string tmp1;

	for (int nline=0; nline <3; ++nline)
	{
		getline(ifs, line);
		istringstream smallData(line, ios_base::in);
		for (int ncol=0; ncol <3;  ++ncol)
			smallData >> inertia_(nline, ncol) >> tmp1;
	}
}

void Body::display () const
{
	std::cout << " Body " << name_ << ", " << id_ << std::endl;
	std::cout << " mass_ " << mass_ << std::endl;
	std::cout << " com_ " << com_ << std::endl;
	std::cout << " inertia_ " << std::endl << inertia_ << std::endl;
	std::cout << std::endl;
}

void Joint::ParseName(string line)
{
	name_ = parseTableElement<std::string>(line, "");
}

void Joint::ParseAxis(ifstream & ifs_joints)
{
	string line;
	getline(ifs_joints, line);
	int found=line.find("</td>");
	char axe = tolower(line.substr(found-2,1)[0]);
	if (axe == 'x')
		axis_ = afbase::vector3d(1,0,0);
	else if (axe == 'y')
		axis_ = afbase::vector3d(0,1,0);
	else if (axe == 'z')
		axis_ = afbase::vector3d(0,0,1);
	else
		std::cerr << "Joint::ParseAxis error " << std::endl;
}

void Joint::ParseRange(ifstream & ifs_joints)
{
	string line;
	getline(ifs_joints, line);
	parseTableElement2_double(line, positionMin_, positionMax_, "<td>", "to", "</td>");
}

void Joint::ParseStaticsParameters(ifstream & ifs_links)
{
	string line;
	int found0,found1;
	string tmp;
	for(int i=0;i<3;i++){
		getline(ifs_links, line);
		staticXYZ_[i] = parseTableElement<double>(line, 0, "<td>", "</td>");
	}

	staticXYZ_ *=1.e-3;
}

void Joint::display () const
{
	std::cout << " Joint " << name_ << ", " << id_ << std::endl;
	std::cout << "   pos   " << positionMin_ << ", " << positionMax_ << std::endl;
	std::cout << "   speed " << speedMax_ << ", " << torqueMax_ << std::endl;
	std::cout << "   inner " << innerBodyId_ << ", " << innerBodyName_ << std::endl;
	std::cout << "   outer " << outerBodyId_ << ", " << outerBodyName_ << std::endl;
	std::cout << "   axis_ " << type_ << ", " << axis_ << ", " << staticXYZ_ << ", " << staticRPY_ << std::endl;
	std::cout << std::endl;
}

Camera::Camera()
: name_ ("")
, pos_(0,0,0)
{
}

void parseSensor(MultiBody* mb, ifstream & ifs_sensor)
{
	string line, tmp;
	int pos;
	while(getline(ifs_sensor, line))
	{
		if (line.find("Camera") != string::npos)
		{
			istringstream smallData(line, ios_base::in);
			Camera* cam = new Camera();
			smallData >> tmp >> cam->name_ >> tmp >> cam->pos_[0] >> tmp >> cam->pos_[1] >> tmp >> cam->pos_[2];
			mb->cameraMap_[cam->name_] = cam;
		}
	}
}

