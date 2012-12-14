#include "writeMaple.h"

#include "tools.h"

#include <fstream>
#include <sstream>

using namespace Robot;
using namespace std;

//declaration of useful methods to build Additional data file.
void createAdditionnalData (
	const MultiBody * mb,
	const std::string & addDataFile,
	const std::string & robotName,
	const std::string & version,
	int numContactBodies,
	bool useToes
);

void createPrefixForAdditionnalData (ofstream & outAddMaple,
	int numContactSolids,
	const std::string & robotName,
	const std::string & version
);

void createFootTagForAdditionnalData(ofstream & outAddMaple,
	int startingIndex, const std::string & lbName,
	double rear, double front, double left, double right, double height
);

void createDisplayTagForAdditionnalData(ofstream & outAddMaple,
		const std::vector<std::string> & nameList, int startingIndexAddData);

//declaration of a useful method to build lagrangian data file.
void writeLagrangianFile(const std::string & lagModelFile,
		int numJoints,
		int numContactBodies);


//writeMaple
void writeMaple(
	const Robot::MultiBody* mb,
	const std::string & kinematicfile,
	const std::string & dynamicfile,
	const std::string & limitsfile,
	const std::string & addDataFile,
	const std::string & lagModelFile,
	const std::string & robotName,
	bool useToes
	)
{
	// number of bodies in contact. The number of contact points is this number * 4
	int numContactBodies = 	(useToes)? 4 : 2;

	// -- Write The dynamic part of the robot description.
	std::cout << "Write Maple files" << std::endl;
	DECLARE_OFSTREAM(outDynamicMaple,dynamicfile.c_str())

	// The basic information corresponding to the universe.
	outDynamicMaple << "# "<< robotName << " robot, version " << mb->version_ << endl;
	outDynamicMaple << endl;
	outDynamicMaple << "#gravity vector" << endl;
	outDynamicMaple << "Gravity := vector([0, -9.81, 0]):" << endl;
	outDynamicMaple << endl;
	outDynamicMaple << "# Frame 1 : (none)" << endl;
	outDynamicMaple << "m_1  := 0:" << endl;
	outDynamicMaple << "G_1  := vector([0, 0, 0]*1e-3):" << endl;
	outDynamicMaple << "IG_1 := matrix([[0, 0, 0],[0, 0, 0],[0, 0, 0]]*1e-2):" << endl;

	// Then Add the informations of each body
	for (std::map<int,Body*>::const_iterator it = mb->bodyMap_.begin(); it != mb->bodyMap_.end(); ++it)
		writeMaple(it->second, outDynamicMaple);

	outDynamicMaple.close();
	// -- end of the dynamic part

	// Write the kinematic part of the robot description.
	DECLARE_OFSTREAM(outKinematicMaple, kinematicfile.c_str())

	stringstream data_kinematic(stringstream::in | stringstream::out);
	for (std::map<int,Joint*>::const_iterator it = mb->jointMap_.begin(); it != mb->jointMap_.end(); ++it)
		writeMaple(it->second, data_kinematic);

	outKinematicMaple << "# "<< robotName << " robot, version " << mb->version_ << endl;
	outKinematicMaple << endl;
	outKinematicMaple << "#number of frames" << endl;
	outKinematicMaple << "NSOL := " << (mb->nbBodies()+1) << ":" << endl;
	outKinematicMaple << "#number of DOFs" << endl;
	outKinematicMaple << "NDDL := " << (mb->nbJoints()+6) << ":" << endl;

	outKinematicMaple << endl;
	outKinematicMaple << "# Definitions of coordinates and generalized speeds" << endl;
	outKinematicMaple << "q := vector(NDDL):" << endl;
	outKinematicMaple << "qdot := vector(NDDL):" << endl;
	outKinematicMaple << endl;
	outKinematicMaple << "#Ground" << endl;
	outKinematicMaple << "ref_1 := 0:" << endl;
	outKinematicMaple << "Rx_1 := -Pi/2:" << endl;
	outKinematicMaple << "Ry_1 := 0:" << endl;
	outKinematicMaple << "Rz_1 := 0:" << endl;
	outKinematicMaple << "Tx_1 := 0:" << endl;
	outKinematicMaple << "Ty_1 := 0:" << endl;
	outKinematicMaple << "Tz_1 := 0:" << endl;
	outKinematicMaple << endl;
	outKinematicMaple << "#Trunk - Primary body" << endl;
	outKinematicMaple << "ref_2\t:= 1:" << endl;
	outKinematicMaple << "Rx_2\t:= q["  << mb->nbJoints()+3+1 << "]:" << endl;
	outKinematicMaple << "Ry_2\t:= -q[" << mb->nbJoints()+5+1 << "]:" << endl;
	outKinematicMaple << "Rz_2\t:= q["  << mb->nbJoints()+4+1 << "]:" << endl;
	outKinematicMaple << "Tx_2\t:= q["  << mb->nbJoints()+0+1 << "]:" << endl;
	outKinematicMaple << "Ty_2\t:= -q[" << mb->nbJoints()+2+1 << "]:" << endl;
	outKinematicMaple << "Tz_2\t:= q["  << mb->nbJoints()+1+1 << "]:" << endl;
	outKinematicMaple << data_kinematic.str();

	outKinematicMaple.close();
	// -- end of the kinematic part

	// finally, write the joints limitations.
	DECLARE_OFSTREAM(outLimitsMaple, limitsfile.c_str());
	copyFile("../data/SomeDefinitions_Top.txt", outLimitsMaple);

	outLimitsMaple << "#define NDOF "<< (mb->nbJoints()+6) <<std::endl;
	outLimitsMaple << "#define NCONT " << (numContactBodies*2) << std::endl;
	outLimitsMaple << "#define deg2rad M_PI/180.0" <<std::endl;
	outLimitsMaple << std::endl << std::endl;
	outLimitsMaple << "//Definition of "<< robotName << " joints limits" <<std::endl;
	outLimitsMaple << std::endl;

	for (std::map<int,Joint*>::const_iterator it = mb->jointMap_.begin(); it != mb->jointMap_.end(); ++it)
	{
		double pMin = it->second->positionMin_;
		double pMax = it->second->positionMax_;

		if (it->second->axis_ == afbase::vector3d(1,0,0)){}
		else if (it->second->axis_ == afbase::vector3d(0,1,0)){}
		else if (it->second->axis_ == afbase::vector3d(0,0,1)){}
		else
		{
			pMin = - it->second->positionMax_;
			pMax = - it->second->positionMin_;
		}

		outLimitsMaple << "#define " << it->second->name_ << "qMin ("<< pMin <<"*deg2rad)" <<std::endl;
		outLimitsMaple << "#define " << it->second->name_ << "qMax ("<< pMax <<"*deg2rad)" <<std::endl;
	}

	outLimitsMaple << std::endl;
	outLimitsMaple << "/**" <<std::endl;
	outLimitsMaple << " * Global variable corresponding to the max joints limit" <<std::endl;
	outLimitsMaple << " *" <<std::endl;
	outLimitsMaple << " * \\var MAXQ (double) <em> dim NDOF </em>" <<std::endl;
	outLimitsMaple << " */" <<std::endl;
	outLimitsMaple << "double MAXQ[NDOF] = {" <<std::endl << "\t";
	for (std::map<int,Joint*>::const_iterator it = mb->jointMap_.begin(); it != mb->jointMap_.end(); ++it)
	{
		outLimitsMaple << it->second->name_ << "qMax, ";
		if ( it->first == 0 ||  it->first == 4 ||  it->first == 11
			||  it->first == 18 ||  it->first == 25 ||  it->first == 32)
			outLimitsMaple << std::endl << "\t";
	}
	outLimitsMaple << "1e10, 1e10, 1e10, 1e10, 1e10, 1e10};" << std::endl << std::endl;


	outLimitsMaple << "/**" << std::endl;
	outLimitsMaple << " * Global variable corresponding to the min joints limit" << std::endl;
	outLimitsMaple << " *" << std::endl;
	outLimitsMaple << " * \\var MINQ (double) <em> dim NDOF </em>" << std::endl;
	outLimitsMaple << " */" << std::endl;
	outLimitsMaple << "double MINQ[NDOF] = {" << std::endl << "\t";
	for (std::map<int,Joint*>::const_iterator it = mb->jointMap_.begin(); it != mb->jointMap_.end(); ++it)
	{
		outLimitsMaple << it->second->name_ << "qMin, ";
		if ( it->first == 0 ||  it->first == 4 ||  it->first == 11
			||  it->first == 18 ||  it->first == 25 ||  it->first == 32)
			outLimitsMaple << std::endl << "\t";
	}
	outLimitsMaple << "-1e10, -1e10, -1e10, -1e10, -1e10, -1e10};" << std::endl << std::endl;

	copyFile("../data/SomeDefinitions_Bottom.txt", outLimitsMaple);
	// end of joints limitations.

	createAdditionnalData (
		mb, addDataFile,
		robotName, mb->version_,
		numContactBodies, useToes
	);

	writeLagrangianFile(lagModelFile,
		mb->nbJoints(),	numContactBodies);
}

// also, write the additional data
void createAdditionnalData (
		const MultiBody * mb,
		const std::string & addDataFile,
		const std::string & robotName,
		const std::string & version,
		int numContactBodies,
		bool useToes
	)
{
	DECLARE_OFSTREAM(outAddMaple, addDataFile.c_str());

	copyFile("../data/AdditionnalData_Top.maple", outAddMaple);

	// create the upper part of the file
	createPrefixForAdditionnalData (outAddMaple, numContactBodies, robotName, version);

	if(useToes)
	{
		// create the tags
		createFootTagForAdditionnalData(outAddMaple, 1, "LAnkleRoll",
			mb->ankle_rearBound_, mb->ankle_frontBound_, mb->ankle_leftBound_, mb->ankle_rightBound_, mb->ankle_Height_);
		createFootTagForAdditionnalData(outAddMaple, 5, "RAnkleRoll",
			mb->ankle_rearBound_, mb->ankle_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);

		createFootTagForAdditionnalData(outAddMaple, 9, "LToePitch",
			mb->toe_rearBound_, mb->toe_frontBound_, mb->toe_leftBound_, mb->toe_rightBound_, mb->toe_Height_);
		createFootTagForAdditionnalData(outAddMaple, 13, "RToePitch",
			mb->toe_rearBound_, mb->toe_frontBound_, mb->toe_rightBound_, mb->toe_leftBound_, mb->toe_Height_);
	}
	else
	{
		// create the tags
		createFootTagForAdditionnalData(outAddMaple, 1, "LAnkleRollBasic",
			mb->ankle_rearBound_, mb->ankle_frontBound_+mb->toe_frontBound_, mb->ankle_leftBound_, mb->ankle_rightBound_, mb->ankle_Height_);
		createFootTagForAdditionnalData(outAddMaple, 5, "RAnkleRollBasic",
			mb->ankle_rearBound_, mb->ankle_frontBound_+mb->toe_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);

		// even though they are not used, we add the tags of the toes, just to be keep the same number of tags
		outAddMaple << "# The following tags (9..16) should not be used, but are kept so that the number of tags does not change" << std::endl;
		createFootTagForAdditionnalData(outAddMaple, 9, "", 0, 0, 0, 0, 0);
		createFootTagForAdditionnalData(outAddMaple, 13, "", 0, 0, 0, 0, 0);
	}

	// add the tags for the end effectors
	int tagIndex = 17;
	outAddMaple << "### End effectors" << std::endl << std::endl;

	std::vector<std::string> bodyList;
	bodyList.push_back("LWristPitch");
	bodyList.push_back("RWristPitch");
	if (useToes)
	{
		bodyList.push_back("LAnkleRoll");
		bodyList.push_back("RAnkleRoll");
	}
	else
	{
		bodyList.push_back("LAnkleRollBasic");
		bodyList.push_back("RAnkleRollBasic");
	}
	createDisplayTagForAdditionnalData(outAddMaple, bodyList, tagIndex);
	tagIndex += bodyList.size();
	bodyList.clear();

	// add the tags for the display
	outAddMaple << "### Other bodies (for the display)" << std::endl << std::endl;
	bodyList.push_back("LKneePitch");
	bodyList.push_back("RKneePitch");
	bodyList.push_back("LHipPitch");
	bodyList.push_back("RHipPitch");
	bodyList.push_back("TrunkYaw");
	bodyList.push_back("NeckYaw");
	bodyList.push_back("LWristPitch");
	bodyList.push_back("RWristPitch");
	bodyList.push_back("LElbowRoll");
	bodyList.push_back("RElbowRoll");
	bodyList.push_back("LShoulderYaw");
	bodyList.push_back("RShoulderYaw");
	createDisplayTagForAdditionnalData(outAddMaple, bodyList, tagIndex);
	tagIndex += bodyList.size();
	bodyList.clear();

	// Add 6 additional tags for the display of the head
	copyFile("../data/AdditionnalData_Bottom.maple", outAddMaple);
	tagIndex += 6;

	// add the tags for the toes
	if (useToes)
	{
		bodyList.push_back("LToePitch");
		bodyList.push_back("RToePitch");
	}
	else
	{
		// again, a dirty patch to keep the same number of tags
		bodyList.push_back("");
		bodyList.push_back("");
	}

	createDisplayTagForAdditionnalData(outAddMaple, bodyList, tagIndex);
	tagIndex += bodyList.size();
	bodyList.clear();

	outAddMaple.close();
	// end of joints limitations.
}


	// finally, write the LagrangianModel data
void writeLagrangianFile(const std::string & lagModelFile,
		int numJoints,
		int numContactBodies)
{
	DECLARE_OFSTREAM(outLMMaple, lagModelFile.c_str());

	copyFile("../data/LagrangianModel_Top.txt", outLMMaple);
	outLMMaple <<"#define NDOF " << (numJoints+6) << std::endl;
	outLMMaple <<"#define NTAGS (40+1)" << std::endl;
	outLMMaple <<"#define NCONT " << (numContactBodies*4) << std::endl;

	copyFile("../data/LagrangianModel_Bottom.txt", outLMMaple);
	outLMMaple.close();
	// end of joints limitations.
}

void writeMaple (const Body* b, ofstream & outDynamicMaple)
{
	outDynamicMaple << endl << "# Frame " << b->name_ << endl;
	outDynamicMaple << "m_" << (b->id_+2) << " := " << b->mass_ << ":" << endl;
	outDynamicMaple << "G_" << (b->id_+2)<< " := " << "vector([" << b->com_(0) << " , " << b->com_(1) << " , " << b->com_(2) << "]):" << endl;
	outDynamicMaple << "IG_" << (b->id_+2) << " := " << "matrix([";

	for (int i=0; i<3; ++i)
	{
		outDynamicMaple << "[";
		for (int j=0; j<3; ++j)
		{
			outDynamicMaple << b->inertia_ (i,j);
			if (j!=2){
				outDynamicMaple << " , ";
			}
		}
		outDynamicMaple << "]";
		if (i!=2){
			outDynamicMaple << " , ";
		}
	}
	outDynamicMaple << "]):" << endl;
}





void writeMaple (const Joint* j, stringstream & data_kinematic)
{
	char axe;
	if (j->axis_ == afbase::vector3d(1,0,0))
		axe = 'x';
	else if (j->axis_ == afbase::vector3d(0,1,0))
		axe = 'y';
	else if (j->axis_ == afbase::vector3d(0,0,1))
		axe = 'z';
	else
	{
		axe = 'y';
	}

	assert(j->staticRPY_[0] == 0 && j->staticRPY_[1] == 0 && j->staticRPY_[2] == 0
		   && "The humans generator does not allow static rotations");
	data_kinematic << endl << "# Frame " << (j->outerBodyId_ +2) << " : " << j->name_ << endl;
	data_kinematic << "ref_" << (j->outerBodyId_ +2) << "\t:= " << (j->innerBodyId_+2) << ":" << endl;
	switch(axe)
	{
	case 'x':
		data_kinematic << "Rx_" << (j->outerBodyId_ +2) << "\t:= q[" << (j->id_+1) << "]:" << endl;
		data_kinematic << "Ry_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		data_kinematic << "Rz_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		break;
	case 'y':
		data_kinematic << "Rx_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		data_kinematic << "Ry_" << (j->outerBodyId_ +2) << "\t:= q[" << (j->id_+1) << "]:" << endl;
		data_kinematic << "Rz_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		break;
	case 'z':
		data_kinematic << "Rx_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		data_kinematic << "Ry_" << (j->outerBodyId_ +2) << "\t:= 0:" << endl;
		data_kinematic << "Rz_" << (j->outerBodyId_ +2) << "\t:= q[" << (j->id_+1) << "]:" << endl;
		break;
	}
	data_kinematic << "Tx_" << (j->outerBodyId_ +2) << "\t:= " << j->staticXYZ_[0] << ":" << endl;
	data_kinematic << "Ty_" << (j->outerBodyId_ +2) << "\t:= " << j->staticXYZ_[1] << ":" << endl;
	data_kinematic << "Tz_" << (j->outerBodyId_ +2) << "\t:= " << j->staticXYZ_[2] << ":" << endl;
}



//definition useful method to build Additional data file.
void createPrefixForAdditionnalData (ofstream & outAddMaple,
	int numContactSolids,
	const std::string & robotName,
	const std::string & version
)
{
	outAddMaple << "# "<< robotName << " robot, version " << version << endl;
	outAddMaple << "#" << std::endl << std::endl;
	outAddMaple << "# Definition of important tags" << std::endl << std::endl;
	outAddMaple << "# Contact tags" << std::endl;
	outAddMaple << "points_contact := [";
	for (unsigned i=1; i<= (numContactSolids*4); ++i)
	{
		outAddMaple << i;
		if (i == (numContactSolids*4))
			outAddMaple << "]:" << std::endl << std::endl;
		else
			outAddMaple << ", ";
	}

	outAddMaple << "# Contact solids definition (gather the contact points by solid)" << std::endl;
	outAddMaple << "# e.g. the contact points 1..4 correspond to the same solid (left foot)" << std::endl;
	outAddMaple << "ContactSolids := matrix([";
	for (unsigned i=1; i<= numContactSolids; ++i)
	{
		int lowIndex= 4*(i-1)+1;
		int topIndex=4*i;
		outAddMaple << "[" << lowIndex << ", " << topIndex << "]";
		if (i < numContactSolids)
			outAddMaple << ", ";
	}
	outAddMaple << "]):" << std::endl << std::endl;

	outAddMaple << "# Number of contact solids" << std::endl;
	outAddMaple << "NCONTSOL := "<< (numContactSolids) <<":" << std::endl << std::endl;

	outAddMaple << "# Number of contacts" << std::endl;
	outAddMaple << "NCONT := "<< (numContactSolids*4) <<":"<<std::endl << std::endl;

	outAddMaple << "#Definition vecteur Lambda:" << std::endl;
	outAddMaple << "Lambda := vector(3*NCONT):" << std::endl << std::endl;

	outAddMaple << "# Number of tags" << std::endl;
	outAddMaple << "NTAG := 40:" << std::endl;
}

void createFootTagForAdditionnalData(ofstream & outAddMaple, int startingIndex,
	const std::string & lbName,
	double rear, double front, double left, double right, double height
)
{
	//get the ankles and toes index
	int indexLeftAnkle =0;
	std::map<std::string,int>::const_iterator it_la = Body::bodyNameToId_.find(lbName);
	if (it_la != Body::bodyNameToId_.end())
		indexLeftAnkle = it_la->second +2;
	else if (lbName == "")
		indexLeftAnkle = 0;
	else
		std::cerr << " Issue in createTagForAdditionnalData: unable to find body "<<lbName<<"" << std::endl;

	int i = startingIndex;
	outAddMaple << "# Upper left bound of the bounding box wrapping the toe  (y value outside the BB)"<<std::endl;
	outAddMaple << "# Tag "<<i<<" : "<<lbName<<" Contact Point" << std::endl;
	outAddMaple << "reftag_"<<i<<" := "<< indexLeftAnkle<< ":" << std::endl;
	outAddMaple << "tag_"<<i<<"    := vector(["<< front <<", "<<left<<", "<<(-height)<<"]):" << std::endl;
	outAddMaple << std::endl;
	++i;

	outAddMaple << "# Upper right bound of the bounding box wrapping the toe (y value outside the BB)" <<std::endl;
	outAddMaple << "# Tag "<<i<<" : "<<lbName<<" Contact Point" << std::endl;
	outAddMaple << "reftag_"<<i<<" := "<< indexLeftAnkle<< ":" << std::endl;
	outAddMaple << "tag_"<<i<<"    := vector(["<< front <<", "<<(-right)<<", "<<(-height)<<"]):" << std::endl;
	outAddMaple << std::endl;
	++i;

	outAddMaple << "# Lower left bound of the bounding box wrapping the toe  (y value outside the BB)" << std::endl;
	outAddMaple << "# Tag "<<i<<" : "<<lbName<<" Contact Point" << std::endl;
	outAddMaple << "reftag_"<<i<<" := "<< indexLeftAnkle<< ":" << std::endl;
	outAddMaple << "tag_"<<i<<"    := vector(["<< (-rear) <<", "<<left<<", "<<(-height)<<"]):" << std::endl;
	outAddMaple << std::endl;
	++i;

	outAddMaple << "# Lower right bound of the bounding box wrapping the toe (y value outside the BB)" << std::endl;
	outAddMaple << "# Tag "<<i<<" : "<<lbName<<" Contact Point" << std::endl;
	outAddMaple << "reftag_"<<i<<" := "<< indexLeftAnkle<< ":" << std::endl;
	outAddMaple << "tag_"<<i<<"    := vector(["<< (-rear) <<", "<<(-right)<<", "<<(-height)<<"]):" << std::endl;
	++i;
	outAddMaple << std::endl;
	outAddMaple << std::endl;
}

void createDisplayTagForAdditionnalData(ofstream & outAddMaple,
		const std::vector<std::string> & nameList, int startingIndexAddData)
{
	for(int i=0;i<nameList.size();++i)
	{
		std::map<std::string, int>::const_iterator it = Body::bodyNameToId_.find(nameList[i]);
		int index=0;
		if(it!= Body::bodyNameToId_.end())
			index = it->second + 2;
		else if (! (nameList[i] == "" ))
			std::cerr << " The body " << nameList[i] << " is unknown " << std::endl;

		outAddMaple << "# Tag "<< startingIndexAddData <<" : " << nameList[i] << std::endl;
		outAddMaple << "reftag_"<< startingIndexAddData <<" := "<< index <<":"<< std::endl;
		outAddMaple << "tag_"<< startingIndexAddData <<"    := vector([0, 0, 0]):"<< std::endl << std::endl;
		++startingIndexAddData;
	}
}
