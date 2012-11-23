#include "robot.h"
#include "writeOpenHRP.h"

#include "tools.h"
#include "matrix_operation.hxx"

#include <cstdlib>

using namespace Robot;
using namespace std;

// write OpenHRP
void writeOpenHRP (
		const MultiBody* mb,
		const std::string & openhrpFolder,
		const std::string & robotNameForHeader,
		const std::string & robotName, bool useToes)
{
	std::cout << "Write OpenHRP files" << std::endl;
	system( ("mkdir -p " + openhrpFolder).c_str());
	system( ("mkdir -p " + openhrpFolder + "/../include/" + robotNameForHeader).c_str());
	system( ("mkdir -p " + openhrpFolder + "/../python/").c_str());

	std::string name = openhrpFolder + robotName + ".wrl";
	DECLARE_OFSTREAM(out, name.c_str());

	// Write the beginning of the file.
	copyFile("../data/openhrp_header.wrl", out);
	
	std::string space ("\t");

	out <<"DEF "<<robotName<<" Humanoid {" << std::endl;
	out <<"\tname \""<< robotName << "\" " << std::endl;
	out <<"\tversion \""<< mb->version_ << "\"" << std::endl;
	out <<"\tinfo [" << std::endl;
	out <<"\t\t\"Date    : 2000.12.08\"" << std::endl;
	out <<"\t\t\"Version : 0.1\"" << std::endl;
	out	<<"	]" << std::endl;
	out << std::endl;
	out << space << "humanoidBody [" << std::endl;

	out << space << space << "DEF WAIST Joint {" << std::endl;
	out << space << space << space << "jointType \"free\"" << std::endl;
	out << space << space << space << "translation 0 0 0" << std::endl;
	out << space << space << space << "children [" << std::endl;

	std::string space2 = space + "\t\t";
	{
	std::map<int,Body*>::const_iterator it = mb->bodyMap_.find(0);
	writeOpenHRP(it->second, space2, out);
	}
	out << space << space << space << "]" << std::endl;
	out << space << space << "} # Joint WAIST " << std::endl;
	out << space << "]" << std::endl;

	out << std::endl;
	out << space << "# List up all the joints' name you use" << std::endl;
	out << space << "joints ["<< std::endl;
	for (std::map<int,Joint*>::const_iterator it2 = mb->jointMap_.begin(); it2 != mb->jointMap_.end(); ++it2)
	{	
		char axe;
		if (it2->second->axis_  == afbase::vector3d(1,0,0))
			axe = 'x';
		else if (it2->second->axis_  == afbase::vector3d(0,1,0))
			axe = 'y';
		else if (it2->second->axis_  == afbase::vector3d(0,0,1))
			axe = 'z';
		else
		{
			axe = 'y';
		}

		out << space << space << "USE " << it2->second->name_ << "_" << axe;
		if (it2 != mb->jointMap_.end())
			out << ",";
		out << std::endl;
	}
	out << space << "]" << std::endl;
	out << std::endl;
	out << space << "# List up all the segments' name you use" << std::endl;
	out << space << "  segments ["<< std::endl;
	for (std::map<int,Body*>::const_iterator it2 = mb->bodyMap_.begin(); it2 != mb->bodyMap_.end(); ++it2)
	{	
		out << space << space << "USE " << it2->second->name_;
		if (it2 != mb->bodyMap_.end())
			out << ",";
		out << std::endl;
	}
	out << space << "]" << std::endl;
	
	//eof
	out <<"}" << std::endl;
	out.close();

	writeOpenHRPLinkJointRank (mb, openhrpFolder, robotName);
	writeOpenHRPSpecificities (mb, openhrpFolder, robotName, useToes);

	//
	std::map<unsigned, std::string> limbMap;
	limbMap[0] =  "chest      ";
	limbMap[1] =  "head       ";
	limbMap[5] =  "left arm   ";
	limbMap[12] = "right arm  ";
	limbMap[19] = "left leg   ";
	if (useToes)
	{
		limbMap[26] = "right leg  ";
		limbMap[33] = "left hand   ";
		limbMap[45] = "right hand  ";
	}
	else
	{
		limbMap[25] = "right leg  ";
		limbMap[31] = "left hand  ";
		limbMap[43] = "right hand ";
	}

	// load the initial config
	std::string inFile("../data/"+robotNameForHeader+"-HalfSitting.dat");
	DECLARE_IFSTREAM(in, inFile.c_str());
	std::vector<double> initialPosition(mb->nbJoints()+6);
	if(in)
	{
		for(int i=0; i<mb->nbJoints()+6; ++i)
			in >> initialPosition[i];
		in.close();
	}
	else
	{
		std::cerr << "  *** Warning: the half sitting position is not defined *** " << std::endl;
		for(int i=0; i<mb->nbJoints()+6; ++i)
			initialPosition[i] = 0;
	}
	//end of loading

//	writeRomeoHeader(
//		openhrpFolder,
//		robotNameForHeader,
//		robotName,	mb,
//		limbMap, initialPosition
//	);


	writeInitConfig(
		openhrpFolder, robotName,
		mb, limbMap, initialPosition
		);

	writeRobotSpecificPython(
			openhrpFolder,
			robotNameForHeader,
			robotName,	mb,
			limbMap, initialPosition
		);
}

void writeOpenHRPLinkJointRank (
		const MultiBody *mb,
		const std::string & openhrpFolder, const std::string & robotName)
{
	std::string name = openhrpFolder + robotName + "LinkJointRank.xml";
	DECLARE_OFSTREAM(out2, name.c_str())
		
	out2 << "<!-- " << robotName <<" robot, version " << mb->version_ << "-->" << endl;
	out2 << "<LinkJointNameAndRank>" << std::endl;
    out2 << "\t<Nb> "<< (mb->nbJoints() + 1) << "</Nb>" << std::endl;
    out2 << "\t<Link> WAIST 0 </Link>" << std::endl;
	for (std::map<int,Joint*>::const_iterator it2 = mb->jointMap_.begin(); it2 != mb->jointMap_.end(); ++it2)
	{
		char axe;
		if (it2->second->axis_  == afbase::vector3d(1,0,0))
			axe = 'x';
		else if (it2->second->axis_  == afbase::vector3d(0,1,0))
			axe = 'y';
		else if (it2->second->axis_  == afbase::vector3d(0,0,1))
			axe = 'z';
		else
		{
			axe = 'y';
		}

	    out2 << "\t<Link> " << it2->second->name_ << "_" << axe <<"  " << (it2->second->id_ + 6) << " </Link>" << std::endl;
	}
	out2 << "</LinkJointNameAndRank>"<< std::endl;
	
	out2.close();
}

void writeOpenHRPSpecificities (
	const MultiBody* mb,
	const std::string & openhrpFolder,
	const std::string & robotName, bool useToes)
{
	std::string name = openhrpFolder + robotName + "Specificities.xml";
	DECLARE_OFSTREAM(out3, name.c_str())

	out3 << "<!-- " << robotName << " robot, version " << mb->version_ << "-->" << endl;
	out3 <<"<Humanoid name=\"" << robotName << "\">" << std::endl;

	if(useToes)
	{
		out3 <<"\t<Feet>" << std::endl;
		out3 <<"\t\t<Right>" << std::endl;
		writeFootPosition(out3, mb->ankle_rearBound_,
				mb->ankle_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);
		out3 <<"\t\t\t<JointNb> 2 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 30 31 </JointsID>" << std::endl;
		out3 <<"\t\t\t<!-- Toe 32 -->" << std::endl;
		out3 <<"\t\t</Right>" << std::endl;

		out3 <<"\t\t<Left>" << std::endl;
		writeFootPosition(out3, mb->ankle_rearBound_,
				mb->ankle_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);
		out3 <<"\t\t\t<JointNb> 2 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 23 24 </JointsID>" << std::endl;
		out3 <<"\t\t\t<!-- Toe 25 -->" << std::endl;
		out3 <<"\t\t</Left>" << std::endl;
		out3 <<"\t</Feet>" << std::endl;
		out3 << std::endl;
	}
	else
	{
		out3 <<"\t<Feet>" << std::endl;
		out3 <<"\t\t<Right>" << std::endl;
		writeFootPosition(out3, mb->ankle_rearBound_,
				mb->ankle_frontBound_+ mb->toe_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);
		out3 <<"\t\t\t<JointNb> 2 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 29 30 </JointsID>" << std::endl;
		out3 <<"\t\t</Right>" << std::endl;

		out3 <<"\t\t<Left>" << std::endl;
		writeFootPosition(out3, mb->ankle_rearBound_,
				mb->ankle_frontBound_+ mb->toe_frontBound_, mb->ankle_rightBound_, mb->ankle_leftBound_, mb->ankle_Height_);
		out3 <<"\t\t\t<JointNb> 2 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 23 24 </JointsID>" << std::endl;
		out3 <<"\t\t</Left>" << std::endl;
		out3 <<"\t</Feet>" << std::endl;
		out3 << std::endl;
	}

	out3 <<"\t<Waist>" << std::endl;
	out3 <<"\t\t<Right>" << std::endl;
	if(useToes)
		out3 <<"\t\t\t<WaistToHip>" << mb->jointMap_.find(26)->second->staticXYZ_ << "</WaistToHip>" << std::endl;
	else
		out3 <<"\t\t\t<WaistToHip>" << mb->jointMap_.find(25)->second->staticXYZ_ << "</WaistToHip>" << std::endl;
	out3 <<"\t\t</Right>" << std::endl;
	out3 <<"\t\t<Left>" << std::endl;
	out3 <<"\t\t\t<WaistToHip>" << mb->jointMap_.find(19)->second->staticXYZ_ << "</WaistToHip>" << std::endl;
	out3 <<"\t\t</Left>" << std::endl;
	out3 <<"\t\t<JointNb> 1 </JointNb>" << std::endl;
	out3 <<"\t\t<JointsID> -1 </JointsID>" << std::endl;
	out3 <<"\t</Waist>" << std::endl;
	out3 << std::endl;

	out3 <<"\t<Legs>" << std::endl;
	out3 <<"\t\t<Right>" << std::endl;
	out3 <<"\t\t\t<HipLength> 0  0  0 </HipLength>" << std::endl;
	if(useToes)
	{
		out3 <<"\t\t\t<FemurLength> " << abs( mb->jointMap_.find(29)->second->staticXYZ_[2]) << " </FemurLength>" << std::endl;
		out3 <<"\t\t\t<TibiaLength> " << abs( mb->jointMap_.find(30)->second->staticXYZ_[2]) << " </TibiaLength>" << std::endl;
		out3 <<"\t\t\t<JointNb> 4 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 26 27 28 29 </JointsID>" << std::endl;
	}
	else
	{
		out3 <<"\t\t\t<FemurLength> " << abs( mb->jointMap_.find(28)->second->staticXYZ_[2]) << " </FemurLength>" << std::endl;
		out3 <<"\t\t\t<TibiaLength> " << abs( mb->jointMap_.find(29)->second->staticXYZ_[2]) << " </TibiaLength>" << std::endl;
		out3 <<"\t\t\t<JointNb> 4 </JointNb>" << std::endl;
		out3 <<"\t\t\t<JointsID> 25 26 27 28 </JointsID>" << std::endl;
	}
	out3 <<"\t\t</Right>" << std::endl;
	out3 <<"\t\t<Left>" << std::endl;
	out3 <<"\t\t\t<HipLength> 0  0  0 </HipLength>" << std::endl;
	out3 <<"\t\t\t<FemurLength>" << abs( mb->jointMap_.find(22)->second->staticXYZ_[2]) << " </FemurLength>" << std::endl;
	out3 <<"\t\t\t<TibiaLength>" << abs( mb->jointMap_.find(23)->second->staticXYZ_[2]) << " </TibiaLength>" << std::endl;
	out3 <<"\t\t\t<JointNb> 4 </JointNb>" << std::endl;
	out3 <<"\t\t\t<JointsID> 19 20 21 22 </JointsID>" << std::endl;
	out3 <<"\t\t</Left>" << std::endl;
	out3 <<"\t</Legs>" << std::endl;
	out3 << std::endl;

	out3 <<"\t<Hands> <!-- unknown yet -->" << std::endl;
	out3 <<"\t\t<Right>" << std::endl;
	writeHandPosition(out3);
	out3 <<"\t\t</Right>" << std::endl;
	out3 <<"\t\t<Left>" << std::endl;
	writeHandPosition(out3);
	out3 <<"\t\t</Left>" << std::endl;
	out3 <<"\t</Hands>" << std::endl;
	out3 << std::endl;

	out3 <<"\t<Wrists>" << std::endl;
	out3 <<"\t\t<Right>" << std::endl;
	out3 <<"\t\t\t<JointNb> 1 </JointNb>" << std::endl;
	out3 <<"\t\t\t<JointsID> 17 </JointsID>" << std::endl;
	out3 <<"\t\t</Right>" << std::endl;
	out3 <<"\t\t<Left>" << std::endl;
	out3 <<"\t\t\t<JointNb> 1 </JointNb>" << std::endl;
	out3 <<"\t\t\t<JointsID> 10 </JointsID>" << std::endl;
	out3 <<"\t\t</Left>" << std::endl;
	out3 <<"\t</Wrists>" << std::endl;
	out3 << std::endl;

	out3 <<"\t<Arms>" << std::endl;
	out3 <<"\t\t<Right>" << std::endl;
	out3 <<"\t\t\t<UpperArmLength> " << abs( mb->jointMap_.find(14)->second->staticXYZ_[0]) << " </UpperArmLength>" << std::endl;
	out3 <<"\t\t\t<ForeArmLength>  " << abs( mb->jointMap_.find(16)->second->staticXYZ_[0]) << " </ForeArmLength>" << std::endl;
	out3 <<"\t\t\t<JointNb> 7 </JointNb>" << std::endl;
	out3 <<"\t\t\t<JointsID> 12 13 14 15 16 17 18 </JointsID>" << std::endl;
	out3 <<"\t\t</Right>" << std::endl;

	out3 <<"\t\t<Left>" << std::endl;
	out3 <<"\t\t\t<UpperArmLength> " << abs( mb->jointMap_.find(7)->second->staticXYZ_[0]) << " </UpperArmLength>" << std::endl;
	out3 <<"\t\t\t<ForeArmLength>  " << abs( mb->jointMap_.find(9)->second->staticXYZ_[0]) << " </ForeArmLength>" << std::endl;
	out3 <<"\t\t\t<JointNb> 7 </JointNb>" << std::endl;
	out3 <<"\t\t\t<JointsID> 5 6 7 8 9 10 11 </JointsID>" << std::endl;
	out3 <<"\t\t</Left>" << std::endl;
	out3 <<"\t</Arms>" << std::endl;

	out3 <<"\t<Head>" << std::endl;
	out3 <<"\t\t<JointNb> 4 </JointNb>" << std::endl;
	out3 <<"\t\t<JointsID> 1 2 3 4 </JointsID>" << std::endl;
	out3 <<"\t</Head>" << std::endl;

	out3 <<"\t<Chest>" << std::endl;
	out3 <<"\t\t<JointNb> 1 </JointNb>" << std::endl;
	out3 <<"\t\t<JointsID> 0 </JointsID>" << std::endl;
	out3 <<"\t</Chest>" << std::endl;
	out3 <<"</Humanoid>" << std::endl;
	out3.close();
}

void writeFootPosition(std::ofstream & out,
	double rear, double front, double left, double right, double height	)
{
	out <<"\t\t\t<SizeX> "<< (rear + front) <<" </SizeX>" << std::endl;
	out <<"\t\t\t<SizeY> "<< (right + left) <<" </SizeY>" << std::endl;
	out <<"\t\t\t<SizeZ> "<< height <<" </SizeZ>" << std::endl;
	out <<"\t\t\t<AnklePosition> 0.0 0.0 "<< height <<" </AnklePosition>" << std::endl;
}

void writeHandPosition(std::ofstream & out)
{
	out <<"\t\t\t<Center>0  0  0</Center>" << std::endl;
	out <<"\t\t\t<okayAxis>0.0 1.0 0.0 </okayAxis>" << std::endl;
	out <<"\t\t\t<showingAxis>0.0 0.0 1.0</showingAxis>" << std::endl;
	out <<"\t\t\t<palmAxis>1.0 0.0 0.0</palmAxis>" << std::endl;
}

void writeOpenHRP (const Body* b, const std::string & space, std::ofstream & out)
{
	out << space << "DEF " << b->name_ << " Segment {" <<std::endl;
	out << space << "\t" << "centerOfMass          " << b->com_ <<std::endl;
	out << space << "\t" << "mass                  " << b->mass_ <<std::endl;
	out << space << "\t" << "momentsOfInertia      [ ";
	for (unsigned i=0; i<3; ++i)
		for (unsigned j=0; j<3; ++j)
			out << b->inertia_(i,j) << " ";
	out << "]" <<std::endl;
	
	if (Body::bodyNameToVRML_.find(b->name_)!=Body::bodyNameToVRML_.end())
	{
		out << space << "\t" << "children ["<<std::endl;
		const std::vector<string> & vrmls = Body::bodyNameToVRML_[ b->name_ ];
		for(unsigned i = 0; i < vrmls.size(); ++i)
			out << space << "\t" << "\t" << "Inline { url \""<< vrmls[i] << ".wrl\" }"<<std::endl;
   		out << space << "\t" << "]"<<std::endl;
	}
	out << space << "} # Segment " << b->name_ <<std::endl;

	for (unsigned i=0; i<b->outerJointList_.size() ;++i)
		writeOpenHRP(b->outerJointList_[i], space, out);
}

void writeOpenHRP (const Joint* j, const std::string & space, std::ofstream & out)
{
	char axe;
	double pMin = j->positionMin_;
	double pMax = j->positionMax_;
	if (j->axis_  == afbase::vector3d(1,0,0))
		axe = 'x';
	else if (j->axis_  == afbase::vector3d(0,1,0))
		axe = 'y';
	else if (j->axis_  == afbase::vector3d(0,0,1))
		axe = 'z';
	else
	{
		axe = 'y';
		pMin = - j->positionMax_;
		pMax = - j->positionMin_;
	}

	afbase::matrix3d orient = computeRotationMatrixFromEuler(j->staticRPY_);
	afbase::vector3d orAxis;
	double orAngle;
	matrixToUTheta(orAxis, orAngle, orient);

	char axis = toupper(axe);
	out << space << "DEF " << j->name_ << "_" << axe << " Joint {" << std::endl;
	out << space << "\t" << "jointType \"rotate\"" << std::endl;
	out << space << "\t" << "jointAxis \""<< axis <<"\"" << std::endl;
	out << space << "\t" << "jointId "<< j->id_ <<std::endl;
	out << space << "\t" << "translation     "<< j->staticXYZ_ <<std::endl;
	out << space << "\t" << "rotation     "<< orAxis << "  " << orAngle <<std::endl;
	out << space << "\t" << "llimit ["<< (pMin *(3.1415926/ 180.))<< "]  # " << pMin  << std::endl;
	out << space << "\t" << "ulimit ["<< (pMax *(3.1415926/ 180.))<< "]  # " << pMax << std::endl;
	out << space << "\t" << "lvlimit ["<< (-j->speedMax_)<< "]" << std::endl;
	out << space << "\t" << "uvlimit ["<< (j->speedMax_)<< "]" << std::endl;
    out << space << "\t" << "children [" << std::endl;
     
	writeOpenHRP(j->outerBody_, space+"\t\t", out);

    out << space << "\t" << "]" << std::endl;
    out << space << "} # Joint " << j->name_ << std::endl;
}

// write romeo.h
void writeRomeoHeader(
	const std::string & openhrpFolder,
	const std::string & robotNameForHeader,
	const std::string & robotName,
	const MultiBody* mb,
	const std::map<unsigned, std::string> & limbMap,
	std::vector<double> & initialPosition
)
{
	std::string fileName = robotNameForHeader;
//	fileName.replace(fileName.begin(), fileName.end(), '_', '-');
	for (unsigned i=0; i<fileName.size(); ++i)
		if(fileName[i] == '_')
			fileName[i] ='-';

	std::string headerFile(openhrpFolder+"/../include/"+fileName+"/"+fileName+".h");

	DECLARE_OFSTREAM(out, headerFile.c_str());

	std::string macro = toupper(robotName) +"_H";

	out << "/*" << std::endl;
	out << " *  Copyright INRIA RHONES ALPES" << std::endl;
	out << " *" << std::endl;
	out << " *  Authors: Francois Keith" << std::endl;
	out << " *" << std::endl;
	out << " */" << std::endl << std::endl;

	out << "#ifndef " << macro << std::endl;
	out << "#define " << macro << std::endl;
	out << std::endl;

	out << "/**" << std::endl;
	out << "   \\brief HALFSITTINGPOSITION  \\" << std::endl;
	for (unsigned i=0; i<mb->nbJoints(); ++i)
	{
		std::map<unsigned, std::string>::const_iterator it = limbMap.find(i);
		if( it != limbMap.end() )
			out << std::endl << "   " << (it->second) << " : ";
		else
			out << ", ";
		double angle = initialPosition[i] * 180 / 3.14159267;
		angle = round(angle);
		out << angle;
	}
	out << std::endl << "*/" << std::endl;
	out << std::endl;

	out << "#define HALFSITTINGPOSITION_DEG_OPENHRP {	";
	for (unsigned i=0; i< mb->nbJoints(); ++i)
	{
		if( limbMap.find(i) != limbMap.end() )
			out << "\t\t\\" << std::endl << "   ";
		else
			out << ", ";
		double angle = initialPosition[i] * 180 / 3.14159267;
		angle = round(angle);
		out << angle;
	}
	out << std::endl << std::endl;


	out << "enum " << robotName << "JointId {" << std::endl;
	out << "  WAIST = 0," << std::endl;
	out << "  WAIST_TY," << std::endl;
	out << "  WAIST_TZ," << std::endl;
	out << "  WAIST_RX," << std::endl;
	out << "  WAIST_RY," << std::endl;
	out << "  WAIST_RZ," << std::endl;

	int numDofs=6;
	for (std::map<int,Joint*>::const_iterator it2 = mb->jointMap_.begin(); it2 != mb->jointMap_.end(); ++it2)
	{
		char axe;
		if (it2->second->axis_  == afbase::vector3d(1,0,0))
			axe = 'x';
		else if (it2->second->axis_  == afbase::vector3d(0,1,0))
			axe = 'y';
		else if (it2->second->axis_  == afbase::vector3d(0,0,1))
			axe = 'z';
		else
		{
			axe = 'y';
		}

		out << "  " << it2->second->name_ << "_" << axe;
		if (it2 != mb->jointMap_.end())
			out << ",\t\t";
		else
			out << " \t\t";
		out << "// " << numDofs << std::endl;
		++ numDofs;
	}
	out << "};" << std::endl << std::endl;
	out << "#endif // " << macro << std::endl;
}

void writeInitConfig(
		const std::string & openhrpFolder,
		const std::string & robotName,
		const MultiBody* mb,
		const std::map<unsigned, std::string> & limbMap,
		std::vector<double> & initialPosition
	)
{
	std::string file(openhrpFolder+robotName+"InitConfig.dat");
	DECLARE_OFSTREAM(out, file.c_str());

	for (unsigned i=0; i< mb->nbJoints(); ++i)
	{
		if( (i!=0) && (limbMap.find(i) != limbMap.end() ))
			out << std::endl;
		double angle = initialPosition[i] * 180 / 3.14159267;
		angle = round(angle);
		out << angle << "  ";
	}
	out << std::endl;
	out.close();
}

void writeRobotSpecificPython(
	const std::string & openhrpFolder,
	const std::string & robotNameForHeader,
	const std::string & robotName,
	const MultiBody* mb,
	const std::map<unsigned, std::string> & limbMap,
	const std::vector<double> & initialPosition
)
{
	std::string file(openhrpFolder+"../python/robotSpecific_" +robotNameForHeader+".py.cmake");
	DECLARE_OFSTREAM(out, file.c_str());
	out << "# --- ROBOT CONFIG -------------------------------------------------------------" << std::endl;
	out << "from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,specificitiesName,jointRankName" << std::endl;
	out << std::endl;
	out << "# " << robotName << " data" << std::endl;
	out << "pkgDataRootDir['"<<robotNameForHeader<<"'] = \"${CMAKE_INSTALL_PREFIX}/share/" << (robotNameForHeader.substr(0,5)) << "\""  << std::endl;
	out << "modelName['"<<robotNameForHeader<<"'] = '"<<robotName<<".wrl'" << std::endl;
	out << "specificitiesName['" <<robotNameForHeader<< "'] = '" <<robotName<< "Specificities.xml'" << std::endl;
	out << "jointRankName['"     <<robotNameForHeader<< "'] = '" <<robotName<< "LinkJointRank.xml'" << std::endl;
	out << "robotDimension['"    <<robotNameForHeader<< "'] = "  << (mb->nbJoints()+6) <<std::endl;
	out << "initialConfig['"     <<robotNameForHeader<< "'] = (" << std::endl;

	// the free floatting part
	for (unsigned i=0; i<6; ++i)
		out << initialPosition[mb->nbJoints()+i] << ", ";
	out << "\t\t # FF " << std::endl;

	// the joint part
	for (unsigned i=0; i<mb->nbJoints(); ++i)
	{
		std::map<unsigned, std::string>::const_iterator it = limbMap.find(i);
		if( it != limbMap.begin() && it != limbMap.end() )
			out << "\t\t # " << ((--it)->second) << std::endl;
		out << initialPosition[i] << ", ";
	}
	std::map<unsigned, std::string>::const_iterator it2 = limbMap.end();
	-- it2;
	out << ")\t\t # " << ((it2)->second) << std::endl << std::endl;

	// inertia rotor (for now, the null vector)
	out << "inertiaRotor['"<<robotNameForHeader<<"'] = (";
	for(unsigned i=0; i<mb->nbJoints()+6; ++i)
	{
		out << 0;
		if (i < mb->nbJoints()+5)	out << ",";
	}
	out << ")" << std::endl;

	// gear rotor (for now, the null vector)
	out << "gearRatio['"<<robotNameForHeader<<"'] = (";
	for(unsigned i=0; i<mb->nbJoints()+6; ++i)
	{
		out << 0;
		if (i < mb->nbJoints()+5)	out << ",";
	}
	out << ")" << std::endl;
}
