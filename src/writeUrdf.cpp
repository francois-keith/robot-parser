#include "robot.h"
#include "tools.h"

#include <cstdlib>

using namespace Robot;
using namespace std;
using namespace afbase;

// tools method.

// write a Body into urdf format.
void writeUrdf(const Body*  b, std::ofstream & xml, const std::map<string,std::vector<string> > & bodyNameMap, const std::string & robotName);

// write a Joint into urdf format.
void writeUrdf(const MultiBody* mb, const Joint* j, std::ofstream & xml);

// write a CameraJoint into urdf format.
void writeUrdfCameraJoint (const MultiBody* mb, const Joint * j, ofstream & xml);

void writeUrdf(const MultiBody* mb, const Camera * j, std::ofstream & xml);

vector3d convert (const std::string & axis);

// get the urdf name for the joint name
std::string getUrdfName(const std::string & name, const std::map<std::string, std::string> bodyNameToUrdf);

std::string space (int num, const std::string & name)
{
	std::string res;
	for (int i=name.length(); i<num; ++i)
		res += " ";
	return res;
}
void writeUrdf(const MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName);

void writeSrdf (const MultiBody* mb, const std::string & xmlfolder,
		const std::string & fileName, const std::string & robotName,
		const std::map<unsigned, std::string> & limbMap,
		std::vector<double> & initialPosition);

void writeRos(const MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName)
{
	system( ("mkdir -p " + xmlfolder + "/srdf/").c_str() );
	system( ("mkdir -p " + xmlfolder + "/urdf/").c_str() );

	writeUrdf(mb, xmlfolder+"/urdf/", fileName, robotName);

	std::cout << "Write Srdf files" << std::endl;

	// load the initial config
	std::string inFile("../data/" + fileName + "-HalfSitting.dat");
	std::vector<double> initialPosition(mb->nbJoints()+6);
	DECLARE_IFSTREAM(in, inFile.c_str());
	if (in)
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


	// Split the limb by groups for Romeo
	std::map<unsigned, std::string> limbMap;

	if (mb->isRomeo_)
	{
		limbMap[0] =  "chest      ";
		limbMap[1] =  "head       ";
		limbMap[5] =  "left arm   ";
		limbMap[12] = "right arm  ";
		limbMap[19] = "left leg   ";
		if (true) //mb->useToes)
			limbMap[26] = "right leg  ";
		else
			limbMap[25] = "right leg  ";
	}

	writeSrdf (mb, xmlfolder+"/srdf/", fileName, robotName, limbMap, initialPosition);
}

// write URDF
void writeUrdf(const MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName)
{
	std::cout << "Write Urdf files" << std::endl;

	std::string xmlfile = xmlfolder + fileName+ ".urdf";
	std::map<std::string, std::vector<std::string> > & bodyNameMap = Body::bodyNameToVRML_;

	DECLARE_OFSTREAM(outXML, xmlfile.c_str())
	

	outXML << "<?xml version=\"1.0\"?>" << endl;
	outXML << "<!--" << endl;
	if (mb->isRomeo_)
		outXML << "   Aldebaran Robotics " + robotName + " URDF model" << endl;
	else
		outXML << "   " + robotName + " URDF model" << endl;
	outXML << endl;
	outXML << "   FIXME: fill missing data: sole, gripper and sensors" << endl;
	outXML << "  -->" << endl;
	outXML << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\" name=\""<<robotName<<"\">" << endl;

	// print all the links
	outXML << "  <link name=\"base_link\"/>" <<std::endl;
	outXML << std::endl;

	for (std::map<int,Body*>::const_iterator it = mb->bodyMap_.begin(); it != mb->bodyMap_.end(); ++it)
		writeUrdf(it->second, outXML, bodyNameMap, robotName);

	outXML << endl;
	outXML << endl;

	// print all the joints
	outXML << "  <!--   Joints following below -->"<<std::endl;
	outXML << endl;
	outXML << "  <joint name=\"waist\" type=\"fixed\">" <<std::endl;
	outXML << "    <parent link=\"base_link\"/>" <<std::endl;
	outXML << "    <child link=\"body\"/>" <<std::endl;
	outXML << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>" <<std::endl;
	outXML << "  </joint>" <<std::endl;

	for (std::map<int,Joint*>::const_iterator it = (mb->jointMap_).begin(); it != (mb->jointMap_).end(); ++it)
		writeUrdf(mb, it->second, outXML);

	// add sole joints
	outXML << "  <joint name=\"l_sole_joint\" type=\"fixed\">" <<std::endl;
	outXML << "    <parent link=\"l_ankle\"/>" <<std::endl;
	outXML << "    <child link=\"l_sole\"/>" <<std::endl;
	outXML << "    <origin xyz=\"0 0 " << - mb->ankle_Height_ << "\" rpy=\"0 0 0\"/>" <<std::endl;
	outXML << "  </joint>" <<std::endl;
	outXML << "  <joint name=\"r_sole_joint\" type=\"fixed\">" <<std::endl;
	outXML << "    <parent link=\"r_ankle\"/>" <<std::endl;
	outXML << "    <child link=\"r_sole\"/>" <<std::endl;
	outXML << "    <origin xyz=\"0 0 " << - mb->ankle_Height_ << "\" rpy=\"0 0 0\"/>" <<std::endl;
	outXML << "  </joint>" <<std::endl;

	// add camera
	if(! mb->cameraMap_.empty())
	{
		outXML << std::endl;
		outXML << "  <!-- Humanoid standard frame: gaze -->" << std::endl;
		outXML << "  <joint name=\"gaze_joint\" type=\"fixed\">" << std::endl;
		outXML << "    <parent link=\"HeadRoll_link\"/>" << std::endl;
		outXML << "    <child link=\"gaze\"/>" << std::endl;
		outXML << "    <origin xyz=\""<< mb->gazePosition_ <<"\" rpy=\"0 0 0\" />" << std::endl;
		outXML << "  </joint>" << std::endl;
		outXML << "  <link name=\"gaze\"/>" << std::endl;

		outXML << std::endl;

		outXML << "  <!-- Sensors: cameras -->" << std::endl;
		for (std::map<int,Joint*>::const_iterator it = (mb->cameraJointMap_).begin(); it != (mb->cameraJointMap_).end(); ++it)
			writeUrdfCameraJoint(mb, it->second, outXML);

		for (std::map<std::string,Camera*>::const_iterator it = (mb->cameraMap_).begin(); it != (mb->cameraMap_).end(); ++it)
			writeUrdf(mb, it->second, outXML);
	}

	// end of the urdf
	outXML << "</robot>" << endl;
	outXML.close();
}

void writeUrdf (const Body * body, ofstream & xml,
		const std::map<string,std::vector<string> > & bodyNameMap,
		const std::string & robotName)
{	
	assert(body != 0x0);
	std::string sp="  ";
	std::string urdfName = getUrdfName(body->name_, Body::bodyNameToURDF_);
	xml <<sp<< "<link name=\"" << urdfName << "\">" << endl;
	xml <<sp<<sp<< "<inertial>" << std::endl;
	xml <<sp<<sp<<sp<< "<origin xyz=\"" << body->com_ <<  "\" rpy=\"0 0 0\"/>" << std::endl;
	xml <<sp<<sp<<sp<< "<mass value=\"" << body->mass_ << "\"/>" << std::endl;
	xml <<sp<<sp<<sp<< "<inertia "
			<< "ixx=\"" << body->inertia_ (0,0) <<  "\" "
			<< "ixy=\"" << body->inertia_ (0,1) <<  "\" "
			<< "ixz=\"" << body->inertia_ (0,2) <<  "\" "
			<< "iyy=\"" << body->inertia_ (1,1) <<  "\" "
			<< "iyz=\"" << body->inertia_ (1,2) <<  "\" "
			<< "izz=\"" << body->inertia_ (2,2) <<  "\" "
			<< "/>" << std::endl;
	xml <<sp<<sp<< "</inertial>" << std::endl;

	std::map<string,std::vector<string> >::const_iterator it = bodyNameMap.find(body->name_);
	if ( it != bodyNameMap.end())
	{
		const std::vector<string> & vrmls = it->second;
		xml <<sp<<sp<< "<visual>" << std::endl;
		xml <<sp<<sp<<sp<< "<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"<< std::endl;
	    xml <<sp<<sp<<sp<< "<geometry>"<< std::endl;
	    if ( it != bodyNameMap.end())
	    {
	    	const std::vector<string> & vrmls = it->second;
	    	for(unsigned i = 0; i < vrmls.size(); ++i)
	    		xml <<"\t"<< "<mesh filename=\"package://"+ robotName +"_description/meshes/" <<  (vrmls[i]) << ".dae\" />"<< std::endl;
	    }
		xml <<sp<<sp<<sp<< "</geometry>"<< std::endl;
		xml <<sp<<sp<< "</visual>" << std::endl;
	    xml <<sp<<sp<< "<collision>"<< std::endl;
	    xml <<sp<<sp<<sp<< "<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"<< std::endl;
	    xml <<sp<<sp<<sp<< "<geometry>"<< std::endl;
	    if ( it != bodyNameMap.end())
	    {
	    	const std::vector<string> & vrmls = it->second;
	    	for(unsigned i = 0; i < vrmls.size(); ++i)
	    		xml <<"\t"<< "<mesh filename=\"package://"+ robotName +"_description/meshes/" <<  (vrmls[i]) << ".dae\" />"<< std::endl;
	    }

	    xml <<sp<<sp<<sp<< "</geometry>"<< std::endl;
		xml <<sp<<sp<< "</collision>"<< std::endl;
	}

//	xml <<sp<<sp<< "<!-- display -->" << std::endl;

//	std::map<string,std::vector<string> >::const_iterator it = bodyNameMap.find(body->name_);
//	if ( it != bodyNameMap.end())
//	{
//		const std::vector<string> & vrmls = it->second;
//		for(unsigned i = 0; i < vrmls.size(); ++i)
//			xml << "\t\t\t<File>afresources/romeo/vrml/" <<  (vrmls[i]) << ".wrl</File>" << std::endl;
//	}
	xml <<sp<< "</link>" << std::endl;
	if ( urdfName == "r_ankle" )
		xml <<sp<< "<link name=\"r_sole\"/>"<<std::endl;
	else if ( urdfName == "l_ankle" )
		xml <<sp<< "<link name=\"l_sole\"/>"<<std::endl;
	xml << endl;
}


void writeUrdf (const MultiBody* mb, const Joint * j, ofstream & xml)
{
	std::string sp="  ";
	xml <<sp<< "<joint name=\"" << j->name_ << "\" type=\"revolute\">" << std::endl;
	xml <<sp<<sp<< "<origin xyz=\"" << j->staticXYZ_ <<"\" rpy=\""<< j->staticRPY_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<axis xyz=\"" << j->axis_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<parent link=\"" <<
			getUrdfName(j->innerBodyName_, Body::bodyNameToURDF_)
			 << "\"/>" << std::endl;
	xml <<sp<<sp<< "<child link=\""  <<
			getUrdfName(j->outerBodyName_, Body::bodyNameToURDF_)
			 << "\"/>" << std::endl;
//	xml <<sp<<sp<< "<calibration rising=\"0.0\"/>" << std::endl;
//	xml <<sp<<sp<< "<dynamics damping=\"0.0\" friction=\"0.0\"/>" << std::endl;
	xml <<sp<<sp<< "<limit effort=\"30\""
			<<" lower=\""    << (j->positionMin_ *(3.1415926/ 180.)) <<"\""
			<<" upper=\""    << (j->positionMax_ *(3.1415926/ 180.)) <<"\""
			<<" velocity=\"" << j->speedMax_ <<"\" />" << std::endl;
	if (j->id_ >= 34 && j->id_ < 45)
		xml <<sp<<sp<< "<mimic joint=\"LFinger11\" multiplier=\"1\" offset=\"0\"/>" << std::endl;
	if (j->id_ >= 46)
		xml <<sp<<sp<< "<mimic joint=\"RFinger11\" multiplier=\"1\" offset=\"0\"/>" << std::endl;
	xml <<sp<< "</joint>" << std::endl;
	xml << endl;
}


void writeUrdfCameraJoint (const MultiBody* mb, const Joint * j, ofstream & xml)
{
	std::string sp="  ";
	xml <<sp<< "<joint name=\"" << j->name_ << "\" type=\"revolute\">" << std::endl;
	xml <<sp<<sp<< "<origin xyz=\"" << j->staticXYZ_ <<"\" rpy=\""<< j->staticRPY_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<axis xyz=\"" << j->axis_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<parent link=\"" << j->innerBodyName_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<child link=\""  << j->outerBodyName_ << "\"/>" << std::endl;
	xml <<sp<<sp<< "<limit effort=\"0\""
			<<" lower=\""    << (j->positionMin_ *(3.1415926/ 180.)) <<"\""
			<<" upper=\""    << (j->positionMax_ *(3.1415926/ 180.)) <<"\""
			<<" velocity=\"" << j->speedMax_ <<"\" />" << std::endl;
	xml <<sp<< "</joint>" << std::endl;
	xml <<sp<< "<link name=\"" <<  j->outerBodyName_ << "\"/>" << std::endl;
	xml << endl;
}

void writeUrdf(const MultiBody* mb, const Camera * cam, ofstream & xml)
{
	std::string parent = "gaze";
//	if (cam->name_ == "CameraRightEye")
//		parent = "CameraRightEye_joint";
//	if (cam->name_ == "CameraLeftEye")
//		parent = "CameraLeftEye_joint";

	std::string sp="  ";
	xml <<sp<< "<joint name=\""<<cam->name_<<"_joint\" type=\"fixed\">" << std::endl;
	xml <<sp<<sp<< "<origin"
			<<" xyz=\"" << (cam->pos_) << "\""
			<<" rpy=\"" << vector3d(0,0,0) << "\" />" << std::endl;
	xml <<sp<<sp<< "<parent link=\"" << parent <<"\"/>" << std::endl;
	xml <<sp<<sp<< "<child link=\"" << cam->name_ << "\"/>" << std::endl;
	xml <<sp<< "</joint>"<< std::endl;
	xml <<sp<< "<link name=\""<< cam->name_ <<"\"/>"<< std::endl;
	xml << std::endl;
}


void writeSrdf (const MultiBody* mb, const std::string & xmlfolder,
		const std::string & fileName, const std::string & robotName,
		const std::map<unsigned, std::string> & limbMap,
		std::vector<double> & initialPosition)
{
	std::string sp="  ";
	std::string xmlfile = xmlfolder + fileName+ ".srdf";
	std::map<std::string, std::vector<std::string> > & bodyNameMap = Body::bodyNameToVRML_;

	DECLARE_OFSTREAM(out, xmlfile.c_str())
	if (mb->isRomeo_)
		copyFile("../data/romeo/srdf_Top.txt", out);
	else
	{
		copyFile("../data/srdf_Top_part1.txt", out);
		out << "<robot name="+robotName+">" << std::endl;
		copyFile("../data/srdf_Top_part2.txt", out);
	}
	out << "  <!-- " + robotName + " pre-defined states -->"<< std::endl;
	out <<sp<< "<group_state name=\"half_sitting\" group=\"all\">"<< std::endl;
	out <<sp<<sp<< "<joint name=\"waist\" value=\"";
	for (unsigned i=0; i<6; ++i)
		out << initialPosition[mb->nbJoints()+i] << " ";
	out << "\" />"<< std::endl;
	for (unsigned i=0; i<mb->nbJoints(); ++i)
	{
		if( limbMap.find(i) != limbMap.end() )
			out << std::endl;

		std::map<int,Joint*>::const_iterator it = mb->jointMap_.find(i);
		if (it != mb->jointMap_.end() )
			out <<sp<<sp<< "<joint name=\""<< it->second->name_ <<"\"" << space(15,it->second->name_) << "value=\""<<initialPosition[i]<<"\" />"<< std::endl;
		else
			std::cerr << "WTH !" << std::endl;
	}

	out << std::endl;
	for (std::map<int, Joint*>::const_iterator it = mb->cameraJointMap_.begin(); it != mb->cameraJointMap_.end(); ++it)
	{
		out <<sp<<sp<< "<joint name=\""<< it->second->name_ <<"\"" << space(15,it->second->name_) << "value=\""<<0<<"\" />"<< std::endl;
	}

	out << sp << "</group_state>"<< std::endl;
	out << "</robot>" << std::endl;
	out.close();
}

std::string getUrdfName(const std::string & name, const std::map<std::string, std::string> bodyNameToUrdf)
{
	if ( bodyNameToUrdf.find(name) == bodyNameToUrdf.end() )
		return (name + "_link");
	else
		return bodyNameToUrdf.find(name)->second;
}
