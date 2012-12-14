#include "writeAmelif.h"
#include "tools.h"

#include <cstdlib>

using namespace Robot;
using namespace std;

// write Amelif
void writeAmelif(const MultiBody* mb, const std::string & xmlfolder, const std::string & fileName, const std::string & robotName)
{
	system( ("mkdir -p " + xmlfolder + "xml").c_str() );

	std::cout << "Write Amelif files" << std::endl;
	std::map<string,std::vector<string> > emptyMap;

	// the skeleton model (no vrml)
	writeXmlFile(mb, xmlfolder + "xml/" + fileName+ "-skeleton.xml", robotName, emptyMap);
	// the real model (with real feet)
	writeXmlFile(mb, xmlfolder + "xml/" + fileName+ ".xml"         , robotName, Body::bodyNameToVRML_);
	 // the simplified model (with bv instead of feet)
	writeXmlFile(mb, xmlfolder + "xml/" + fileName+ "-bv.xml"      , robotName, Body::bodyNameToVRML_BV_);

	if (mb->isRomeo_)
	{
		system( ("mkdir -p " + xmlfolder + "/vrml/").c_str() );
		system( ("mkdir -p " + xmlfolder + "/control/").c_str() );
		createBoundingVolumeFiles(xmlfolder + "/vrml/", mb);
		copyFile("../data/romeo-HalfSitting.dat",        xmlfolder + "control/romeo-HalfSitting.dat");
		copyFile("../data/romeo_notoes-HalfSitting.dat", xmlfolder + "control/romeo_notoes-HalfSitting.dat");
	}
}

void writeXmlFile(const MultiBody* mb, const std::string & xmlfile, const std::string & robotName,
	const std::map<string,std::vector<string> > & bodyNameMap)
{
	DECLARE_OFSTREAM(outXML, xmlfile.c_str())
	
	outXML << "<!-- " << robotName << " robot, version " << mb->version_ << "-->" << endl;
	outXML << "<MultiBody>" << endl;
	outXML << "\t<Root id=\"0\" />" << endl;

	outXML << "\t<Bodies>" << endl;		
	for (std::map<int,Body*>::const_iterator it = mb->bodyMap_.begin(); it != mb->bodyMap_.end(); ++it)
		writeAmelif(it->second, outXML, bodyNameMap, "afresources/"+robotName+"/");
	outXML << "\t</Bodies>" << endl;

	outXML << "\t<Joints>" << endl;
	for (std::map<int,Joint*>::const_iterator it = (mb->jointMap_).begin(); it != (mb->jointMap_).end(); ++it)
		writeAmelif(it->second, outXML);
	outXML << "\t</Joints>" << endl;

	outXML << "</MultiBody>" << endl;
	outXML.close();
}

void writeAmelif (const Body * body, ofstream & xml, const std::map<string,std::vector<string> > & bodyNameMap,
		const std::string & vrmlPath)
{	
	assert(body != 0x0);
	xml <<"\t\t<Body id=\"" << body->id_ << "\">" << endl;
	xml <<"\t\t\t<Label>"   << body->name_ << "</Label>" << endl;
	xml <<"\t\t\t<Mass>"    << body->mass_ << "</Mass>" << endl;
	xml <<"\t\t\t<CoM>"     << body->com_ << "</CoM>" << endl;
	xml <<"\t\t\t<Inertia>";

	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			xml << body->inertia_ (i,j);
			if (i !=2 || j != 2)
				xml << " ";
		}
	}

	xml << "</Inertia>" << endl;
	std::map<string,std::vector<string> >::const_iterator it = bodyNameMap.find(body->name_);
	if ( it != bodyNameMap.end())
	{
		const std::vector<string> & vrmls = it->second;
		for(unsigned i = 0; i < vrmls.size(); ++i)
			xml << "\t\t\t<File>" << vrmlPath << "vrml/" <<  (vrmls[i]) << ".wrl</File>" << std::endl;
	}
	xml << "\t\t</Body>" << endl;
}


void writeAmelif (const Joint * j, ofstream & xml)
{
	double pMin = j->positionMin_;
	double pMax = j->positionMax_;

	char axe;
	if (j->axis_ == afbase::vector3d(1,0,0))
		axe = 'x';
	else if (j->axis_ == afbase::vector3d(0,1,0))
		axe = 'y';
	else if (j->axis_ == afbase::vector3d(0,0,1))
		axe = 'z';
	else
	{
		pMin = - j->positionMax_;
		pMax = - j->positionMin_;
		axe = 'y';
	}

	if (j->type_ == "fixed")
		xml << "\t\t<Joint id=\"" << j->id_ << "\" type=\"fixed\" innerId=\"" << j->innerBodyId_ << "\" outerId=\"" << j->outerBodyId_ << "\">" << endl;
	else if (j->type_ == "revolute")
		xml << "\t\t<Joint id=\"" << j->id_ << "\" type=\"revolute\" axis=\"" << axe << "\" innerId=\"" << j->innerBodyId_ << "\" outerId=\"" << j->outerBodyId_ << "\">" << endl;
	else
		std::cout << " WUT ?" << std::endl;
	xml << "\t\t\t<Label>" << j->name_ << "</Label>" << endl;

	if (j->type_ != "fixed")
	{
		xml << "\t\t\t<TorqueLimit>" << j->torqueMax_ << "</TorqueLimit>" << endl;
		xml << "\t\t\t<SpeedLimit>" << j->speedMax_ << "</SpeedLimit>" << endl;
	}

	xml << "\t\t\t<StaticParameters>" 
		<< j->staticXYZ_[0] << " " << j->staticXYZ_[1] << " " << j->staticXYZ_[2]
	<< " " << round(j->staticRPY_[0] *(180/3.1415926))<< " " << round(j->staticRPY_[1] *(180/3.1415926)) << " " << round(j->staticRPY_[2] *(180/3.1415926))
		<< "</StaticParameters>" << endl;

	if (j->type_ != "fixed")
	{
		xml << "\t\t\t<PositionMin>" << pMin << "</PositionMin>" << endl;
		xml << "\t\t\t<PositionMax>" << pMax << "</PositionMax>" << endl;
	}
	xml << "\t\t</Joint>" << endl;
}

void createBoundingVolumeFile(const std::string & filename,
	double rear, double front, double left, double right, double height	)
{
	//create the left feet bounding volume
	DECLARE_OFSTREAM(out, filename.c_str());

	copyFile("../data/foot_romeo_BV_Top.wrl", out);
	out << "\t\t\t\t\t\t";
	out << rear  << "  " << left   << "  0.0,     ";
	out << front << "  " << left   << "  0.0,     ";
	out << front << "  " << right  << "  0.0,     ";
	out << rear  << "  " << right  << "  0.0,     ";
	out << std::endl    << "\t\t\t\t\t\t";

	out << rear  << "  " << left  << "  " << height << ",  ";
	out << front << "  " << left  << "  " << height << ",  ";
	out << front << "  " << right << "  " << height << ",  ";
	out << rear  << "  " << right << "  " << height << ",  ";
	out << std::endl;
	copyFile("../data/foot_romeo_BV_Bottom.wrl", out);
	out.close();
}

void createBoundingVolumeFiles(const std::string & xmlfolder, const Robot::MultiBody* mb)
{
	//create the left feet bounding volume
	createBoundingVolumeFile(xmlfolder+"lfoot_romeo_BV.wrl",
		-mb->ankle_rearBound_, mb->ankle_frontBound_+mb->toe_frontBound_, mb->ankle_leftBound_, -mb->ankle_rightBound_, -mb->ankle_Height_ );

	createBoundingVolumeFile(xmlfolder+"lankle_romeo_BV.wrl",
		-mb->ankle_rearBound_, mb->ankle_frontBound_, mb->ankle_leftBound_, -mb->ankle_rightBound_, -mb->ankle_Height_ );
	createBoundingVolumeFile(xmlfolder+"ltoe_romeo_BV.wrl",
		-mb->toe_rearBound_, mb->toe_frontBound_, mb->toe_leftBound_, -mb->toe_rightBound_, -mb->toe_Height_ );

	// -- -- -- --
	//create the left feet bounding volume
	createBoundingVolumeFile(xmlfolder+"rfoot_romeo_BV.wrl",
		-mb->ankle_rearBound_, mb->ankle_frontBound_+mb->toe_frontBound_, mb->ankle_rightBound_, -mb->ankle_leftBound_, -mb->ankle_Height_ );

	createBoundingVolumeFile(xmlfolder+"rankle_romeo_BV.wrl",
		-mb->ankle_rearBound_, mb->ankle_frontBound_, mb->ankle_rightBound_, -mb->ankle_leftBound_, -mb->ankle_Height_ );
	createBoundingVolumeFile(xmlfolder+"rtoe_romeo_BV.wrl",
		-mb->toe_rearBound_, mb->toe_frontBound_, mb->toe_rightBound_, -mb->toe_leftBound_, -mb->toe_Height_ );
}

