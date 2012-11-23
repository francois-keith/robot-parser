// This file gathers the methods used to parse a vrml file.
// The vrml parser is specified for OpenHRP specific vrml

#include "robotParser.h"
#include "tools.h"
#include "matrix_operation.hxx"

#include <sstream>

using namespace Robot;
using namespace std;

// main method to parse the wrl file. 
// in ifs: ifstream corresponding to the multibody
// in ifs_jointLimits: additional ifstream redefining the joint limits
void MultiBody::parseWRL(ifstream & ifs, ifstream & ifs_jointLimits)
{
	std::string line;
	while ( (getline(ifs, line) ) &&
			(line.find("Segment") == string::npos)
			){}

	if (line.find("PROTO") == string::npos)
		(getline(ifs, line) );

	while ( (getline(ifs, line) ) &&
			(line.find("Segment") == string::npos)
			){}

	parseBodyWRL (ifs, line);
	ifs.close();

	// If the extra joint limits file is provided, use it.
	if(ifs_jointLimits)
		parseWRLJointLimit (ifs_jointLimits);
}

//parsers
Body* MultiBody::parseBodyWRL (ifstream & ifs, const std::string & line)
{
	// parse the name of the body
	Body *body = new Body();

	// get the name
	body->name_ = parseTableElement<std::string>(line, "", "DEF", "Segment");
	// std::cout << " ** parsing Body " << body->name_ << std::endl;

	body->id_ = Body::bodyGlobalId_;
	++Body::bodyGlobalId_;
	Body::bodyNameToId_[body->name_]=body->id_;
	bodyMap_[body->id_] = body;

	int level = 0;
	std::string line2;
	std::string draft;
	while ( getline(ifs, line2) )
	{
		// std::cout << " lLINE    " << line2 << std::endl;
		std::string tmp("");
		std::istringstream istt(line2);
		istt >> tmp;

		if (tmp == "centerOfMass" )
			istt >> body->com_;
		else if (tmp == "mass" )
			istt >> body->mass_;
		else if (tmp == "momentsOfInertia" )
			istt >> draft >> body->inertia_;
		else if (tmp == "children" )
		{
			int levelCrochet = 0;
			std::string tmp2;

			while (istt >> tmp2)
			{
				if (tmp2.find("[") != string::npos)
					++levelCrochet;
				else if (tmp2.find("]") != string::npos)
					-- levelCrochet;
			}

			bool childrenEnd = false;
			while ( childrenEnd == false && ifs >> tmp2)
			{
				if (tmp2.find("[") != string::npos)
					++levelCrochet;
				else if (tmp2.find("]") != string::npos)
				{
					-- levelCrochet;
					// std::cout << " -- levelCrochet " << levelCrochet << std::endl;
					if (levelCrochet == 0)childrenEnd = true;
				}
				else if (tmp2 == "url")
				{
					std::string vrmlFull;
					ifs >> vrmlFull;
					std::string vrml = vrmlFull.substr(1, vrmlFull.size()-6);

					Body::bodyNameToVRML_[body->name_].push_back(vrml);
					// std::cout << " vrml " << vrml << std::endl;
					while (tmp2 != "}")
					{ ifs >> tmp2; }
				}
			}
		}
		else if ( line2.find("{") != string::npos)
			++level;
		//end of the body
		else if (tmp == "}" && level > 0)
			--level;

		else if (tmp == "}" && level == 0)
		{
			// std::cout << " The end of the body " << body->name_ << std::endl;
			// std::cout << "  with line " << line2 << std::endl;

			// go to the next important item
			while ( true )
			{
				while ( (getline(ifs, line2) ) &&
						(line2.find("]") == string::npos) &&
						(line2.find("Joint") == string::npos)
						){}

				// this body is a leaf
				if (line2.find("]") != string::npos)
				{
					return body;
				}

				// there is a joint attached to that body
				else if (line2.find("Joint") != string::npos)
					parseJointWRL (ifs, line2, body);

				// ?? WUT ??
				else
				{
					std::cerr << " Well, this is odd " << std::endl;
//					std::exit(0);
				}
			}
		}
	}
//	body->display();

	return body;
}

Joint* MultiBody::parseJointWRL (ifstream & ifs, const std::string & line, Body* innerBody)
{
	int levelBracket = 0;
	if(line.find("{") != string::npos)
		++levelBracket;

	// parse the name of the body
	Joint * joint = new Joint;

	// get the name
	joint->name_ = parseTableElement<std::string>(line, "", "DEF", "Joint");
	joint->innerBody_ = innerBody;
	joint->innerBodyId_ = joint->innerBody_->id_;
	joint->innerBodyName_ = joint->innerBody_->name_;
	// std::cout << "Start " <<  joint->name_ << " inner " << innerBody->name_ << std::endl;

	std::string tmp;
	std::string draft;
	std::string line2;

	while ( getline(ifs, line2) )
	{
		std::istringstream istt(line2);
		istt >> tmp;
		if (tmp == "#" )
			std::cout << " skipping line "  << line2 << std::endl;
		else if (tmp == "llimit" )
		{
			// skip the value in radian.
			istt >> draft >> draft;
			std::string value = draft.substr(1, draft.size() - 1);
			std::istringstream istt2(value);
			istt2 >> joint->positionMin_;
		}
		else if (tmp == "ulimit" )
		{
			// skip the value in radian.
			istt >> draft >> draft;
			std::string value = draft.substr(1, draft.size() - 1);
			std::istringstream istt2(value);
			istt2 >> joint->positionMax_;
		}
		else if (tmp == "rotorInertia" )
			istt >> joint->rotorInertia_;
		else if (tmp == "jointAxis" )
		{
			istt >> draft;
			std::string axe = draft.substr(1,1);
			if (axe == "X") joint->axis_ = afbase::vector3d(1,0,0);
			else if (axe == "Y") joint->axis_ = afbase::vector3d(0,1,0);
			else if (axe == "Z") joint->axis_ = afbase::vector3d(0,0,1);
			else std::cout << " Unknown axis " << axe <<  std::endl;
		}
		else if (tmp == "translation" )
			istt >> joint->staticXYZ_;
		else if (tmp == "#rotation" ){}
		else if (tmp == "rotation" )
		{
			afbase::vector3d axis(0,0,1);
			double angle(0);
			istt >> axis >> angle;
			afbase::matrix3d rot;
			uThetaToMatrix(rot, axis, angle);
			joint->staticRPY_ = computeEulerFromRotationMatrix ( rot );
		}
		else if (tmp == "jointType" )
		{
			istt >> draft;
			joint->type_  = draft.substr(1,draft.size() - 2);
			if (joint->type_ == "rotate")
				joint->type_ = "revolute";
		}
		else if (tmp == "jointId" )
		{
			istt >> joint->id_;
		}

		// look for the outer body.
		else if (tmp == "children" )
		{
			if (joint->id_ == -1)
			{
				joint->id_ = -1 +Joint::anonymousJointId_;
				--Joint::anonymousJointId_;
			}
			else
			{
				Joint::jointGlobalId_ = std::max(Joint::jointGlobalId_, joint->id_);
			}
			jointMap_[joint->id_] = joint;
			Joint::jointNameToId_[joint->name_] = joint->id_;

			// Look for the key world segment.
			while ( (getline(ifs, line2) ) && (line2.find("Segment") == string::npos) )
			{
				if (line2.find("{") != std::string::npos)
					++levelBracket;
			}

			joint->outerBody_ = parseBodyWRL (ifs, line2);
			// std::cout << " joint " << joint->name_ <<  " is back " << levelBracket << std::endl;
			joint->outerBodyId_ = joint->outerBody_->id_;
			joint->outerBodyName_ = joint->outerBody_->name_;

			while ( levelBracket >0 && (getline(ifs, line2) ))
			{
				if (line2.find("}") != std::string::npos)
					--levelBracket;
			}

			// std::cout << " end of Joint " << joint->name_<< std::endl;
			// std::cout << "   with line " << line2 << std::endl;
			break;
		}
	}

//	joint->display();
	return joint;
}


void MultiBody::parseWRLJointLimit (ifstream & ifs_jointLimits)
{
	std::string line;

	// skip the two useless lines.
	getline(ifs_jointLimits, line);
	getline(ifs_jointLimits, line);

	// skip the two useless lines.
	while ( getline(ifs_jointLimits, line) )
	{
		if (! ( line.empty() ) )
		{
		std::istringstream ist(line);
		double id(0);
		double posMin(0);
		double posMax(0);
		double speedLimit(0);
		double torqueLimit(0);

		std::string	name;
		ist >> id >> name >> posMax >> posMin >> torqueLimit >> speedLimit;
		if(jointMap_[id]->name_ != name)
		{
			std::cerr << line << std::endl;
			std::cerr << " label different " << jointMap_[id]->name_ << " != " << name << std::endl;
		}

		if(jointMap_[id]->positionMin_ != posMin)
			std::cerr << " positionMin different for " << jointMap_[id]->name_ << "  " << jointMap_[id]->positionMin_ << " != " << posMin << std::endl;

		if(jointMap_[id]->positionMax_ != posMax)
			std::cerr << " positionMax different " << jointMap_[id]->name_ << "  " << jointMap_[id]->positionMax_ << " != " << posMax << std::endl;

		jointMap_[id]->positionMin_  = posMin;
		jointMap_[id]->positionMax_  = posMax;
		jointMap_[id]->speedMax_  = speedLimit;
		jointMap_[id]->torqueMax_ = torqueLimit;
		}
	}
}
