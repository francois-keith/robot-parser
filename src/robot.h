#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string> 
#include <fstream>
#include <vector>
#include <map>

#include "vector3.h" 
#include "matrix3.h" 


namespace Robot
{
	// tool methods

	//copy a file in an opened stream
	//void copyFile (const std::string & file, std::ofstream & out);


	// Definition of the multibody and the objects it contains.
	class Body;
	class Joint;
	class Camera;

	// Motors
	struct Motor
	{
		Motor();

		std::string motor_;
		std::string model_;
		double noLoadSpeed_;
		double torqueCst_;
	};

	// Multibody
	class MultiBody
	{
	public:
		MultiBody(const std::string & robotName);
		~MultiBody();
		bool parseBody(std::ifstream & ifs, bool useToes);
		bool parseBody2(std::ifstream & ifs, bool useToes);
		bool parseJoint(std::ifstream & ifs_joints, bool useToes);
		bool parseJoint2(std::ifstream & ifs_joints, bool useToes);
		bool parseLink(std::ifstream & ifs_links, bool useToes);
		bool parseMotor(std::ifstream & ifs);

		void parseWRL  (std::ifstream & ifs, std::ifstream & ifs_jointLimits);
		void parseTags(std::ifstream &ifs);

		Body*  parseBodyWRL  (std::ifstream & ifs_joints, const std::string & line);
		Joint* parseJointWRL (std::ifstream & ifs_joints, const std::string & line, Body* innerBody);
		void parseWRLJointLimit (std::ifstream & ifs_jointLimits);

		int nbBodies(void) const;
		int nbJoints(void) const;

		std::string getBodyFromTag(const std::string & tag) const;
		void getBodyNameToURDF(const std::string & robotName);
		void getBodyNameToVRML(const std::string & robotName);

		void computeGazePosition();

	private:
		void parseMotorList(std::ifstream & ifs_motor);
		double getTorqueOfMotor(const std::string & motorName);

	public:
		std::map<std::string, std::string> tagMap_;
		std::map<int,Body*> bodyMap_;
		std::map<int,Joint*> jointMap_;
		std::vector<Motor*> motorVec_;
		
		// info relative to the camera
		afbase::vector3d gazePosition_;
		std::map<std::string,Camera*> cameraMap_;

		int cameraJointGlobalId_;
		std::map<int,Joint*> cameraJointMap_;
		std::map<std::string,int> cameraJointNameToId_;

		// version of the model;
		std::string version_;
		bool isRomeo_;

		// foot dimension
	public:
		double ankle_rearBound_;
		double ankle_frontBound_;
		double ankle_rightBound_;
		double ankle_leftBound_;
		double ankle_Height_;

		double toe_rearBound_;
		double toe_frontBound_;
		double toe_rightBound_;
		double toe_leftBound_;
		double toe_Height_;
	};

	class Body
	{	
	public:
		Body();

		// -- read html file 
		void ParseName  (std::ifstream & ifs);
		void ParseName  (const std::string & line);
		void ParseInertia	(std::ifstream & ifs);
		void ParseCom	(std::ifstream & ifs);
		void display () const;
		
	public:
		static int bodyGlobalId_;
		static std::map<std::string,int> bodyNameToId_;
		static std::map<std::string, std::vector<std::string> > bodyNameToVRML_;
		static std::map<std::string, std::vector<std::string> > bodyNameToVRML_BV_;
		static std::map<std::string, std::string> bodyNameToURDF_;

		std::vector<Joint*> outerJointList_;

		int id_;
		std::string name_;
		double mass_;
		afbase::vector3d com_;
		afbase::matrix3d inertia_;
	};

	class Joint
	{
	public:
		Joint();
		
		void ParseName(std::string line);
		void ParseAxis(std::ifstream & ifs_joints);
		void ParseRange(std::ifstream & ifs_joints);
		void ParseStaticsParameters(std::ifstream & ifs_links);
		void display () const;

	public:
		static int jointGlobalId_;
		static int anonymousJointId_; //gather the joints that do not have a given id;
		static std::map<std::string,int> jointNameToId_;

		int id_;
		std::string name_;
		double positionMin_;
		double positionMax_;
		double speedMax_;
		double torqueMax_;

		Body* outerBody_;
		int outerBodyId_;
		std::string outerBodyName_;
		
		Body* innerBody_;
		int innerBodyId_;
		std::string innerBodyName_;

		std::string type_;
		afbase::vector3d axis_;
		afbase::vector3d staticXYZ_;
		afbase::vector3d staticRPY_;
		double rotorInertia_;
	};

	// A camera has simply a name and a position
	class Camera
	{
	public:
		Camera();
		std::string name_;
		afbase::vector3d pos_;
	};
}

#endif

