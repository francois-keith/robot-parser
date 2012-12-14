#ifndef FOOT_DIMENSIONS_H
#define FOOT_DIMENSIONS_H

#include "robot.h"

// Scheme of the left foot (from above)
// frontBound_
//    ______
//   /      |
//  /       |
//  |       |
//  |       | RightBound
//  |   x   |
//  |       |
//   \______|
// rearBound

// The reference for those data is the left foot
// based on romeo_hardware_dimension_lfoot.png
void loadFeetData(const std::string & robotName, Robot::MultiBody * mb)
{
	if (robotName == "romeo")
	{
	mb->ankle_rearBound_  = 0.080;
	mb->ankle_frontBound_ = 0.1135;
	mb->ankle_rightBound_ = 0.053;
	mb->ankle_leftBound_  = 0.068;
	mb->ankle_Height_ = 0.067;

	mb->toe_rearBound_  = 0.0;
	mb->toe_frontBound_ = 0.0885;
	mb->toe_rightBound_ = 0.053;
	mb->toe_leftBound_  = 0.068;
	mb->toe_Height_ = 0.015;
	}
	else
	{
		std::cerr << "The dimension of the feet of the robot where not found for robot "<< robotName << std::endl;
		std::cerr << "Please define them in the file footDimensions.h "<< robotName << std::endl;
	}
}

#endif // FOOT_DIMENSIONS_H
