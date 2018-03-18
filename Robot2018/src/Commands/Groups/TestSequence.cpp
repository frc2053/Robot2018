/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "TestSequence.h"
#include "../Autonomous/GoDistance.h"

TestSequence::TestSequence() {
	AddSequential(new GoDistance(0, 6));
	AddSequential(new GoDistance(6, 0));
}
