/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "GoToSwitchAndDropCube.h"
#include "../Autonomous/GoDistance.h"
#include "../Autonomous/DriveCommandAuto.h"
#include "../Intake/IntakeUntilCurrentSpike.h"
#include "../Elevator/GoToElevatorPosition.h"
GoToSwitchAndDropCube::GoToSwitchAndDropCube(bool isFar, bool isOnRight) {
	if(isFar) {
		AddSequential(new GoDistance(0, 10)); //go towards far switch
		AddSequential(new DriveCommandAuto(0, 0, 0, .3, 90)); //turn 90 degrees
		AddSequential(new GoDistance(5, 0)); //go towards switch
		AddSequential(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
		AddSequential(new IntakeUntilCurrentSpike(.5, -.5, false)); //spit cube
	}
	else {
		AddSequential(new GoDistance(5, 0)); //go towards switch
		AddSequential(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
		AddSequential(new IntakeUntilCurrentSpike(.5, -.5, false)); //spit cube
	}
}
