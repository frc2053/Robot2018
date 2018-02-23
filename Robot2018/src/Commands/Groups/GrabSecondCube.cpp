/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "GrabSecondCube.h"

#include "../Elevator/GoToElevatorPosition.h"
#include "../Autonomous/DriveCommandAuto.h"
#include "../../RobotMap.h"
#include "../Intake/IntakeUntilCurrentSpike.h"

GrabSecondCube::GrabSecondCube() {
	AddSequential(new DriveCommandAuto(0,0,0,1,135));
	AddSequential(new GoToElevatorPosition(RobotMap::GROUND_POS_FT, false));
	AddSequential(new IntakeUntilCurrentSpike(1, .6, true));
	AddSequential(new DriveCommandAuto(0,0,0,1,180));
	AddSequential(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
}
