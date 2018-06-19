/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "GoToHeightAndPoop.h"
#include "../Elevator/GoToElevatorPosition.h"
#include "../Intake/IntakeUntilCurrentSpike.h"


GoToHeightAndPoop::GoToHeightAndPoop(double height) {
	AddSequential(new GoToElevatorPosition(height, false));
	//AddSequential(new IntakeUntilCurrentSpike(.5, -1, false));
}
