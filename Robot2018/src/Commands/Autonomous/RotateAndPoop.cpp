/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RotateAndPoop.h"
#include "DriveCommandAuto.h"
#include "../Intake/IntakeUntilCurrentSpike.h"

RotateAndPoop::RotateAndPoop(float time, float angle) {
	AddSequential(new DriveCommandAuto(0, 0, 0, time, angle));
	AddSequential(new IntakeUntilCurrentSpike(.5, -1, false));
}
