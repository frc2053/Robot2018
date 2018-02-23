/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ElevatorControl.h"
#include "../../Robot.h"

ElevatorControl::ElevatorControl() {
	Requires(Robot::elevatorSubsystem.get());
	setpoint = RobotMap::elevatorClimberSubsystemPrimaryTalon->GetSelectedSensorPosition(0);
}

// Called just before this Command runs the first time
void ElevatorControl::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ElevatorControl::Execute() {
	double joystickVal = Robot::oi->GetOperatorJoystick()->GetLeftYAxis();
	if(joystickVal > 0) {
		setpoint = setpoint + 10;
	}

	if(joystickVal < 0) {
		setpoint = setpoint - 10;
	}
	RobotMap::elevatorClimberSubsystemPrimaryTalon->Set(ControlMode::Position, setpoint);
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorControl::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ElevatorControl::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorControl::Interrupted() {

}
