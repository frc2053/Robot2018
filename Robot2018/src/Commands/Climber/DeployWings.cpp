#include "DeployWings.h"
#include "../../Robot.h"

DeployWings::DeployWings(bool direction) {
	Requires(Robot::climberSubsystem.get());
	isDone = false;
	currentDirection = direction;
}

void DeployWings::Initialize() {
	isDone = false;
}

// Called repeatedly when this Command is scheduled to run
// Make this return true when this Command no longer needs to run execute()

void DeployWings::Execute() {
	if(currentDirection == 1) {
		Robot::climberSubsystem->ReleaseWings();
	}
	if(currentDirection == 0) {
		Robot::climberSubsystem->RetractWings();
	}
	isDone = true;
}

bool DeployWings::IsFinished() {
	return isDone;
}

// Called once after isFinished returns true

void DeployWings::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run

void DeployWings::Interrupted() {

}
