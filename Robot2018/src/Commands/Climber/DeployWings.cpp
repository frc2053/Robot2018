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

void DeployWings::End() {

}

void DeployWings::Interrupted() {

}
