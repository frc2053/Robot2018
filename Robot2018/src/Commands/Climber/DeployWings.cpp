#include "DeployWings.h"

DeployWings::DeployWings() {
	Requires(Robot::climberSubsystem.get());
	isDone = false;
}

// Called just before this Command runs the first time
void DeployWings::Initialize() {
	isDone = false;
}

// Called repeatedly when this Command is scheduled to run
void DeployWings::Execute() {
	Robot::climberSubsystem->ReleaseWings();
	isDone = true;

}

// Make this return true when this Command no longer needs to run execute()
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
