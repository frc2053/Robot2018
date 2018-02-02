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

<<<<<<< HEAD
// Called repeatedly when this Command is scheduled to run
// Make this return true when this Command no longer needs to run execute()
=======
>>>>>>> ce3394613c23ad1596312f251a3e1d46d227aa44
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

<<<<<<< HEAD
// Called once after isFinished returns true
=======
>>>>>>> ce3394613c23ad1596312f251a3e1d46d227aa44
void DeployWings::End() {

}

<<<<<<< HEAD
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
=======
>>>>>>> ce3394613c23ad1596312f251a3e1d46d227aa44
void DeployWings::Interrupted() {

}
