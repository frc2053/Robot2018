#include "GoToElevatorPosition.h"

GoToElevatorPosition::GoToElevatorPosition(double inputHeight, bool ifClimbing) {
	Requires(Robot::elevatorSubsystem.get());
	Requires(Robot::climberSubsystem.get());

	heightTarget = inputHeight;
	isClimbing = ifClimbing;
	isDone = false;
}

void GoToElevatorPosition::Initialize() {
	if(isClimbing && (Robot::MATCHTIME <= 35))
	{
		//Robot::climberSubsystem->SwitchToClimberMode();
	}

	isDone = false;
}

void GoToElevatorPosition::Execute() {
	Robot::elevatorSubsystem->GoToHeight(heightTarget);
	isDone = true;
}

bool GoToElevatorPosition::IsFinished() {
	return isDone;
}

void GoToElevatorPosition::End() {
}

void GoToElevatorPosition::Interrupted() {

}
