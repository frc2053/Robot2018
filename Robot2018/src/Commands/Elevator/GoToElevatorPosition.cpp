#include "GoToElevatorPosition.h"

GoToElevatorPosition::GoToElevatorPosition(double inputHeight) {
	Requires(Robot::elevatorSubsystem.get());
	heightTarget = inputHeight;
	isDone = false;
}

void GoToElevatorPosition::Initialize() {
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
