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
	//Robot::elevatorSubsystem->RunElevatorMotor(heightTarget);
	int delta = abs(RobotMap::elevatorClimberSubsystemPrimaryTalon->GetSelectedSensorPosition(0) - RobotMap::elevatorClimberSubsystemPrimaryTalon->GetClosedLoopTarget(0));
	if(RobotMap::elevatorClimberSubsystemPrimaryTalon->GetClosedLoopError(0) < 30) {
		isDone = true;
	}
}

bool GoToElevatorPosition::IsFinished() {
	return isDone;
}

void GoToElevatorPosition::End() {
}

void GoToElevatorPosition::Interrupted() {

}
