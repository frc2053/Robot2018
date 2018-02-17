#include "ZeroElevator.h"
#include "../../Robot.h"

ZeroElevator::ZeroElevator() {
	Requires(Robot::elevatorSubsystem.get());
	isDone = false;
}

void ZeroElevator::Initialize() {

}

void ZeroElevator::Execute() {
	RobotMap::elevatorClimberSubsystemPrimaryTalon->SetSelectedSensorPosition(0, 0, 0);
	isDone = true;
}

bool ZeroElevator::IsFinished() {
	return false;
}

void ZeroElevator::End() {

}

void ZeroElevator::Interrupted() {

}
