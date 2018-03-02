#include "BrakeElevator.h"

BrakeElevator::BrakeElevator(double Power) {
	Requires(Robot::climberSubsystem.get());

	inputPower = Power;
	isDone = false;
}

void BrakeElevator::Initialize() {

	isDone = false;
}

void BrakeElevator::Execute() {
	Robot::climberSubsystem->SetStopperServo(inputPower);

	isDone = true;
}

bool BrakeElevator::IsFinished() {
	return isDone;
}

void BrakeElevator::End() {
}

void BrakeElevator::Interrupted() {

}
