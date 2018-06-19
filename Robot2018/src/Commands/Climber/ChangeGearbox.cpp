#include "ChangeGearbox.h"
#include "../../Robot.h"

ChangeGearbox::ChangeGearbox(bool inputMode) {
	Requires(Robot::elevatorSubsystem.get());
	mode = inputMode;
	isDone = false;
}

void ChangeGearbox::Initialize() {
	isDone = false;
}

void ChangeGearbox::Execute() {
	if(mode) {
		Robot::elevatorSubsystem->SwitchToClimberMotor();
	}
	else {
		Robot::elevatorSubsystem->SwitchToElevatorMotor();
	}
	isDone = true;
}

bool ChangeGearbox::IsFinished() {
	return isDone;
}

void ChangeGearbox::End() {

}

void ChangeGearbox::Interrupted() {

}
