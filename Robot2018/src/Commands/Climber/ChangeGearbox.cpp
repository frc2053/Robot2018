#include "ChangeGearbox.h"
#include "../../Robot.h"

ChangeGearbox::ChangeGearbox(bool inputMode) {
	Requires(Robot::elevatorSubsystem.get());
	mode = inputMode;
}

void ChangeGearbox::Initialize() {
}

void ChangeGearbox::Execute() {
	if(mode) {
		Robot::elevatorSubsystem->SwitchToClimberMotor();
	}
	else {
		Robot::elevatorSubsystem->SwitchToElevatorMotor();
	}
}

bool ChangeGearbox::IsFinished() {
	return true;
}

void ChangeGearbox::End() {

}

void ChangeGearbox::Interrupted() {

}
