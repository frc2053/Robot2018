#include "IntakeSubsystem.h"
#include "../RobotMap.h"

IntakeSubsystem::IntakeSubsystem() : Subsystem("IntakeSubsystem") {
	std::cout << "Constructor for Intake Subsystem!" << std::endl;
	leftIntakeTalon = RobotMap::intakeSubsystemLeftMotor;
	rightIntakeTalon = RobotMap::intakeSubsystemRightMotor;
}

void IntakeSubsystem::InitDefaultCommand() {

}

void IntakeSubsystem::RunBothMotors(double power) {
	leftIntakeTalon->Set(ControlMode::PercentOutput, power);
	rightIntakeTalon->Set(ControlMode::PercentOutput, power);
}

void IntakeSubsystem::RunLeftMotor(double power) {
	leftIntakeTalon->Set(ControlMode::PercentOutput, power);
}

void IntakeSubsystem::RunRightMotor(double power) {
	rightIntakeTalon->Set(ControlMode::PercentOutput, power);
}

double IntakeSubsystem::GetLeftCurrent() {
	return leftIntakeTalon->GetOutputCurrent();
}

double IntakeSubsystem::GetRightCurrent() {
	return rightIntakeTalon->GetOutputCurrent();
}
