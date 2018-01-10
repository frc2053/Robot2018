#include "ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() : Subsystem("ClimberSubsystem") {
	std::cout << "Constructor for Climber Subsystem!" << std::endl;
	primaryMotor = RobotMap::climberSubsystemPrimaryMotor;
	followerMotor = RobotMap::climberSubsystemFollowerMotor;
}

void ClimberSubsystem::InitDefaultCommand() {

}

void ClimberSubsystem::SetPrimaryMotor(double power) {
	primaryMotor->Set(ControlMode::PercentOutput, power);
}
