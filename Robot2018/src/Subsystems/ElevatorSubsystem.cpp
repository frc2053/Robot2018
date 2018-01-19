#include "ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() : Subsystem("ElevatorSubsystem") {
	std::cout << "Constructor for Elevator Subsystem!" << std::endl;
	primaryMotor = RobotMap::elevatorSubsystemPrimaryMotor;
}

void ElevatorSubsystem::InitDefaultCommand() {

}

void ElevatorSubsystem::GoToHeight(double inputHeight) {
	int ticks = ConvertHeightToTicks(inputHeight);
	primaryMotor->Set(ControlMode::Position, ticks);
}

int ElevatorSubsystem::ConvertHeightToTicks(double inputHeight) {
	int tickSetpoint = 0;
	return tickSetpoint;
}
