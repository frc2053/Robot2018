#include "ClimberSubsystem.h"
#include "WPILib.h"


ClimberSubsystem::ClimberSubsystem() : Subsystem("ClimberSubsystem") {
	std::cout << "Constructor for Climber Subsystem!" << std::endl;
	primaryMotor = RobotMap::elevatorClimberSubsystemPrimaryTalon;
	followerMotor01 = RobotMap::elevatorClimberSubsystemFollower01Talon;
	followerMotor02 = RobotMap::elevatorClimberSubsystemFollower02Talon;
	shifterSolenoid = RobotMap::elevatorClimberSubsystemShifterSolenoid;
	latchSolenoid = RobotMap::climberSubsystemLatchSolenoid;
	wingSolenoid = RobotMap::climberSubsystemWingSolenoid;
}

void ClimberSubsystem::InitDefaultCommand() {

}

void ClimberSubsystem::SetPrimaryMotor(double power) {
	primaryMotor->Set(ControlMode::PercentOutput, power);
}

void ClimberSubsystem::SwitchToClimberMode() {
	shifterSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
	primaryMotor->Set(ControlMode::PercentOutput, 0);
}

void ClimberSubsystem::ReleaseWings() {
	wingSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void ClimberSubsystem::RetractWings() {
	wingSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void ClimberSubsystem::HookLatch() {
	latchSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void ClimberSubsystem::UnhookLatch() {
	latchSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

