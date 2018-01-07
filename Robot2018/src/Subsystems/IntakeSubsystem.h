#pragma once

#include "../RobotMap.h"
#include <Commands/Subsystem.h>

class IntakeSubsystem : public frc::Subsystem {
private:
	std::shared_ptr<can::TalonSRX> leftIntakeTalon;
	std::shared_ptr<can::TalonSRX> rightIntakeTalon;
public:
	IntakeSubsystem();
	void InitDefaultCommand();
	void RunBothMotors(double power);
	void RunLeftMotor(double power);
	void RunRightMotor(double power);
};
