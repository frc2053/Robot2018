#ifndef ElevatorSubsystem_H
#define ElevatorSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>
#include "WPILib.h"

class ElevatorSubsystem : public frc::Subsystem {
private:
	std::shared_ptr<can::TalonSRX> primaryMotor;
	std::shared_ptr<can::TalonSRX> followerMotor01;
	std::shared_ptr<can::TalonSRX> followerMotor02;
	std::shared_ptr<frc::DoubleSolenoid> shifterSolenoid;
public:
	ElevatorSubsystem();
	void InitDefaultCommand();
	void SwitchToElevatorMotor();
	void GoToHeight(double inputHeight);
	int ConvertHeightToTicks(double inputHeight);
};

#endif
