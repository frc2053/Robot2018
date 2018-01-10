#ifndef ElevatorSubsystem_H
#define ElevatorSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>

class ElevatorSubsystem : public frc::Subsystem {
private:
	std::shared_ptr<can::TalonSRX> primaryMotor;
	std::shared_ptr<can::TalonSRX> followerMotor;
public:
	ElevatorSubsystem();
	void InitDefaultCommand();
	void GoToHeight(double inputHeight);
	int ConvertHeightToTicks(double inputHeight);
};

#endif
