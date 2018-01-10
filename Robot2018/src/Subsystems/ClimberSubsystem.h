#ifndef ClimberSubsystem_H
#define ClimberSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>

class ClimberSubsystem : public frc::Subsystem {
private:
	std::shared_ptr<can::TalonSRX> primaryMotor;
	std::shared_ptr<can::TalonSRX> followerMotor;
public:
	ClimberSubsystem();
	void InitDefaultCommand();
	void SetPrimaryMotor(double power);
};

#endif
