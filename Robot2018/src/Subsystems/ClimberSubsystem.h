#ifndef ClimberSubsystem_H
#define ClimberSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>
#include "WPILib.h"

class ClimberSubsystem : public frc::Subsystem {
private:
	std::shared_ptr<can::TalonSRX> primaryMotor;
	std::shared_ptr<can::TalonSRX> followerMotor01;
	std::shared_ptr<can::TalonSRX> followerMotor02;
	std::shared_ptr<frc::DoubleSolenoid> shifterSolenoid;
	std::shared_ptr<frc::DoubleSolenoid> latchSolenoid;
	std::shared_ptr<frc::DoubleSolenoid> wingSolenoid;
public:
	ClimberSubsystem();
	void InitDefaultCommand();
	void SwitchToClimberMode();
	void ReleaseWings();
	void HookLatch();
	void SetPrimaryMotor(double power);
};

#endif
