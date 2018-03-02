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
	std::shared_ptr<frc::DoubleSolenoid> latchSolenoid;
	std::shared_ptr<frc::DoubleSolenoid> wingSolenoid;
	std::shared_ptr<frc::Servo> stopperServo;
public:
	ClimberSubsystem();
	void InitDefaultCommand();
	void ReleaseWings();
	void RetractWings();
	void UnhookLatch();
	void HookLatch();
	void SetPrimaryMotor(double power);
	void SetStopperServo(double power);
};

#endif
