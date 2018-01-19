#ifndef _ROBOT_H
#define _ROBOT_H

#include "RobotMap.h"
#include "Subsystems/SwerveSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/ClimberSubsystem.h"
#include "OI.h"

class Robot : public frc::TimedRobot {
public:
	static std::unique_ptr<SwerveSubsystem> swerveSubsystem;
	static std::unique_ptr<IntakeSubsystem> intakeSubsystem;
	static std::unique_ptr<ElevatorSubsystem> elevatorSubsystem;
	static std::unique_ptr<ClimberSubsystem> climberSubsystem;
	static std::unique_ptr<OI> oi;
	virtual void RobotInit();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
};

#endif
