#ifndef _ROBOT_H
#define _ROBOT_H

#include "RobotMap.h"
#include "Subsystems/SwerveSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/ClimberSubsystem.h"
#include "OI.h"
#include "Pathfinder/TestFollower.h"

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
	void LoadChosenPath(std::string switchPathName, std::string scalePathName);
	std::string MakeDecision(char switchSide, char scaleSide, char robotSide, bool doScale);
	static double MATCHTIME;
private:
	frc::SendableChooser<frc::Command*> autoChooser;
	std::unique_ptr<frc::Command> selectedMode;
	std::string gameData;
	bool runOnce;
	std::string leftOrRight;
	bool doScale;
	Segment trajToSwitch[1024];
	Segment trajToScale[1024];
	Command* cmdSwitch;
	Command* cmdScale;
	int lengthOfSwitchTraj;
	int lengthOfScaleTraj;
};

#endif
