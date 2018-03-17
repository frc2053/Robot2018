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
	void LoadScaleOnlyPath(std::string scalePathName);
	std::string MakeDecision(char sideOfSwitch, char robotSide, char switchSide);
	std::string MakeDecisionScale(char scaleSide, char robotSide);
	int FigureOutWhatAngleTheRobotProbablyStartedAtOnTheField(char robotSide, bool doStraight, bool doSwitch, char SwitchApproach, bool doScale);
	static double MATCHTIME;
private:
	frc::SendableChooser<std::string> autoChooser;
	frc::SendableChooser<std::string> robotPosChooser;
	std::string selectedMode;
	std::string gameData;
	bool runOnce;
	bool leftOrRight;
	std::string LoR;
	int timeToWait;
	bool doScale;
	bool doSwitch;
	bool justStraight;
	int OffsetAngle;
	std::string switchApproach;
	char switchApproachchar;

	Segment trajToSwitch[1024];
	Segment trajToScale[1024];
	Segment trajStraight[1024];
	Command* cmdSwitch;
	Command* cmdScale;
	Command* cmdStraight;
	int lengthOfSwitchTraj = 0;
	int lengthOfScaleTraj = 0;
	int lengthOfStraightPath = 0;
	frc::Timer gameDataTimer;
};

#endif
