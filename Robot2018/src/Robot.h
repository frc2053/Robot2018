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

	static double MATCHTIME;

private:
	frc::SendableChooser<frc::Command*> autoChooser;
	std::unique_ptr<frc::Command> selectedMode;
	std::string gameData;
	bool runOnce;
	static const int POINT_LENGTH = 2;
	TrajectoryCandidate candidate;
	TrajectoryCandidate candidate2;
	Waypoint points[POINT_LENGTH];
	Waypoint points2[POINT_LENGTH];
	static Segment* pathGenerated;
	static Command* pathFollower;
	static Segment* pathGenerated2;
	static Command* pathFollower2;
	std::string leftOrRight;
	bool doScale;
};

#endif
