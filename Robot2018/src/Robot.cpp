#include <Commands/Groups/EverythingAuto.h>
#include <Robot.h>
#include "Commands/Autonomous/DoNothingAuto.h"
#include "Pathfinder/TestFollower.h"
#include "Commands/Autonomous/DriveCommandAuto.h"
#include "Commands/Drive/DriveCommand.h"
#include "pathfinder.h"
#include "Commands/Autonomous/FollowPath.h"
#include "Commands/Autonomous/GoDistance.h"



std::unique_ptr<OI> Robot::oi;
std::unique_ptr<SwerveSubsystem> Robot::swerveSubsystem;
std::unique_ptr<IntakeSubsystem> Robot::intakeSubsystem;
std::unique_ptr<ElevatorSubsystem> Robot::elevatorSubsystem;
std::unique_ptr<ClimberSubsystem> Robot::climberSubsystem;

double Robot::MATCHTIME;

Segment* Robot::pathGenerated;
Command* Robot::pathFollower;

void Robot::RobotInit() {
	std::cout << "Robot is starting!" << std::endl;
	MATCHTIME = 0;

	RobotMap::init();
	std::cout << "Before follower!" << std::endl;
	std::cout << "After follower!" << std::endl;
	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());
	runOnce = false;

	leftOrRight = "L";
	doScale = true;

	SmartDashboard::PutString("Left or Right", leftOrRight);
	SmartDashboard::PutBoolean("Do Scale", doScale);

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);

	//generate paths
	Waypoint p1 = {0, 0, d2r(0)};
	Waypoint p2 = {12, 0, d2r(0)};
	points[0] = p1;
	points[1] = p2;

	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, RobotMap::TIMESTEP, RobotMap::MAX_VEL, RobotMap::MAX_ACCEL, RobotMap::MAX_JERK, &candidate);
	int trajLength = candidate.length;
	std::cout << "trajLength: " << trajLength << "\n";
	pathGenerated = (Segment*)malloc(trajLength * sizeof(Segment));
	pathfinder_generate(&candidate, pathGenerated);


	//pathFollower = new FollowPath(pathGenerated, trajLength);

	autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());

	//Make the list of auto options avaliable on the Smart Dash
	SmartDashboard::PutData("Auto mode chooser", &autoChooser);

	//Robot::swerveSubsystem->CalibrateWheels();

}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
}

void Robot::AutonomousInit() {
	leftOrRight = SmartDashboard::GetString("Left or Right", "L");
	doScale = SmartDashboard::GetBoolean("Do Scale", true);
	std::cout << "Autonomous Init!" << std::endl;
	//we need to make sure we are elevator mode
	Robot::elevatorSubsystem->SwitchToElevatorMotor();

	if(gameData.size() == 3) {
		Command* autoCmd = new EverythingAuto(gameData.at(0), gameData.at(1), leftOrRight.at(0), doScale);
		autoCmd->Start();
	}
	else {
		std::cout << "GAME DATA NOT RECEIVED!";
		Command* fallback = new GoDistance(0, 5);
		fallback->Start();
	}
	//align the wheels straight
	//Robot::swerveSubsystem->CalibrateWheels();
	std::cout << "BACK IN AUTO INIT" << std::endl;
	//get the auto mode we want to run from the smart dashboard
	selectedMode.reset(autoChooser.GetSelected());
	if(selectedMode != nullptr) {
		//selectedMode->Start();
	}

	std::cout << "STARTING PATHFOLLOWER" << std::endl;

	//pathFollower->Start();

	std::cout << "ENDING PATHFOLLOWER" << std::endl;

}

void Robot::AutonomousPeriodic() {

	//std::cout << "MADE IT TO AUTO PERIODIC" << std::endl;

	//Scheduler::GetInstance()->Run();
	bool isFinished = false;
	if(!runOnce) {
	}
	if(isFinished && !runOnce) {
		std::cout << "Finished first traj! Rotating" << "\n";
		//Scheduler::GetInstance()->AddCommand(new DriveCommandAuto(0, 0, 0, 1, -90));
		runOnce = true;
	}
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;
	if(selectedMode != nullptr) {
		selectedMode->Cancel();
	}

	Robot::swerveSubsystem->SetDefaultCommand(new DriveCommand());

	RobotMap::elevatorClimberSubsystemShifterSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Robot::TeleopPeriodic() {
	MATCHTIME = DriverStation::GetInstance().GetMatchTime();
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);
