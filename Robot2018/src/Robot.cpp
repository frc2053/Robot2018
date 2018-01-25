#include <Robot.h>
#include "Commands/Autonomous/DoNothingAuto.h"
#include "Pathfinder/TestFollower.h"


std::unique_ptr<OI> Robot::oi;
std::unique_ptr<SwerveSubsystem> Robot::swerveSubsystem;
std::unique_ptr<IntakeSubsystem> Robot::intakeSubsystem;
std::unique_ptr<ElevatorSubsystem> Robot::elevatorSubsystem;
std::unique_ptr<ClimberSubsystem> Robot::climberSubsystem;
std::unique_ptr<TestFollower> Robot::follower;


void Robot::RobotInit() {
	std::cout << "Robot is starting!" << std::endl;
	RobotMap::init();
	std::cout << "Before follower!" << std::endl;
	follower.reset(new TestFollower());
	std::cout << "After follower!" << std::endl;
	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);

	//generate paths
	follower->Generate();

	autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());

	//Make the list of auto options avaliable on the Smart Dash
	SmartDashboard::PutData("Auto mode chooser", &autoChooser);

	follower->Generate();
}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
}

void Robot::AutonomousInit() {
	std::cout << "Autonomous Init!" << std::endl;
	//we need to make sure we are elevator mode
	Robot::elevatorSubsystem->SwitchToElevatorMotor();
	//align the wheels straight
	//Robot::swerveSubsystem->CalibrateWheels();
	//get the auto mode we want to run from the smart dashboard
	selectedMode.reset(autoChooser.GetSelected());
	if(selectedMode != nullptr) {
		selectedMode->Start();
	}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
	follower->FollowPath();
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;
	if(selectedMode != nullptr) {
		selectedMode->Cancel();
	}
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);
