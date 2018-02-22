#include <Commands/Groups/EverythingAuto.h>
#include <Robot.h>
#include "Commands/Autonomous/DoNothingAuto.h"
#include "Pathfinder/TestFollower.h"
#include "Commands/Autonomous/DriveCommandAuto.h"
#include "Commands/Drive/DriveCommand.h"
#include "pathfinder.h"
#include "Commands/Autonomous/FollowPath.h"
#include "Commands/Elevator/ZeroElevator.h"
#include "Commands/Autonomous/GoDistance.h"



std::unique_ptr<OI> Robot::oi;
std::unique_ptr<SwerveSubsystem> Robot::swerveSubsystem;
std::unique_ptr<IntakeSubsystem> Robot::intakeSubsystem;
std::unique_ptr<ElevatorSubsystem> Robot::elevatorSubsystem;
std::unique_ptr<ClimberSubsystem> Robot::climberSubsystem;

double Robot::MATCHTIME;

void Robot::RobotInit() {
	std::cout << "Robot is starting!" << std::endl;
	RobotMap::init();

	MATCHTIME = 0;
	runOnce = false;
	leftOrRight = "L";
	doScale = true;

	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());

	SmartDashboard::PutString("Left or Right", leftOrRight);
	SmartDashboard::PutBoolean("Do Scale", doScale);

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);

	autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());

	//Make the list of auto options avaliable on the Smart Dash
	SmartDashboard::PutData("Auto mode chooser", &autoChooser);
}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	leftOrRight = SmartDashboard::GetString("Left or Right", "L");
	doScale = SmartDashboard::GetBoolean("Do Scale", true);
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
}

void Robot::AutonomousInit() {
	std::cout << "Autonomous Init!" << std::endl;
	//get data to choose correct auto modes
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	leftOrRight = SmartDashboard::GetString("Left or Right", "L");
	doScale = SmartDashboard::GetBoolean("Do Scale", true);

	//cal wheels
	Robot::swerveSubsystem->CalibrateWheelsSimple();
	//we need to make sure we are elevator mode
	Robot::elevatorSubsystem->SwitchToElevatorMotor();

	if(gameData.size() == 3) {
		std::cout << "got game data!" << std::endl;
		char switchSide = gameData.at(0);
		char scaleSide = gameData.at(1);
		char oppSwitchSide = gameData.at(2);
		std::cout << "SWITCH SIDE: " << switchSide << std::endl;
		std::cout << "SCALE SIDE: " << scaleSide << std::endl;
		std::cout << "OPPONENT SWITCH SIDE: " << oppSwitchSide << std::endl;

		std::string toPath = MakeDecision(switchSide, scaleSide, leftOrRight.at(0), doScale);
		LoadChosenPath(toPath.substr(0, 2), toPath.substr(2, 1));
	}
	else {
		std::cout << "GAME DATA NOT RECEIVED!";
		Command* fallback = new GoDistance(0, 1);
		fallback->Start();
	}

	//get the auto mode we want to run from the smart dashboard
    //selectedMode.reset(autoChooser.GetSelected());

	//if(selectedMode != nullptr) {
	//	selectedMode->Start();
	//}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
	MATCHTIME = DriverStation::GetInstance().GetMatchTime();
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;
	if(selectedMode != nullptr) {
		selectedMode->Cancel();
	}

	Robot::swerveSubsystem->SetDefaultCommand(new DriveCommand());
}

void Robot::TeleopPeriodic() {
	MATCHTIME = DriverStation::GetInstance().GetMatchTime();
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

void Robot::LoadChosenPath(std::string switchPathName, std::string scalePathName) {
	FILE *switchFile = fopen(switchPathName.c_str(), "r");
	int lengthSwitch = pathfinder_deserialize_csv(switchFile, trajToSwitch);

	if(scalePathName != "N") {
		FILE *scaleFile = fopen(scalePathName.c_str(), "r");
		int lengthScale = pathfinder_deserialize_csv(scaleFile, trajToScale);
	}
}

std::string Robot::MakeDecision(char switchSide, char scaleSide, char robotSide, bool doScale) {
	std::string retVal = "";
	if(robotSide == 'L') {
		retVal = retVal + "L";
		if(switchSide == 'L') {
			retVal = retVal + "L";
			if(doScale) {
				if(scaleSide == 'L') {
					retVal = retVal + "L";
					return retVal;
				}
				if(scaleSide == 'R') {
					retVal = retVal + "R";
					return retVal;
				}
			}
			retVal = retVal + "N";
			return retVal;
		}
		if(switchSide == 'R') {
			retVal = retVal + "R";
			if(doScale) {
				if(scaleSide == 'L') {
					retVal = retVal + "L";
					return retVal;
				}
				if(scaleSide == 'R') {
					retVal = retVal + "R";
					return retVal;
				}
			}
			retVal = retVal + "N";
			return retVal;
		}
	}
	if(robotSide == 'R') {
		retVal = retVal + "R";
		if(switchSide == 'L') {
			retVal = retVal + "L";
			if(doScale) {
				if(scaleSide == 'L') {
					retVal = retVal + "L";
					return retVal;
				}
				if(scaleSide == 'R') {
					retVal = retVal + "R";
					return retVal;
				}
			}
			retVal = retVal + "N";
			return retVal;
		}
		if(switchSide == 'R') {
			retVal = retVal + "R";
			if(doScale) {
				if(scaleSide == 'L') {
					retVal = retVal + "L";
					return retVal;
				}
				if(scaleSide == 'R') {
					retVal = retVal + "R";
					return retVal;
				}
			}
			retVal = retVal + "N";
			return retVal;
		}
	}
	return "NNN";
}

START_ROBOT_CLASS(Robot);
