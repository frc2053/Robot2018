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
#include "Commands/Elevator/GoToElevatorPosition.h"
#include "Commands/Groups/GrabSecondCube.h"
#include "Commands/Elevator/ElevatorControl.h"
#include "Commands/Intake/IntakeUntilCurrentSpike.h"


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
	leftOrRight = false; //false is left, true is right
	doScale = true;
	LoR = "L";
	justStraight = false;

	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());

	SmartDashboard::PutBoolean("Left (False) or Right (True)", leftOrRight);
	SmartDashboard::PutBoolean("Do Scale", doScale);
	SmartDashboard::PutBoolean("Just Straight", justStraight);

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);

	autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());

	//Make the list of auto options avaliable on the Smart Dash
	SmartDashboard::PutData("Auto mode chooser", &autoChooser);

	//timer to check for gameData
	gameDataTimer.Reset();
}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	leftOrRight = SmartDashboard::GetBoolean("Left (False) or Right (True)", false);
	doScale = SmartDashboard::GetBoolean("Do Scale", true);
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
}

void Robot::AutonomousInit() {
	std::cout << "Autonomous Init!" << std::endl;
	//gameData start timer
	gameDataTimer.Reset();
	gameDataTimer.Start();

	//move to switch height
	Command* toSwitchHeight = new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false);
	toSwitchHeight->Start();

	//get data to choose correct auto modes
	while(gameData.size() < 3 && gameDataTimer.Get() <= 1) {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	leftOrRight = SmartDashboard::GetBoolean("Left (False) or Right (True)", false);
	justStraight = SmartDashboard::GetBoolean("Just Straight", false);
	if(leftOrRight == true) {
		LoR = "R";
	}
	else {
		LoR = "L";
	}
	doScale = SmartDashboard::GetBoolean("Do Scale", true);

	//cal wheels
	//Robot::swerveSubsystem->CalibrateWheelsSimple();
	//we need to make sure we are elevator mode
	Robot::elevatorSubsystem->SwitchToElevatorMotor();

	if(!(gameData.size() == 3) || (justStraight == true)) {
		std::cout << "GAME DATA NOT RECEIVED!";
		std::string straightPathPath = "/home/lvuser/S.csv";
		FILE* straightPath = fopen(straightPathPath.c_str(), "r");
		lengthOfStraightPath = pathfinder_deserialize_csv(straightPath, trajStraight);
		fclose(straightPath);	cmdStraight = new FollowPath(trajStraight, lengthOfStraightPath, 0);
		std::cout << "length of straigh path: " << lengthOfStraightPath << "\n";
		cmdStraight->Start();
	}
	else {
		std::cout << "got game data!" << std::endl;
		char switchSide = gameData.at(0);
		char scaleSide = gameData.at(1);
		char oppSwitchSide = gameData.at(2);
		std::cout << "SWITCH SIDE: " << switchSide << "\n";
		std::cout << "SCALE SIDE: " << scaleSide << "\n";
		std::cout << "OPPONENT SWITCH SIDE: " << oppSwitchSide << "\n";
		std::cout << "LEFT OR RIGHT: " << LoR << "\n";

		std::string toPath = MakeDecision(switchSide, scaleSide, LoR.at(0), doScale);
		std::cout << "toPath: " << toPath;
		std::cout << "switchPath: " << toPath.substr(0,2) << "\n";
		std::cout << "scalePath: " << toPath.substr(1,2) << "\n";
		LoadChosenPath(toPath.substr(0,2), toPath.substr(1,2));

		cmdSwitch = new FollowPath(trajToSwitch, lengthOfSwitchTraj, 0);
		cmdScale = new FollowPath(trajToScale, lengthOfScaleTraj, 0);

		std::this_thread::sleep_for(std::chrono::seconds(8));
		cmdSwitch->Start();
	}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
	MATCHTIME = DriverStation::GetInstance().GetMatchTime();

	if(cmdSwitch != nullptr) {
		if(cmdSwitch->IsCompleted()) {
			Command* poopCube = new IntakeUntilCurrentSpike(.5, -1, false);
			poopCube->Start();

			CommandGroup* grabSecondCube = new GrabSecondCube();
			grabSecondCube->Start();

			if(lengthOfScaleTraj != 0) {
				if(!runOnce) {
					cmdScale->Start();
					Command* toScaleHeight = new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false);
					toScaleHeight->Start();
					runOnce = true;
				}
			}
		}
	}

	if(cmdScale != nullptr) {
		if(cmdScale->IsCompleted()) {
			Command* poopCubeScale = new IntakeUntilCurrentSpike(.5, -1, false);
			poopCubeScale->Start();
		}
	}
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;
	if(cmdSwitch != nullptr) {
		cmdSwitch->Cancel();
	}
	if(cmdScale != nullptr) {
		cmdScale->Cancel();
	}
	Robot::swerveSubsystem->CalibrateWheelsSimple();

	Robot::swerveSubsystem->SetDefaultCommand(new DriveCommand());
	Robot::elevatorSubsystem->SetDefaultCommand(new ElevatorControl());
}

void Robot::TeleopPeriodic() {
	MATCHTIME = DriverStation::GetInstance().GetMatchTime();
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

void Robot::LoadChosenPath(std::string switchPathName, std::string scalePathName) {
	std::string path = "/home/lvuser/";
	std::string csvEx = ".csv";
	switchPathName = path + switchPathName + csvEx;
	std::cout << "switchPathName: " << switchPathName << "\n";
	FILE *switchFile = fopen(switchPathName.c_str(), "r");
	std::cout << "opened file!" << "\n";
	lengthOfSwitchTraj = pathfinder_deserialize_csv(switchFile, trajToSwitch);
	std::cout << "length of switch traj: " << lengthOfSwitchTraj << "\n";
	fclose(switchFile);

	if(scalePathName != "N") {
		scalePathName = scalePathName + "S";
		scalePathName = path + scalePathName + csvEx;
		std::cout << "scalePathName: " << scalePathName << "\n";
		FILE *scaleFile = fopen(scalePathName.c_str(), "r");
		lengthOfScaleTraj = pathfinder_deserialize_csv(scaleFile, trajToScale);
		fclose(scaleFile);
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
