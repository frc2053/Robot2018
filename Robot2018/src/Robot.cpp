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
	timeToWait = 0;

	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());

	SmartDashboard::PutBoolean("Left (False) or Right (True)", leftOrRight);
	SmartDashboard::PutBoolean("Do Scale", doScale);
	SmartDashboard::PutBoolean("Just Straight", justStraight);
	SmartDashboard::PutNumber("Time to Wait", timeToWait);

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);

<<<<<<< HEAD
	//autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());
=======
	autoChooser.AddDefault("Switch Back", "Switch Back");
	autoChooser.AddObject("Scale", "Scale");
	autoChooser.AddObject("Switch Side", "Switch Side");
	autoChooser.AddObject("Switch Front", "Switch Front");
>>>>>>> a8e51de0e47a865d261c43dec6cfaacabe68417f

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

	timeToWait = SmartDashboard::GetNumber("Time to Wait", 0);
	selectedMode = (std::string) autoChooser.GetSelected();

	//move to switch height
	Command* toSwitchHeight = new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false);
	toSwitchHeight->Start();

	//get data to choose correct auto modes
	while(gameData.size() < 3 && gameDataTimer.Get() <= 1) {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	std::cout << "Got Game Data from FMS." << std::endl;

	leftOrRight = SmartDashboard::GetBoolean("Left (False) or Right (True)", false);
	if(leftOrRight == true) {
		LoR = "R";
	}
	else {
		LoR = "L";
	}
	std::cout << "Robot Start Field Position: " << LoR << std::endl;

	justStraight = SmartDashboard::GetBoolean("Just Straight", false);
	doScale = SmartDashboard::GetBoolean("Do Scale", true);
	doScale = false;
	//cal wheels
	//Robot::swerveSubsystem->CalibrateWheelsSimple();

	//we need to make sure we are elevator mode
	Robot::elevatorSubsystem->SwitchToElevatorMotor();

	if(!(gameData.size() == 3) || (justStraight == true)) {
		if(justStraight == true) {
		std::cout << "User Selected Go Straight Only" << std::endl;
		}

		if(gameData.size() != 3) {
		std::cout << "PROBLEM WITH GAME DATA! - Just Go Straight" << std::endl;
		}

		std::string straightPathPath = "/home/lvuser/S.csv";
		FILE* straightPath = fopen(straightPathPath.c_str(), "r");
		lengthOfStraightPath = pathfinder_deserialize_csv(straightPath, trajStraight);
		fclose(straightPath);	cmdStraight = new FollowPath(trajStraight, lengthOfStraightPath, 0);
		std::cout << "Straight Pathfinder Trajectory Points: " << lengthOfStraightPath << "\n";
		cmdStraight->Start();
	}


	else {
		if(gameData.size() == 3) {
			std::cout << "Game Data Correct Length" << std::endl;
		}

		char switchSide = gameData.at(0);
		char scaleSide = gameData.at(1);
		char oppSwitchSide = gameData.at(2);

		std::cout << "SWITCH SIDE: " << switchSide << std::endl;;
		std::cout << "SCALE SIDE: " << scaleSide << std::endl;;
		std::cout << "OPPONENT SWITCH SIDE: " << oppSwitchSide << std::endl;;

		if(selectedMode == "Switch Back" || selectedMode == "Switch Side" || selectedMode == "Switch Front") {
			std::string toPath = MakeDecision(switchSide, scaleSide, LoR.at(0), doScale);

			if(selectedMode == "Switch Back") {
				toPath = "B" + toPath;
			}
			if(selectedMode == "Switch Front") {
				toPath = "F" + toPath;
			}
			if(selectedMode == "Switch Side") {
				toPath = "S" + toPath;
			}

			std::cout << "toPath: " << toPath << std::endl;
			std::cout << "switchPath: " << toPath.substr(0,2) << std::endl;
			std::cout << "scalePath: " << toPath.substr(1,2) << std::endl;
			LoadChosenPath(toPath.substr(0,2), toPath.substr(1,2));

			cmdSwitch = new FollowPath(trajToSwitch, lengthOfSwitchTraj, 0);
			std::cout << "Switch Pathfinder Trajectory Points: " << lengthOfSwitchTraj << std::endl;

			cmdScale = new FollowPath(trajToScale, lengthOfScaleTraj, 0);
			std::cout << "Scale Pathfinder Trajectory Points: " << lengthOfScaleTraj << std::endl;

			//MAKE THIS USER INPUT FROM DASH
			std::this_thread::sleep_for(std::chrono::seconds(timeToWait));

			cmdSwitch->Start();
		}
		if(selectedMode == "Scale") {
			std::string toPath = MakeDecisionScale(scaleSide, LoR.at(0));
			LoadScaleOnlyPath(toPath);
			cmdScale = new FollowPath(trajToSwitch, lengthOfScaleTraj, 0);
			cmdScale->Start();

			Command* toScaleHeight = new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false);
			toScaleHeight->Start();
		}
	}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
	//MATCHTIME = DriverStation::GetInstance().GetMatchTime();

	//std::cout << "In Auto Periodic" << std::endl;

	if(cmdSwitch != nullptr) {
		if(cmdSwitch->IsCompleted()) {

			//std::cout << "Switch Cube Pooper" << std::endl;
			Command* poopCube = new IntakeUntilCurrentSpike(.5, -1, false);
			poopCube->Start();

			//std::cout << "Retrieve Another Cube" << std::endl;
			CommandGroup* grabSecondCube = new GrabSecondCube();
			grabSecondCube->Start();

			if(lengthOfScaleTraj != 0) {
				//std::cout << "Scale Trajectory Exists" << std::endl;
				if(!runOnce) {
					std::cout << "Going to Scale" << std::endl;
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
			//std::cout << "Scale Cube Pooper" << std::endl;
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
	//MATCHTIME = DriverStation::GetInstance().GetMatchTime();
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

void Robot::LoadScaleOnlyPath(std::string scalePathName) {
	std::string path = "/home/lvuser/";
	std::string csvEx = ".csv";
	scalePathName = path + scalePathName + csvEx;
	std::cout << "scale path name: " << scalePathName << std::endl;

	FILE *scaleFile = fopen(scalePathName.c_str(), "r");
	lengthOfScaleTraj = pathfinder_deserialize_csv(scaleFile, trajToScale);
	fclose(scaleFile);
}

void Robot::LoadChosenPath(std::string switchPathName, std::string scalePathName) {
	std::string path = "/home/lvuser/";
	std::string csvEx = ".csv";
	switchPathName = path + switchPathName + csvEx;
	std::cout << "switchPathName: " << switchPathName << std::endl;

	FILE *switchFile = fopen(switchPathName.c_str(), "r");
	std::cout << "opened file!" << std::endl;

	lengthOfSwitchTraj = pathfinder_deserialize_csv(switchFile, trajToSwitch);
	std::cout << "length of switch traj: " << lengthOfSwitchTraj << std::endl;

	fclose(switchFile);

	std::string temp = scalePathName.substr(1,1);

	std::cout << "TempScale: " <<  temp << std::endl;

	if(temp != "N") {
		scalePathName = scalePathName + "S";
		scalePathName = path + scalePathName + csvEx;
		std::cout << "scalePathName: " << scalePathName << std::endl;

		FILE *scaleFile = fopen(scalePathName.c_str(), "r");
		lengthOfScaleTraj = pathfinder_deserialize_csv(scaleFile, trajToScale);
		std::cout << "length of scale traj: " << lengthOfSwitchTraj << std::endl;

		fclose(scaleFile);
	}
}

std::string Robot::MakeDecision(char switchSide, char scaleSide, char robotSide, bool doScale) {
	std::string retVal = "";
	if(robotSide == 'L') {
		std::cout << ".robotSide L" << std::endl;
		retVal = retVal + "L";
		if(switchSide == 'L') {
			std::cout << ".switchSide L" << std::endl;
			retVal = retVal + "L";
			if(doScale) {
				std::cout << ".doScale true" << std::endl;
				if(scaleSide == 'L') {
					std::cout << ".scaleSide L" << std::endl;
					retVal = retVal + "L";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
				if(scaleSide == 'R') {
					std::cout << ".scaleSide R" << std::endl;
					retVal = retVal + "R";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
			}
			retVal = retVal + "N";
			std::cout << "Decision: " << retVal << std::endl;
			return retVal;
		}
		if(switchSide == 'R') {
			std::cout << ".switchSide R" << std::endl;
			retVal = retVal + "R";
			if(doScale) {
				std::cout << ".doScale true" << std::endl;
				if(scaleSide == 'L') {
					std::cout << ".scaleSide L" << std::endl;
					retVal = retVal + "L";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
				if(scaleSide == 'R') {
					std::cout << ".scaleSide R" << std::endl;
					retVal = retVal + "R";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
			}
			retVal = retVal + "N";
			std::cout << "Decision: " << retVal << std::endl;
			return retVal;
		}
	}
	if(robotSide == 'R') {
		std::cout << ".robotSide R" << std::endl;
		retVal = retVal + "R";
		if(switchSide == 'L') {
			std::cout << ".switchSide R" << std::endl;
			retVal = retVal + "L";
			if(doScale) {
				std::cout << ".doSwitch true" << std::endl;
				if(scaleSide == 'L') {
					std::cout << ".scaleSide L" << std::endl;
					retVal = retVal + "L";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
				if(scaleSide == 'R') {
					std::cout << ".scaleSide R" << std::endl;
					retVal = retVal + "R";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
			}
			retVal = retVal + "N";
			std::cout << "Decision: " << retVal << std::endl;
			return retVal;
		}
		if(switchSide == 'R') {
			std::cout << ".switchSide R" << std::endl;
			retVal = retVal + "R";
			if(doScale) {
				std::cout << ".doScale true" << std::endl;
				if(scaleSide == 'L') {
					std::cout << ".scaleSide L" << std::endl;
					retVal = retVal + "L";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
				if(scaleSide == 'R') {
					std::cout << ".scaleSide R" << std::endl;
					retVal = retVal + "R";
					std::cout << "Decision: " << retVal << std::endl;
					return retVal;
				}
			}
			retVal = retVal + "N";
			std::cout << "Decision: " << retVal << std::endl;
			return retVal;
		}
	}
	std::cout << "Decision: " << retVal << std::endl;
	return "NNN";
}

std::string Robot::MakeDecisionScale(char scaleSide, char robotSide) {
	std::string retVal = "NN";
	std::string combined = "" + robotSide + scaleSide;
	if(combined == "LL") {
		retVal = "SFLL"; //SCALE FIRST FROM LEFT TO LEFT
	}
	if(combined == "LR") {
		retVal = "SFLR";
	}
	if(combined == "RR") {
		retVal = "SFRR";
	}
	if(combined == "RL") {
		retVal = "SFRL";
	}
	std::cout << "MakeDecisionScale: " << retVal << std::endl;
	return retVal;
}

START_ROBOT_CLASS(Robot);
