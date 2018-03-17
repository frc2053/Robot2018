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
	doScale = true;
	doSwitch = true;
	LoR = "L";
	justStraight = false;
	timeToWait = 0;
	OffsetAngle = 0;
	switchApproach = "";

	swerveSubsystem.reset(new SwerveSubsystem());
	intakeSubsystem.reset(new IntakeSubsystem());
	elevatorSubsystem.reset(new ElevatorSubsystem());
	climberSubsystem.reset(new ClimberSubsystem());
	oi.reset(new OI());

	SmartDashboard::PutBoolean("Do Scale", doScale);
	SmartDashboard::PutBoolean("Do Switch", doSwitch);
	SmartDashboard::PutBoolean("Just Straight", justStraight);

	SmartDashboard::PutNumber("Time to Wait", timeToWait);

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);


	//autoChooser.AddDefault("Do Nothing Auto", new DoNothingAuto());

	autoChooser.AddDefault("Switch Back", "B");
	autoChooser.AddObject("Scale", "scale");
	autoChooser.AddObject("Switch Side", "S");
	autoChooser.AddObject("Switch Front", "F");

	robotPosChooser.AddDefault("Left", "L");
	robotPosChooser.AddObject("Right", "R");
	robotPosChooser.AddObject("Center", "C");

	//Make the list of auto options avaliable on the Smart Dash
	SmartDashboard::PutData("Auto mode chooser", &autoChooser);
	SmartDashboard::PutData("Robot Position Chooser", &robotPosChooser);


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

	Command* goDist1 = new GoDistance(14, 0);
	goDist1->Start();

	//Command* goDist2 = new GoDistance(0, 5);
	//goDist2->Start();

	//Command* rotate = new DriveCommandAuto(0, 0, 0, 1, 90);
	//rotate->Start();

	/*timeToWait = SmartDashboard::GetNumber("Time to Wait", 0);
	selectedMode = (std::string) autoChooser.GetSelected();
	LoR = (std::string) robotPosChooser.GetSelected();

	//test for auto modes  - also need to reset back in teleop
	//Robot::swerveSubsystem->SetAdjYaw(90);

	//move to switch height
	Command* toSwitchHeight = new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false);
	toSwitchHeight->Start();

	//get data to choose correct auto modes
	while(gameData.size() < 3 && gameDataTimer.Get() <= 1) {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	std::cout << "Got Game Data from FMS." << std::endl;


	std::cout << "Robot Start Field Position: " << LoR << std::endl;

	justStraight = SmartDashboard::GetBoolean("Just Straight", false);
	doScale = SmartDashboard::GetBoolean("Do Scale", true);
	doScale = false;
	//cal wheels
	Robot::swerveSubsystem->CalibrateWheelsSimple();

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
		fclose(straightPath);
		//load straight path and get length, robot starts at 0 degrees forward
	    //really this should be smarter because it could have strated sideways and then will go sideways - bad
		cmdStraight = new FollowPath(trajStraight, lengthOfStraightPath, 0);

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

		if(selectedMode == "B" || selectedMode == "F" || selectedMode == "S") {
			std::string toPath = MakeDecision(selectedMode.at(0), LoR.at(0), switchSide);

			std::cout << "toPath: " << toPath << std::endl;
			//std::cout << "switchPath: " << toPath.substr(0,2) << std::endl;
			//std::cout << "scalePath: " << toPath.substr(1,2) << std::endl;
			LoadChosenPath(toPath, "");

			//switchApproachchar =  switchApproach.substr(0, 0); NOT WORKING

			OffsetAngle = FigureOutWhatAngleTheRobotProbablyStartedAtOnTheField(LoR.at(0), justStraight, doSwitch, switchApproachchar, doScale);


			cmdSwitch = new FollowPath(trajToSwitch, lengthOfSwitchTraj, OffsetAngle);
			std::cout << "Switch Pathfinder Trajectory Points: " << lengthOfSwitchTraj << std::endl;

			cmdScale = new FollowPath(trajToScale, lengthOfScaleTraj, OffsetAngle);
			std::cout << "Scale Pathfinder Trajectory Points: " << lengthOfScaleTraj << std::endl;

			//MAKE THIS USER INPUT FROM DASH
			std::this_thread::sleep_for(std::chrono::seconds(timeToWait));

			cmdSwitch->Start();
		}
		if(selectedMode == "scale") {
			std::string toPath = MakeDecisionScale(scaleSide, LoR.at(0));
			LoadScaleOnlyPath(toPath);
			// that was trajtoSwitch and I change it to scale because it seemed like a cut and paste
			cmdScale = new FollowPath(trajToScale, lengthOfScaleTraj, 0);
			cmdScale->Start();

			Command* toScaleHeight = new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false);
			toScaleHeight->Start();
		}
	}*/
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
	//MATCHTIME = DriverStation::GetInstance().GetMatchTime();

	//std::cout << "In Auto Periodic" << std::endl;

	if(cmdSwitch != nullptr) {
		if(cmdSwitch->IsCompleted()) {

			std::cout << "Switch Cube Pooper" << std::endl;
			Command* poopCube = new IntakeUntilCurrentSpike(.5, -1, false);
			poopCube->Start();

			std::cout << "Retrieve Another Cube" << std::endl;
			CommandGroup* grabSecondCube = new GrabSecondCube();
			grabSecondCube->Start();

			if(lengthOfScaleTraj != 0) {
				std::cout << "Scale Trajectory Exists" << std::endl;
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
			std::cout << "Scale Cube Pooper" << std::endl;
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

	//test for auto modes  - also need to reset back in teleop
	//i think this should add in the negative of auto to the current - not a fixed position
	//will have to check
	//or rotate to a known position first then reset?  might be hard if stuck on a wall
	//Robot::swerveSubsystem->SetAdjYaw(0);

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

	if(scalePathName != "") {
		scalePathName = path + scalePathName + csvEx;
		std::cout << "scalePathName: " << scalePathName << std::endl;

		FILE *scaleFile = fopen(scalePathName.c_str(), "r");
		lengthOfScaleTraj = pathfinder_deserialize_csv(scaleFile, trajToScale);
		std::cout << "length of scale traj: " << lengthOfSwitchTraj << std::endl;

		fclose(scaleFile);
	}
}

std::string Robot::MakeDecision(char sideOfSwitch, char robotSide, char switchSide) {
	return std::string() + sideOfSwitch + robotSide + switchSide;
}

std::string Robot::MakeDecisionScale(char scaleSide, char robotSide) {
	std::string retVal = "";
	std::string combined = "scale" + robotSide + scaleSide;
	std::cout << "MakeDecisionScale: " << retVal << std::endl;
	return retVal;
}

int Robot::FigureOutWhatAngleTheRobotProbablyStartedAtOnTheField(char robotSide, bool doStraight, bool doSwitch, char switchApproach, bool doScale) {

	int retVal = 0;


	if(doStraight) {
		retVal = 0;
	}
	else {

		if(doSwitch) {
			if (switchApproach == 'F') { //front switch approch
				retVal = 0;
			}

			if (switchApproach == 'B') { //back switch approach
				retVal = 180;
			}

			if (switchApproach == 'S') { //side switch approach
				if (robotSide == 'L') {
					retVal = -90;
				}
				else {
					if (robotSide == 'R') {
						retVal = 90;
					}

				}

			}
		else  {
			if (doScale) {
				if (robotSide == 'L') {
							retVal = -90;
						}
				else {
					if (robotSide == 'R')
								retVal = 90;
						}
			}
		}

	}
	}


	return retVal;
}

START_ROBOT_CLASS(Robot);
