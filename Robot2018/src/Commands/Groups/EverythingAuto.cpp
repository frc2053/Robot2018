#include <Commands/Groups/EverythingAuto.h>
#include "../Autonomous/GoDistance.h"
#include "../Autonomous/DriveCommandAuto.h"
#include "../Intake/IntakeUntilCurrentSpike.h"
#include "../Elevator/GoToElevatorPosition.h"
EverythingAuto::EverythingAuto(char switchSide, char scaleSide, char robotSide, bool doScale) {
	//suck in cube
	std::cout << "help!\n";
	std::cout << "switchSide: " << switchSide << std::endl;
	std::cout << "scaleSide: " << scaleSide << std::endl;
	std::cout << "robotSide: " << robotSide << std::endl;

	AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
	if(robotSide == 'L') {
		std::cout << "left side\n";
		if(switchSide == 'L') {
			//go to switch
			AddSequential(new GoDistance(-10, 3));
			//move to switch position
			AddParallel(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
			//poop cube
			AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
			//go to floor
			AddSequential(new GoToElevatorPosition(RobotMap::GROUND_POS_FT, false));
			//move a bit in range of cube
			AddSequential(new GoDistance(-1, .5));
			//pick up second cube
			AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
			if(doScale) {
				if(scaleSide == 'L') {
					//turn -90 degrees to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .3, -90));
					//move elevator up
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//go to scale
					AddSequential(new GoDistance(0, 4));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
				if(scaleSide == 'R') {
					//go to other side of scale
					AddSequential(new GoDistance(-1, 10));
					//rotate to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .3, -90));
					//move elevator
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
			}
		}
		if(switchSide == 'R') {
			std::cout << "switch right\n";
			//go left
			AddSequential(new GoDistance(-10, 0));
			//go forward towards switch
			AddSequential(new GoDistance(0, 4));
			//turn 90 to face switch
			AddSequential(new DriveCommandAuto(0, 0, 0, .3, 90));
			//poop cube
			AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
			//go towards second cube
			AddSequential(new GoDistance(-1, 0));
			//go to ground
			AddSequential(new GoToElevatorPosition(RobotMap::GROUND_POS_FT, false));
			//intake cube
			AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
			if(doScale) {
				if(scaleSide == 'L') {
					//strafe back towards left scale
					AddSequential(new GoDistance(-4, 0));
					//turn 180 to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .3, -90));
					//Elevator Raise
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//Poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
				}
				if(scaleSide == 'R') {
					std::cout << "scale right\n";
					//turn 180 to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .3, -90));
					//raise elevator
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
				}
			}
		}
	}
	if(robotSide == 'R') {
		std::cout << "here!\n";
		if(switchSide == 'L') {
			//go right past switch
			AddSequential(new GoDistance(-10, 0));
			//go forward to side of scale
			AddSequential(new GoDistance(0, 4));
			//turn to -90 to face switch
			AddSequential(new DriveCommandAuto(0, 0, 0, .4, -90));
			//go to switch height
			AddSequential(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
			//poop cube
			AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
			//go to ground
			AddSequential(new GoToElevatorPosition(RobotMap::GROUND_POS_FT, false));
			//move towards second cube
			AddSequential(new GoDistance(-1, 0));
			//pick up second cube
			AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
			if(doScale) {
				if(scaleSide == 'L') {
					//turn to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .4, 90));
					//move elevator up
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
				if(scaleSide == 'R') {
					//move back to get out of way
					AddSequential(new GoDistance(0, -1));
					//move left towards scale
					AddSequential(new GoDistance(5, 0));
					//turn around to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .4, 90));
					//move up to scale
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
			}
		}
		if(switchSide == 'R') {
			std::cout << "here!\n";
			//move right to switch
			AddSequential(new GoDistance(10, 3));
			//move to switch height
			AddSequential(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
			//poop cube
			AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
			//move right towards cube
			AddSequential(new GoDistance(1, 0));
			//intake second cube
			AddSequential(new IntakeUntilCurrentSpike(.3, .6, true));
			if(doScale) {
				if(scaleSide == 'L') {
					//go right towards scale
					AddSequential(new GoDistance(4, 0));
					//turn around to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .4, 90));
					//go to elevator height
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
				if(scaleSide == 'R') {
					std::cout << "here!\n";
					//turn around to face scale
					AddSequential(new DriveCommandAuto(0, 0, 0, .4, 90));
					//go to elevator height
					AddSequential(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
					//poop cube
					AddSequential(new IntakeUntilCurrentSpike(.3, -.5, false));
				}
			}
		}
	}
	std::cout << "end!\n";
}
