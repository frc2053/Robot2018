#include "OI.h"

#include "Commands/Drive/ZeroYaw.h"
#include "Commands/Elevator/GoToElevatorPosition.h"
#include "Commands/Climber/RunClimberMotor.h"
#include "Commands/Intake/IntakeUntilCurrentSpike.h"
#include "Commands/Climber/DeployWings.h"
#include "Commands/Groups/SwitchToClimb.h"

OI::OI() {

	driverJoystick.reset(new TigerJoystick(0));
	operatorJoystick.reset(new TigerJoystick(1));

	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());

	/*operatorJoystick->aButton->WhenPressed(new GoToElevatorPosition(RobotMap::GROUND_POS_FT));
	operatorJoystick->yButton->WhenPressed(new GoToElevatorPosition(RobotMap::SCALE_POS_FT));
	operatorJoystick->bButton->WhenPressed(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT));

<<<<<<< HEAD
<<<<<<< HEAD
	operatorJoystick->startButton->WhileHeld(new RunClimberMotor(1, 0));
	operatorJoystick->startButton->WhenReleased(new RunClimberMotor(0, 0));

	operatorJoystick->selectButton->WhileHeld(new RunClimberMotor(-1, 0));
	operatorJoystick->selectButton->WhenReleased(new RunClimberMotor(0, 0));*/

	//ELEVATOR POSITIONS
	operatorJoystick->yButton->WhenPressed(new GoToElevatorPosition(RobotMap::SCALE_POS_FT, false));
	operatorJoystick->aButton->WhenPressed(new GoToElevatorPosition(0, false));
	operatorJoystick->xButton->WhenPressed(new GoToElevatorPosition(RobotMap::SWITCH_POS_FT, false));
	operatorJoystick-> bButton->WhenPressed(new GoToElevatorPosition(5.5, false)); //Go to neutral position for Scale
	//ELEVATOR POSITIONS

	//INTAKE/OUTTAKE
	operatorJoystick->leftShoulderButton->WhenActive(new IntakeUntilCurrentSpike(1, 0, true));
	operatorJoystick->leftShoulderButton->WhenInactive(new IntakeUntilCurrentSpike(0, 0, false)); //NOT NECESSARY BUT CAN OVERRIDE IN CASE OF ACCIDENTAL INTAKE BUTTON HIT

	operatorJoystick->rightShoulderButton->WhenActive(new IntakeUntilCurrentSpike(-1, 0, false));
	operatorJoystick->rightShoulderButton->WhenInactive(new IntakeUntilCurrentSpike(0, 0, false));
	//INTAKE/OUTTAKE

	//CLIMBING
	operatorJoystick->GetRightTrigger()->WhenActive(new GoToElevatorPosition(7, false)); //Go to Climb Position
	//operatorJoystick->selectButton->WhenPressed(new DeployWings());
	operatorJoystick->startButton->WhenPressed(new GoToElevatorPosition(6, true));
	//CLIMBLING
	//operatorJoystick->startButton->WhenPressed(new SwitchToClimb());
	//operatorJoystick->startButton->WhenPressed(new SwitchToClimb());

	//the boolean is if we want to stop when we hit current spike
	//this means we dont need to have a command stop the intake it should do it on its own
	//operatorJoystick->GetLeftTrigger()->WhenActive(new IntakeUntilCurrentSpike(1, 0, true));

	//this is to shoot out the box
	//we want this to be manual because the driver needs to make sure we push it out all the way
	//no way to check if box has completely left the intake
	//operatorJoystick->GetRightTrigger()->WhileActive(new IntakeUntilCurrentSpike(-1, 0, false));
	//operatorJoystick->GetRightTrigger()->WhenInactive(new IntakeUntilCurrentSpike(0, 0, false));
}

std::shared_ptr<TigerJoystick> OI::GetDriverJoystick() {
	return driverJoystick;
}

std::shared_ptr<TigerJoystick> OI::GetOperatorJoystick() {
	return operatorJoystick;
}
