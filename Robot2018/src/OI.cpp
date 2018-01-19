#include "OI.h"

#include "Commands/Drive/ZeroYaw.h"
#include "Commands/Elevator/GoToElevatorPosition.h"
#include "Commands/Climber/RunClimberMotor.h"
#include "Commands/Intake/IntakeUntilCurrentSpike.h"

OI::OI() {

	driverJoystick.reset(new TigerJoystick(0));
	operatorJoystick.reset(new TigerJoystick(1));

	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());

	operatorJoystick->aButton->WhenPressed(new GoToElevatorPosition(RobotMap::GROUND_POS_IN));
	operatorJoystick->yButton->WhenPressed(new GoToElevatorPosition(RobotMap::SCALE_POS_IN));
	operatorJoystick->xButton->WhenPressed(new GoToElevatorPosition(RobotMap::SWITCH_POS_IN));

	operatorJoystick->startButton->WhileHeld(new RunClimberMotor(1, 0));
	operatorJoystick->startButton->WhenReleased(new RunClimberMotor(0, 0));

	operatorJoystick->selectButton->WhileHeld(new RunClimberMotor(-1, 0));
	operatorJoystick->selectButton->WhenReleased(new RunClimberMotor(0, 0));

	operatorJoystick->GetRightTrigger()->WhileActive(new IntakeUntilCurrentSpike(1, 0));
	operatorJoystick->GetLeftTrigger()->WhenInactive(new IntakeUntilCurrentSpike(0, 0));
}

std::shared_ptr<TigerJoystick> OI::GetDriverJoystick() {
	return driverJoystick;
}

std::shared_ptr<TigerJoystick> OI::GetOperatorJoystick() {
	return operatorJoystick;
}
