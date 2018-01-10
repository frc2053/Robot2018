#include "GoToGroundAndIntake.h"

#include "../Elevator/GoToElevatorPosition.h"
#include "../Intake/IntakeUntilCurrentSpike.h"

GoToGroundAndIntake::GoToGroundAndIntake() {
	Requires(Robot::elevatorSubsystem.get());
	Requires(Robot::intakeSubsystem.get());

	AddParallel(new GoToElevatorPosition(RobotMap::GROUND_POS_IN));
	AddParallel(new IntakeUntilCurrentSpike(0, 1));
}
