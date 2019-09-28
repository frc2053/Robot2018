#include "Commands/Drive/ZeroWheels.h"

ZeroWheels::ZeroWheels() {
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
}

void ZeroWheels::Initialize() {
	isDone = false;
}

void ZeroWheels::Execute() {
	std::cout << "Hello!" << std::endl;
	//Robot::swerveSubsystem->CalibrateWheels();
	Robot::swerveSubsystem->CalibrateWheelsSimple();
	isDone = true;
}

bool ZeroWheels::IsFinished() {
	return isDone;
}

void ZeroWheels::End() {

}

void ZeroWheels::Interrupted() {

}
