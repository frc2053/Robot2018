#include <Commands/Drive/ZeroWheels.h>

ZeroWheels::ZeroWheels() {
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
}

void ZeroWheels::Initialize() {
	isDone = false;
}

void ZeroWheels::Execute() {
	Robot::swerveSubsystem->CalibrateWheels();
	isDone = true;
}

bool ZeroWheels::IsFinished() {
	return isDone;
}

void ZeroWheels::End() {

}

void ZeroWheels::Interrupted() {

}
