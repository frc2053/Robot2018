#include "Commands/Drive/SetOffset.h"

SetOffset::SetOffset(float input) {
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
	inputYaw = input;
}

void SetOffset::Initialize() {
	isDone = false;
}

void SetOffset::Execute() {
	Robot::swerveSubsystem->SetAdjYaw(inputYaw);
	isDone = true;
}

bool SetOffset::IsFinished() {
	return isDone;
}

void SetOffset::End() {

}

void SetOffset::Interrupted() {

}
