#include "HookLatch.h"
#include "../../Robot.h"

HookLatch::HookLatch(bool direction) {
	Requires(Robot::climberSubsystem.get());
	isDone = false;
	currentDirection = direction;
}

void HookLatch::Initialize() {
	isDone = false;
}

void HookLatch::Execute() {
	if(currentDirection == 1) {
		Robot::climberSubsystem->HookLatch();
	}
	if(currentDirection == 0) {
		Robot::climberSubsystem->UnhookLatch();
	}
	isDone = true;
}

bool HookLatch::IsFinished() {
	return isDone;
}

void HookLatch::End() {

}

void HookLatch::Interrupted() {

}
