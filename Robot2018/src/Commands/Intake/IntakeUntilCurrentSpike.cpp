#include "IntakeUntilCurrentSpike.h"

IntakeUntilCurrentSpike::IntakeUntilCurrentSpike() {
	Requires(Robot::intakeSubsystem.get());
	currentThreshold = .5;
}

void IntakeUntilCurrentSpike::Initialize() {

}

void IntakeUntilCurrentSpike::Execute() {

}

bool IntakeUntilCurrentSpike::IsFinished() {
	return false;
}

void IntakeUntilCurrentSpike::End() {

}

void IntakeUntilCurrentSpike::Interrupted() {

}
