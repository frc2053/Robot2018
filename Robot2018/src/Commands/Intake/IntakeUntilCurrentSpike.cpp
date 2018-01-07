#include "IntakeUntilCurrentSpike.h"

IntakeUntilCurrentSpike::IntakeUntilCurrentSpike(double time, double speed) {
	Requires(Robot::intakeSubsystem.get());
	timer.reset(new Timer());
	timeCurrent = 0;
	currentThreshold = .5;
	isDone = false;
	inputSpeed = speed;
	timeTarget = time;
	timer->Reset();
	timer->Start();
	currentLeft = 0;
	currentRight = 0;
}

void IntakeUntilCurrentSpike::Initialize() {
	isDone = false;
	timeCurrent = 0;
	timer->Reset();
	timer->Start();
}

void IntakeUntilCurrentSpike::Execute() {
	currentLeft = Robot::intakeSubsystem->GetLeftCurrent();
	currentRight = Robot::intakeSubsystem->GetRightCurrent();
	if(inputSpeed == 0) {
		Robot::intakeSubsystem->RunBothMotors(0);
		isDone = true;
	}
	else {
		timeCurrent = timer->Get();
		if(timeTarget == 0) {
			Robot::intakeSubsystem->RunBothMotors(inputSpeed);
			isDone = false;
		}
		else {
			if(timeCurrent >= timeTarget) {
				Robot::intakeSubsystem->RunBothMotors(0);
				isDone = true;
			}
			else {
				Robot::intakeSubsystem->RunBothMotors(inputSpeed);
				isDone = false;
			}
		}
	}

	if(currentLeft >= 1 && currentRight >= 1) {
		isDone = true;
		Robot::intakeSubsystem->RunBothMotors(0);
	}
}

bool IntakeUntilCurrentSpike::IsFinished() {
	return false;
}

void IntakeUntilCurrentSpike::End() {

}

void IntakeUntilCurrentSpike::Interrupted() {

}
