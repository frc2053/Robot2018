#include "RunClimberMotor.h"

RunClimberMotor::RunClimberMotor(double time, double speed) {
	Requires(Robot::climberSubsystem.get());
	timer.reset(new Timer());
	timeCurrent = 0;
	isDone = false;
	inputSpeed = speed;
	timeTarget = time;
	timer->Reset();
	timer->Start();
}

void RunClimberMotor::Initialize() {
	isDone = false;
	timeCurrent = 0;
	timer->Reset();
	timer->Start();
	std::cout << "MADE IT TO RUNCLIMBERMOTOR INIT" << std::endl;
	Robot::climberSubsystem->SwitchToClimberMode();

	//Robot::climberSubsystem->HookLatch();
	//Robot::climberSubsystem->ReleaseWings();


}

void RunClimberMotor::Execute() {


	if(inputSpeed == 0) {
		//Robot::climberSubsystem->SetPrimaryMotor(0);
		Robot::climberSubsystem->SwitchToClimberMode();

		isDone = true;
	}
	else {
		timeCurrent = timer->Get();
		if(timeTarget == 0) {
			//Robot::climberSubsystem->SetPrimaryMotor(inputSpeed);
			isDone = false;
		}
		else {
			if(timeCurrent >= timeTarget) {
				//Robot::climberSubsystem->SetPrimaryMotor(0);
				isDone = true;
			}
			else {
				//Robot::climberSubsystem->SetPrimaryMotor(inputSpeed);
				isDone = false;
			}
		}
	}
}

bool RunClimberMotor::IsFinished() {
	return isDone;
}

void RunClimberMotor::End() {

}

void RunClimberMotor::Interrupted() {

}
