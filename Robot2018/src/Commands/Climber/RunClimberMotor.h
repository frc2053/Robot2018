#ifndef RunClimberMotor_H
#define RunClimberMotor_H

#include "Commands/Command.h"
#include "../../Robot.h"

class RunClimberMotor : public frc::Command {
public:
	RunClimberMotor(double speed = 0, double time = 0);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double timeTarget;
	double timeCurrent;
	double inputSpeed;
	bool isDone;
	std::shared_ptr<frc::Timer> timer;
};

#endif
