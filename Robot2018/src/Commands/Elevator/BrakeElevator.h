#ifndef BrakeElevator_H
#define BrakeElevator_H

#include "Commands/Command.h"
#include "../../Robot.h"

class BrakeElevator : public frc::Command {
public:
	BrakeElevator(double Power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double inputPower;
	bool isDone;
};

#endif
