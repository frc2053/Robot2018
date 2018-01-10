#ifndef GoToElevatorPosition_H
#define GoToElevatorPosition_H

#include "Commands/Command.h"
#include "../../Robot.h"

class GoToElevatorPosition : public frc::Command {
public:
	GoToElevatorPosition(double inputHeight);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double heightTarget;
	bool isDone;
};

#endif
