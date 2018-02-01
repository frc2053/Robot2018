#ifndef DeployWings_H
#define DeployWings_H

#include "Commands/Command.h"

class DeployWings : public frc::Command {
public:
	DeployWings(bool direction);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	bool currentDirection;
};

#endif
