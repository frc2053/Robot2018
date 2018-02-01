#ifndef DeployWings_H
#define DeployWings_H

#include "Commands/Command.h"
#include "../../Robot.h"
class DeployWings : public frc::Command {
public:
	DeployWings();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
};

#endif  // DeployWings_H
