#pragma once

#include <frc/commands/Command.h>
#include "Robot.h"

class SetOffset : public Command {
public:
	SetOffset(float input);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	double inputYaw;
};