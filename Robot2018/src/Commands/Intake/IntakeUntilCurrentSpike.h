#pragma once

#include "Commands/Command.h"
#include "../../Robot.h"

class IntakeUntilCurrentSpike : public Command {
public:
	IntakeUntilCurrentSpike();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double currentThreshold;
};
