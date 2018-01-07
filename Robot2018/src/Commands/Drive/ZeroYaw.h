#pragma once

#include <Commands/Command.h>
#include <Robot.h>

class ZeroYaw: public Command
{
public:
	ZeroYaw();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
};
