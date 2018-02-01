#ifndef HookLatch_H
#define HookLatch_H

#include "Commands/Command.h"

class HookLatch : public frc::Command {
public:
	HookLatch(bool direction);
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
