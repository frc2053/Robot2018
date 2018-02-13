#ifndef ZeroWheels_H
#define ZeroWheels_H

#include <Commands/Command.h>
#include <Robot.h>
#include <Subsystems/SwerveSubsystem.h>

class ZeroWheels: public Command {
public:
	ZeroWheels();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
};

#endif
