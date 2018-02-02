#ifndef DeployWings_H
#define DeployWings_H

#include "Commands/Command.h"
<<<<<<< HEAD
#include "../../Robot.h"
=======
>>>>>>> ce3394613c23ad1596312f251a3e1d46d227aa44

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

<<<<<<< HEAD
#endif  // DeployWings_H
=======
#endif
>>>>>>> ce3394613c23ad1596312f251a3e1d46d227aa44
