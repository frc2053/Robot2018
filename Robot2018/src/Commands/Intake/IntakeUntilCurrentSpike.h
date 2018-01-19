#ifndef IntakeUntilCurrentSpike_H
#define IntakeUntilCurrentSpike_H

#include "Commands/Command.h"
#include "../../Robot.h"

class IntakeUntilCurrentSpike : public Command {
public:
	IntakeUntilCurrentSpike(double speed = 0, double time = 0);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double currentThreshold;
	double currentLeft;
	double currentRight;
	double timeTarget;
	double timeCurrent;
	double inputSpeed;
	bool isDone;
	std::shared_ptr<frc::Timer> timer;
};

#endif
