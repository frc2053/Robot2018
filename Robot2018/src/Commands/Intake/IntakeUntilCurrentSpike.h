#ifndef IntakeUntilCurrentSpike_H
#define IntakeUntilCurrentSpike_H

#include "Commands/Command.h"
#include "../../Robot.h"

class IntakeUntilCurrentSpike : public Command {
public:
	IntakeUntilCurrentSpike(double time = 0, double speed = 0, bool currentCheck = 0);
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
	bool isCheckingForCurrentSpike;
	std::shared_ptr<frc::Timer> timer;
};

#endif
