#ifndef WheelModuleTest_H
#define WheelModuleTest_H

#include "../../Robot.h"
#include <chrono>
#include <thread>

class WheelModuleTest : public Command {
public:
	WheelModuleTest(double setPoint);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool testsPassed;
	double talonSetpoint;
};

#endif
