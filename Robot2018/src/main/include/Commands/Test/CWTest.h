#pragma once

#include "frc/WPILib.h"
#include "Robot.h"
#include <chrono>
#include <thread>

class CWTest : public frc::Command {
public:
	CWTest();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool testsPassed;
};