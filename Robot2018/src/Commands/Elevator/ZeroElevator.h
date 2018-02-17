#pragma once

#include <Commands/Command.h>

class ZeroElevator : public frc::Command {
public:
	ZeroElevator();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	bool isDone;
};

