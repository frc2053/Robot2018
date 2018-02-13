#pragma once

#include <Commands/Command.h>

class ChangeGearbox : public frc::Command {
public:
	ChangeGearbox(bool inputMode);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	bool mode;
};

