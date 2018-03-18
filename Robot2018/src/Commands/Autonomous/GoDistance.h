#pragma once

#include <Commands/Command.h>

#include "../../Util/TigerSwerve/SwerveModule.h"
#include "Timer.h"

class GoDistance : public frc::Command {
public:
	GoDistance(double Xdistance, double Ydistance);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	int ConvertFeetToTicks(double feet);
private:
	bool isDone;
	double _xDistance, _yDistance;
	double deltaDistance;
	double angleForWheel;
	int ticks;
	bool started;
	std::shared_ptr<std::vector<SwerveModule>> modules;
};

