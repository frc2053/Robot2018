#ifndef DriveCommandAuto_H
#define DriveCommandAuto_H

#include "Commands/Command.h"
#include "../../Robot.h"

class DriveCommandAuto: public Command
{
public:
	DriveCommandAuto(float side, float fow, float rot, float time, float angle);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	float inputSide;
	float inputFow;
	float inputRot;
	float inputAngle;
	float timeCurrent;
	float timeTarget;
	bool isDone;
	float finalAutoRot;
	float adjustedYaw;
	bool isRotDone;
	std::shared_ptr<Timer> timer;
};

#endif
