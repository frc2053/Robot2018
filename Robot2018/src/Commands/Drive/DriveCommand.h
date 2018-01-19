#ifndef DriveCommand_H
#define DriveCommand_H

#include "../../Robot.h"

class DriveCommand : public Command {
public:
	DriveCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	void GetInputs();
	void SetAngleFromInput();
	void RotateCommand();
	void CallToSwerveDrive();
	void CheckRotateOverride();
private:
	double xAxis;
	double yAxis;
	double rotAxis;
	double currentYaw;
	double setAngle;
	double finalRotVal;
	bool isAPressed;
	bool isBPressed;
	bool isXPressed;
	bool isYPressed;
	bool isLeftStickPressed;
	bool isRotDone;
};

#endif
