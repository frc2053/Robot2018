#pragma once

#include "WPILib.h"

class TigerLeftTrigger : public Trigger {
public:
	TigerLeftTrigger(frc::Joystick* joy, int axis);
	bool Get();
	double GetTriggerValue();
private:
	double Deadband(float axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};
