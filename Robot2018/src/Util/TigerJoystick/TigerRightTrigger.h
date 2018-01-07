#pragma once

#include "WPILib.h"

class TigerRightTrigger : public Trigger {
public:
	TigerRightTrigger(frc::Joystick* joy, int axis);
	bool Get();
	float GetTriggerValue();
private:
	float Deadband(float axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};
