#pragma once

#include "WPILib.h"

class TigerLeftTrigger : public frc::JoystickButton {
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
