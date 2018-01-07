#include "TigerLeftTrigger.h"

TigerLeftTrigger::TigerLeftTrigger(frc::Joystick* joy, int axis) {
	joystick = joy;
	joystickAxis = axis;
	joystickValue = 0;
}

bool TigerLeftTrigger::Get() {
	joystickValue = TigerLeftTrigger::Deadband(joystick->GetRawAxis(joystickAxis));
	if(joystickValue > 0) {
		return true;
	}
	else {
		return false;
	}
}

double TigerLeftTrigger::Deadband(float axis) {
	if(axis > -0.20 && axis < 0.20)
	{
		axis = 0;
	}
	else
	{
		axis = axis * fabs(axis);
	}
	return axis;
}

double TigerLeftTrigger::GetTriggerValue() {
	joystickValue = TigerLeftTrigger::Deadband(joystick->GetRawAxis(joystickAxis));
	return joystickValue;
}
