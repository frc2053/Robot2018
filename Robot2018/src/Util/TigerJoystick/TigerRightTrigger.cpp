#include "TigerRightTrigger.h"

TigerRightTrigger::TigerRightTrigger(frc::Joystick* joy, int axis) {
	joystick = joy;
	joystickAxis = axis;
	joystickValue = 0;
}

bool TigerRightTrigger::Get() {
	joystickValue = TigerRightTrigger::Deadband(joystick->GetRawAxis(joystickAxis));
	if(joystickValue > 0) {
		return true;
	}
	else {
		return false;
	}
}

float TigerRightTrigger::Deadband(float axis) {
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

float TigerRightTrigger::GetTriggerValue() {
	joystickValue = TigerRightTrigger::Deadband(joystick->GetRawAxis(joystickAxis));
	return joystickValue;
}
