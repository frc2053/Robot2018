#ifndef SRC_UTIL_TIGERJOYSTICK_TIGERLEFTTRIGGER_H_
#define SRC_UTIL_TIGERJOYSTICK_TIGERLEFTTRIGGER_H_

#include "WPILib.h"

class TigerLeftTrigger : public frc::Trigger {
public:
	TigerLeftTrigger(frc::Joystick* joy, int axis);
	bool Get();
	double GetTriggerValue();
private:
	double Deadband(double axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};

#endif
