#ifndef SRC_UTIL_TIGERJOYSTICK_TIGERRIGHTTRIGGER_H_
#define SRC_UTIL_TIGERJOYSTICK_TIGERRIGHTTRIGGER_H_

#include "WPILib.h"

class TigerRightTrigger : public Trigger {
public:
	TigerRightTrigger(frc::Joystick* joy, int axis);
	bool Get();
	float GetTriggerValue();
private:
	double Deadband(double axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};

#endif
