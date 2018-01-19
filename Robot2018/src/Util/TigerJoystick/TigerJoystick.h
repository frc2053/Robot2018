#ifndef SRC_UTIL_TIGERJOYSTICK_TIGERJOYSTICK_H_
#define SRC_UTIL_TIGERJOYSTICK_TIGERJOYSTICK_H_

#include "WPILib.h"
#include "TigerLeftTrigger.h"
#include "TigerRightTrigger.h"

class TigerJoystick {
public:
	TigerJoystick(int port);
	virtual ~TigerJoystick();

	double GetLeftXAxis();
	double GetLeftYAxis();
	double GetRightXAxis();
	double GetRightYAxis();
	double GetLeftTriggerValue();
	double GetRightTriggerValue();
	bool GetLeftTriggerPressed();
	bool GetRightTriggerPressed();
	TigerLeftTrigger* GetLeftTrigger();
	TigerRightTrigger* GetRightTrigger();
	std::unique_ptr<frc::Joystick> joystick;
	std::unique_ptr<frc::JoystickButton> aButton;
	std::unique_ptr<frc::JoystickButton> bButton;
	std::unique_ptr<frc::JoystickButton> xButton;
	std::unique_ptr<frc::JoystickButton> yButton;
	std::unique_ptr<frc::JoystickButton> leftShoulderButton;
	std::unique_ptr<frc::JoystickButton> rightShoulderButton;
	std::unique_ptr<frc::JoystickButton> startButton;
	std::unique_ptr<frc::JoystickButton> selectButton;
	std::unique_ptr<frc::JoystickButton> leftStickButton;
	std::unique_ptr<frc::JoystickButton> rightStickButton;
private:
	double DeadBandJoystick(double axis);
	std::unique_ptr<TigerLeftTrigger> leftTrigger;
	std::unique_ptr<TigerRightTrigger> rightTrigger;
};

#endif
