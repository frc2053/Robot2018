#include <Util/TigerJoystick/TigerJoystick.h>

TigerJoystick::TigerJoystick(int port) {
	joystick.reset(new frc::Joystick(port));
	aButton.reset(new frc::JoystickButton(joystick.get(), 1));
	bButton.reset(new frc::JoystickButton(joystick.get(), 2));
	xButton.reset(new frc::JoystickButton(joystick.get(), 3));
	yButton.reset(new frc::JoystickButton(joystick.get(), 4));
	leftShoulderButton.reset(new frc::JoystickButton(joystick.get(), 5));
	rightShoulderButton.reset(new frc::JoystickButton(joystick.get(), 6));
	selectButton.reset(new frc::JoystickButton(joystick.get(), 7));
	startButton.reset(new frc::JoystickButton(joystick.get(), 8));
	leftStickButton.reset(new frc::JoystickButton(joystick.get(), 9));
	rightStickButton.reset(new frc::JoystickButton(joystick.get(), 10));
	leftTrigger.reset(new TigerLeftTrigger(joystick.get(), 3));
	rightTrigger.reset(new TigerRightTrigger(joystick.get(), 3));
}

TigerJoystick::~TigerJoystick() {
}

double TigerJoystick::GetLeftXAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(0));
}

double TigerJoystick::GetLeftYAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(1));
}

double TigerJoystick::GetRightXAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(4));
}

double TigerJoystick::GetRightYAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(5));
}

double TigerJoystick::GetLeftTriggerValue() {
	return leftTrigger->GetTriggerValue();
}

double TigerJoystick::GetRightTriggerValue() {
	return rightTrigger->GetTriggerValue();
}

bool TigerJoystick::GetLeftTriggerPressed() {
	return leftTrigger->Get();
}

bool TigerJoystick::GetRightTriggerPressed() {
	return rightTrigger->Get();
}

TigerLeftTrigger* TigerJoystick::GetLeftTrigger() {
	return leftTrigger.get();
}

TigerRightTrigger* TigerJoystick::GetRightTrigger() {
	return rightTrigger.get();
}

double TigerJoystick::DeadBandJoystick(double axis) {
	if(axis > -0.20 && axis < 0.20) {
		axis = 0;
	}
	else {
		axis = axis * fabs(axis);
	}
	return axis;
}
