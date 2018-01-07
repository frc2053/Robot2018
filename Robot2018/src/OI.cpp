#include "OI.h"

#include "Commands/Drive/ZeroYaw.h"
#include "Commands/Test/WheelModuleTest.h"

OI::OI() {

	driverJoystick.reset(new TigerJoystick(0));

	driverJoystick->aButton->WhenPressed(new WheelModuleTest(deg2rad(0)));
	driverJoystick->bButton->WhenPressed(new WheelModuleTest(deg2rad(45)));
	driverJoystick->xButton->WhenPressed(new WheelModuleTest(deg2rad(90)));
	driverJoystick->yButton->WhenPressed(new WheelModuleTest(deg2rad(135)));
	driverJoystick->startButton->WhenPressed(new WheelModuleTest(deg2rad(180)));
	driverJoystick->selectButton->WhenPressed(new WheelModuleTest(deg2rad(225)));
	driverJoystick->leftShoulderButton->WhenPressed(new WheelModuleTest(deg2rad(270)));
	driverJoystick->rightShoulderButton->WhenPressed(new WheelModuleTest(deg2rad(315)));
	driverJoystick->leftStickButton->WhenPressed(new WheelModuleTest(deg2rad(360)));

	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());
	SmartDashboard::PutData("Wheel Module Test", new WheelModuleTest(0));
}

std::shared_ptr<TigerJoystick> OI::GetDriverJoystick() {
	return driverJoystick;
}

double OI::deg2rad(double deg) {
	return deg * (M_PI / 180);
}
