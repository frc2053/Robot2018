#include "OI.h"

#include "Commands/Drive/ZeroWheels.h"
#include "Commands/Test/CWTest.h"
#include "Commands/Drive/ZeroYaw.h"

OI::OI() {

	driverJoystick.reset(new TigerJoystick(0));
	operatorJoystick.reset(new TigerJoystick(1));

	SmartDashboard::PutData("CW Test", new CWTest());
	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());
}

std::shared_ptr<TigerJoystick> OI::GetDriverJoystick() {
	return driverJoystick;
}

std::shared_ptr<TigerJoystick> OI::GetOperatorJoystick() {
	return operatorJoystick;
}
