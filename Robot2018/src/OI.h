#pragma once

#include "WPILib.h"
#include "Util/TigerJoystick/TigerJoystick.h"
#include <SmartDashboard/SmartDashboard.h>

class OI {
private:
	std::shared_ptr<TigerJoystick> driverJoystick;
	double deg2rad(double deg);
public:
	OI();
	std::shared_ptr<TigerJoystick> GetDriverJoystick();
};
