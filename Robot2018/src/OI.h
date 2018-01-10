#pragma once

#include "WPILib.h"
#include "Util/TigerJoystick/TigerJoystick.h"
#include <SmartDashboard/SmartDashboard.h>

class OI {
private:
	std::shared_ptr<TigerJoystick> driverJoystick;
	std::shared_ptr<TigerJoystick> operatorJoystick;
public:
	OI();
	std::shared_ptr<TigerJoystick> GetDriverJoystick();
	std::shared_ptr<TigerJoystick> GetOperatorJoystick();
};
