#pragma once

#include "frc/WPILib.h"
#include "Utilities/TigerJoystick/TigerJoystick.h"
#include <frc/smartdashboard/SmartDashboard.h>

class OI {
private:
	std::shared_ptr<TigerJoystick> driverJoystick;
	std::shared_ptr<TigerJoystick> operatorJoystick;
public:
	OI();
	std::shared_ptr<TigerJoystick> GetDriverJoystick();
	std::shared_ptr<TigerJoystick> GetOperatorJoystick();
};
