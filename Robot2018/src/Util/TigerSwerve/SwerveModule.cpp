#include "SwerveModule.h"
#include <iostream>
#include "../../Robot.h"

SwerveModule::SwerveModule(std::shared_ptr<can::TalonSRX> driveController, std::shared_ptr<can::TalonSRX> rotateController) {
	_driveController.reset(driveController.get());
	_rotateController.reset(rotateController.get());
	_angleEncoder.reset(new CTREMagEncoder(_rotateController.get()));
	isOptimizedAngle = false;
}

SwerveModule::~SwerveModule() {

}

Rotation2D SwerveModule::GetAngle() const {
	return _angleEncoder->GetAngle();
}

void SwerveModule::SetAngle(Rotation2D angle) {
	Rotation2D currentAngle = _angleEncoder->GetAngle();
	Rotation2D deltaAngle = currentAngle.rotateBy(angle.inverse());
	if(deltaAngle.getRadians() > M_PI_2 && deltaAngle.getRadians() < 3 * M_PI_2) {
		angle = angle.rotateBy(Rotation2D::fromRadians(M_PI));
		isOptimizedAngle = true;
	}
	else {
		isOptimizedAngle = false;
	}
	int setpoint = _angleEncoder->ConvertAngleToSetpoint(angle);
	_rotateController->Set(ControlMode::Position, setpoint / 4096.0);
}

void SwerveModule::Stop() {
	_driveController->Set(ControlMode::PercentOutput, 0);
}

void SwerveModule::Set(double speed, Rotation2D angle) {
	SetAngle(angle);
	if(isOptimizedAngle) {
		_driveController->Set(ControlMode::PercentOutput, speed*-1);
	}
	else {
		_driveController->Set(ControlMode::PercentOutput, speed);
	}
}
