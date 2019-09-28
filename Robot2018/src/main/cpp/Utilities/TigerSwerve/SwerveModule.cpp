#include "Utilities/TigerSwerve/SwerveModule.h"
#include <iostream>
#include "Robot.h"
#define _USE_MATH_DEFINES
#include <math.h>

SwerveModule::SwerveModule(std::shared_ptr<rev::CANSparkMax> driveController, std::shared_ptr<can::TalonSRX> rotateController) {
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

void SwerveModule::SetAngle(Rotation2D angle, bool doOptimization) {
	// is there a way to print out which module number this is from here???
	//std::cout << "InputSetAngle: " << angle.getDegrees() << std::endl;

	Rotation2D currentAngle = _angleEncoder->GetAngle();
	//std::cout << "CurrentAngle: " << currentAngle.getDegrees() << std::endl;

	Rotation2D deltaAngle = currentAngle.rotateBy(angle.inverse());
	//std::cout << "DeltaAngle: " << deltaAngle.getDegrees() << std::endl;
	deltaAngle = Rotation2D::fromDegrees(fabs(deltaAngle.getDegrees()));
	//std::cout << "Abs DeltaAngle: " << deltaAngle.getDegrees() << std::endl;

	if(doOptimization) {
		if(deltaAngle.getRadians() > M_PI_2 && deltaAngle.getRadians() <= 3 * M_PI_2) {
			//std::cout << "Set Point The Long Way Around" << std::endl;

			angle = angle.rotateBy(Rotation2D::fromRadians(M_PI));
			//std::cout << "New (Optimzed) Set Point " << angle.getDegrees() << std::endl;

			isOptimizedAngle = true;
		}

		else {
			//std::cout << "Set Point The Short Way Around" << std::endl;
			//std::cout << "New Set Point " << angle.getDegrees() << std::endl;
			isOptimizedAngle = false;
		}
	}

	int setpoint = _angleEncoder->ConvertAngleToSetpoint(angle);
	//std::cout << "setpoint: " << setpoint << std::endl;
	_rotateController->Set(ControlMode::Position, setpoint);

	//std::cout << "Final Calculated Set Point: " << setpoint << std::endl;
	//std::cout << "END Set Angle" << std::endl;
}

void SwerveModule::Stop() {
	_driveController->Set(0);
}

void SwerveModule::Set(double speed, Rotation2D angle, bool doOptimization) {
	SetAngle(angle, doOptimization);
	if(isOptimizedAngle) {
		_driveController->Set(speed*-1);
	}
	else {
		_driveController->Set(speed);
	}
}
