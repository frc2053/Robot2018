#include "TigerDrive.h"

TigerDrive::TigerDrive(AHRS* imuP)
{
	tooFarCW = false;
	tooFarCCW = false;
	isRotDone = false;
	timesThroughLoop = 0;
	imu.reset(imuP);
	rotateController.reset(new frc::PIDController(K_P, K_I, K_D, K_F, imu.get(), this));
	rotateController->SetInputRange(-180.0, 180.0);
	rotateController->SetOutputRange(-1.0, 1.0);
	rotateController->SetAbsoluteTolerance(ANGLE_TOLERANCE);
	rotateController->SetContinuous(true);
	rotateToAngleRate = 1;
	controllerOverride = false;
	yawOffset = 0;
}

TigerDrive::~TigerDrive() {

}

double TigerDrive::CalculateRotationValue(double angleToRotateTo, double speedMultiplier) {
	int spinDir = CalculateSpinDirection(angleToRotateTo, imu->GetYaw());
	double speed = 0;
	if(!controllerOverride) {
		speed = CalculateSpeedAndOvershoot(spinDir, speedMultiplier);
	}
	else {
		speed = 0;
	}
	return speed;
}

double TigerDrive::CalculateSpinDirection(double targetAngle, double imuYaw) {
	double degreesToAngle = 0;
	int spinDirection = 0;
	if((fabs(imuYaw - targetAngle) > ANGLE_TOLERANCE) &&
	   (fabs(imuYaw + 360 - targetAngle) > ANGLE_TOLERANCE)) {
		if (imuYaw > targetAngle)  {
			tooFarCW = true;
			spinDirection = -1;
			degreesToAngle = imuYaw - targetAngle;
		}
		else {
			tooFarCCW = true;
			spinDirection = 1;
			degreesToAngle = targetAngle - imuYaw;
		}
	}
	if(degreesToAngle > 180)
	{
		degreesToAngle = 360 - degreesToAngle;
		spinDirection = spinDirection * -1;
	}
	return spinDirection;
}

double TigerDrive::CalculateSpeedAndOvershoot(int spinDir, double speedMulti) {
	double calculatedRotate = 0;
	if(tooFarCW || tooFarCCW)
	{
		rotateController->Enable();
		isRotDone = false;
		calculatedRotate = rotateToAngleRate * speedMulti;
		timesThroughLoop = 1;
	}
	else
	{
		if(timesThroughLoop == OVERSHOOT_TIMEOUT || timesThroughLoop == 0)
		{
			rotateController->Disable();
			isRotDone = true;
			timesThroughLoop = 0;
			calculatedRotate = 0;
		}
		timesThroughLoop = timesThroughLoop + 1;
	}
	return calculatedRotate;
}

double TigerDrive::GetAdjYaw() {
	double imuRaw = imu->GetYaw();
	double calculatedOffset = imuRaw + yawOffset;
	if(calculatedOffset >= 180)
	{
		calculatedOffset = calculatedOffset - 360;
	}
	return calculatedOffset;
}

void TigerDrive::SetAdjYaw(double offset)
{
	yawOffset = offset;
}

void TigerDrive::SetIsRotDone(bool isDone)
{
	isRotDone = isDone;
}

void TigerDrive::SetIsRotDoneOverride(bool isDone)
{
	controllerOverride = isDone;
}

void TigerDrive::SetTimesThroughLoop(int timeLoop)
{
	timesThroughLoop = timeLoop;
}

float TigerDrive::GetImuYaw()
{
	return imu->GetYaw();
}

bool TigerDrive::GetIsRotDone()
{
	return isRotDone;
}

bool TigerDrive::GetIsRotDoneOverride()
{
	return controllerOverride;
}
