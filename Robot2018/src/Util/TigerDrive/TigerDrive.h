#ifndef SRC_UTIL_TIGERDRIVE_TIGERDRIVE_H_
#define SRC_UTIL_TIGERDRIVE_TIGERDRIVE_H_

#include "AHRS.h"
#include "WPILib.h"

class TigerDrive : public frc::PIDOutput
{
private:
	const double K_P = 0.05;
	const double K_I = 0.0;
	const double K_D = 0.035f;
	const double K_F = 0.0;
	const double ANGLE_TOLERANCE = 2;
	const int OVERSHOOT_TIMEOUT = 5;

	bool tooFarCW;
	bool tooFarCCW;
	double rotateToAngleRate;
	double yawOffset;
	int timesThroughLoop;
	bool isRotDone;
	bool controllerOverride;
public:
	TigerDrive(AHRS* imuP);
	virtual ~TigerDrive();
	double CalculateRotationValue(double angleToRotateTo, double speedMultiplier);
	double CalculateSpinDirection(double targetAngle, double imuAngle);
	double CalculateSpeedAndOvershoot(int spinDir, double speedMulti);
	double GetAdjYaw();
	void SetAdjYaw(double offset);
	float GetImuYaw();
	bool GetIsRotDone();
	void SetIsRotDone(bool isDone);
	void SetIsRotDoneOverride(bool isDone);
	void SetTimesThroughLoop(int timeLoop);
	bool GetIsRotDoneOverride();

	std::shared_ptr<frc::PIDController> rotateController;
	std::shared_ptr<AHRS> imu;

	void PIDWrite(double output) {
	    rotateToAngleRate = output;
	}
};

#endif
