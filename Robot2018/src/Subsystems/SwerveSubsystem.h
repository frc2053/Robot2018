#ifndef SwerveSubsystem_H
#define SwerveSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>


class SwerveSubsystem : public frc::Subsystem {
public:
	SwerveSubsystem();
	void InitDefaultCommand();
	double CalculateRotValue(double setAngle, double speedMulti);
	bool GetIsRotDone();
	double GetAdjYaw();
	void SetAdjYaw(double yaw);
	void SwerveDrive(double xAxis, double yAxis, double rotAxis, double currentYaw);
	void SetIsRotDone(bool isDone);
	void SetIsRotDoneOverride(bool isDone);
	void SetTimesThroughLoop(int timeLoop);
	bool GetIsRotDoneOverride();
	void ZeroYaw();
	void CalibrateWheels();
	void CalibrateWheelsSimple();
	int AbsMod(int value, int ticks);
	int OptimizeRot(int value, int ticks);

	std::shared_ptr<TigerSwerve> tigerSwerve;
	std::shared_ptr<TigerDrive> tigerDrive;
	std::shared_ptr<TigerSwerve> GetSwerveStuff();
private:
	std::shared_ptr<can::TalonSRX> frontRightDriveTalon;
	std::shared_ptr<can::TalonSRX> frontLeftDriveTalon;
	std::shared_ptr<can::TalonSRX> backRightDriveTalon;
	std::shared_ptr<can::TalonSRX> backLeftDriveTalon;

	std::shared_ptr<can::TalonSRX> frontRightRotationTalon;
	std::shared_ptr<can::TalonSRX> frontLeftRotationTalon;
	std::shared_ptr<can::TalonSRX> backRightRotationTalon;
	std::shared_ptr<can::TalonSRX> backLeftRotationTalon;
	int FL_CAL = 285;
	int BL_CAL = 2366;
	int FR_CAL = 2229;
	int BR_CAL = 461;


	int TICKS_PER_REV = 4096;
	int absVal = 0;
	int halfTicks = 0;
};

#endif
