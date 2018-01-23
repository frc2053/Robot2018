#ifndef SwerveSubsystem_H
#define SwerveSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>


class SwerveSubsystem : public Subsystem {
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
	int AbsMod(int value, int ticks);

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
	//	int FL_CAL = 817;
	//	int BL_CAL = 911;
	//	int FR_CAL = 2685;
	//	int BR_CAL = 230;
	int FL_CAL = 2756;
	int BL_CAL = 2912;
	int FR_CAL = 3881;
	int BR_CAL = 258;
	int TICKS_PER_REV = 4096;
};

#endif
