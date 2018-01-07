#pragma once

#include <Util/Math/Rotation2D.h>
#include <Util/Math/Translation2D.h>
#include "math.h"
#include "ctre/Phoenix.h"
#include "SwerveModule.h"
#include <vector>
#include <iostream>


using namespace ctre::phoenix::motorcontrol;

class TigerSwerve {
private:
	static constexpr double BASE_LENGTH = 20;
	static constexpr double BASE_WIDTH = 20;

	double xAxis = 0, yAxis = 0, rotAxis = 0, currentYaw = 0;

	std::shared_ptr<Rotation2D> centerOfRotation;

	std::shared_ptr<can::TalonSRX> frontRightDrive;
	std::shared_ptr<can::TalonSRX> frontLeftDrive;
	std::shared_ptr<can::TalonSRX> backRightDrive;
	std::shared_ptr<can::TalonSRX> backLeftDrive;

	std::shared_ptr<can::TalonSRX> frontRightRot;
	std::shared_ptr<can::TalonSRX> frontLeftRot;
	std::shared_ptr<can::TalonSRX> backRightRot;
	std::shared_ptr<can::TalonSRX> backLeftRot;

	std::shared_ptr<std::vector<SwerveModule>> modules;

	void Drive(double xSpeed, double ySpeed, double rotSpeed, double headingOffset);
	double deg2rad(double deg);
public:
	TigerSwerve(std::vector<std::shared_ptr<can::TalonSRX>>& talons);
	virtual ~TigerSwerve();
	void SetCenterOfRotation(double x, double y);
	void SetBrakeMode();
	void DriveRobotOriented(double x, double y, double rotation);
	void DriveFieldOriented(double x, double y, double rotation, double gyro);
	std::shared_ptr<std::vector<SwerveModule>> GetModules();
	static void SwerveInverseKinematics(Translation2D &translation,
				double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
				Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR);
};
