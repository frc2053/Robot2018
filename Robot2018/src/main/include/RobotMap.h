#pragma once

#include "frc/WPILib.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "AHRS.h"
#include "Utilities/TigerSwerve/TigerSwerve.h"
#include "Utilities/TigerDrive/TigerDrive.h"
#include <iostream>
#include <frc/livewindow/LiveWindow.h>

using namespace ctre::phoenix::motorcontrol;

class RobotMap {
public:
	static void init();
	static void resetTalons(std::vector<std::shared_ptr<can::TalonSRX>>);

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRDriveTalon;

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRRotTalon;

	static std::shared_ptr<AHRS> robotIMU;
	static std::shared_ptr<TigerDrive> tigerDrive;
	static std::vector<std::shared_ptr<can::TalonSRX>> talonVector;
	static std::vector<std::shared_ptr<can::TalonSRX>> allTalons;
	static std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel;
	static std::shared_ptr<TigerSwerve> tigerSwerve;


	static int TOP_POSITION_TICKS;
	static double GROUND_POS_FT;
	static double SWITCH_POS_FT;
	static double SCALE_POS_FT;
	static double CLIMBBAR_POS_FT;
	static double WHEELBASE_WIDTH;
	static double WHEELBASE_LENGTH;
	static double TIMESTEP;
	static double MAX_VEL;
	static double MAX_ACCEL;
	static double MAX_JERK;
	static int TICKS_PER_REV;
	static int TICKS_PER_FOOT_DRIVE;
	static double WHEEL_CIRCUMFERENCE;
	static double K_P;
	static double K_I;
	static double K_D;
	static double K_V;
	static double K_A;
	static double K_T;
};