#pragma once

#include "WPILib.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "AHRS.h"
#include "Util/TigerSwerve/TigerSwerve.h"
#include "Util/TigerDrive/TigerDrive.h"
#include <iostream>


using namespace ctre::phoenix::motorcontrol;

class RobotMap {
public:
	static void init();

	static std::shared_ptr<can::TalonSRX> intakeSubsystemLeftMotor;
	static std::shared_ptr<can::TalonSRX> intakeSubsystemRightMotor;

	static std::shared_ptr<can::TalonSRX> elevatorSubsystemPrimaryMotor;
	static std::shared_ptr<can::TalonSRX> elevatorSubsystemFollowerMotor;

	static std::shared_ptr<can::TalonSRX> climberSubsystemPrimaryMotor;
	static std::shared_ptr<can::TalonSRX> climberSubsystemFollowerMotor;

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
	static std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel;
	static std::shared_ptr<TigerSwerve> tigerSwerve;

	static int TOP_POSITION_TICKS;
	static double GROUND_POS_IN;
	static double SWITCH_POS_IN;
	static double SCALE_POS_IN;
};