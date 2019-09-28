#pragma once

#include "frc/WPILib.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include <rev/CANSparkMax.h>
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

	static std::shared_ptr<rev::CANSparkMax> swerveSubsystemFLDriveSpark;
	static std::shared_ptr<rev::CANSparkMax> swerveSubsystemFRDriveSpark;
	static std::shared_ptr<rev::CANSparkMax> swerveSubsystemBLDriveSpark;
	static std::shared_ptr<rev::CANSparkMax> swerveSubsystemBRDriveSpark;

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRRotTalon;

	static std::shared_ptr<AHRS> robotIMU;
	static std::shared_ptr<TigerDrive> tigerDrive;
	static std::vector<std::shared_ptr<can::TalonSRX>> talonVector;
	static std::vector<std::shared_ptr<rev::CANSparkMax>> sparkVector;
	static std::vector<std::shared_ptr<can::TalonSRX>> allTalons;
	static std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel;
	static std::shared_ptr<TigerSwerve> tigerSwerve;


	static double WHEELBASE_WIDTH;
	static double WHEELBASE_LENGTH;
};