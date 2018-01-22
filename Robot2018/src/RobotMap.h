#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "WPILib.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "AHRS.h"
#include "Util/TigerSwerve/TigerSwerve.h"
#include "Util/TigerDrive/TigerDrive.h"
#include <iostream>
#include "pathfinder.h"



using namespace ctre::phoenix::motorcontrol;

class RobotMap {
public:
	static void init();

	static std::shared_ptr<can::TalonSRX> intakeSubsystemLeftMotor;
	static std::shared_ptr<can::TalonSRX> intakeSubsystemRightMotor;

	static std::shared_ptr<can::TalonSRX> elevatorClimberSubsystemPrimaryTalon;
	static std::shared_ptr<can::TalonSRX> elevatorClimberSubsystemFollower01Talon;
	static std::shared_ptr<can::TalonSRX> elevatorClimberSubsystemFollower02Talon;

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRDriveTalon;

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRRotTalon;

	static std::shared_ptr<frc::DoubleSolenoid> climberSubsystemWingSolenoid;
	static std::shared_ptr<frc::DoubleSolenoid> climberSubsystemLatchSolenoid;
	static std::shared_ptr<frc::DoubleSolenoid> elevatorClimberSubsystemShifterSolenoid;

	static std::shared_ptr<AHRS> robotIMU;
	static std::shared_ptr<TigerDrive> tigerDrive;
	static std::vector<std::shared_ptr<can::TalonSRX>> talonVector;
	static std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel;
	static std::shared_ptr<TigerSwerve> tigerSwerve;

	static int TOP_POSITION_TICKS;
	static double GROUND_POS_FT;
	static double SWITCH_POS_FT;
	static double SCALE_POS_FT;
	static double WHEELBASE_WIDTH;
	static double WHEELBASE_LENGTH;
};

#endif
