#include "RobotMap.h"
#include "frc/WPILib.h"

std::shared_ptr<rev::CANSparkMax> RobotMap::swerveSubsystemFLDriveSpark;
//std::shared_ptr<rev::CANSparkMax> RobotMap::swerveSubsystemFRDriveSpark;
//std::shared_ptr<rev::CANSparkMax> RobotMap::swerveSubsystemBLDriveSpark;
//std::shared_ptr<rev::CANSparkMax> RobotMap::swerveSubsystemBRDriveSpark;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLRotTalon;
//std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRRotTalon;
//std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLRotTalon;
//std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRRotTalon;

std::shared_ptr<AHRS> RobotMap::robotIMU;
std::shared_ptr<frc::PowerDistributionPanel> RobotMap::powerDistributionPanel;
std::shared_ptr<TigerDrive> RobotMap::tigerDrive;
std::shared_ptr<TigerSwerve> RobotMap::tigerSwerve;
std::vector<std::shared_ptr<can::TalonSRX>> RobotMap::talonVector;
std::vector<std::shared_ptr<rev::CANSparkMax>> RobotMap::sparkVector;
std::vector<std::shared_ptr<can::TalonSRX>> RobotMap::allTalons;

double RobotMap::WHEELBASE_WIDTH;
double RobotMap::WHEELBASE_LENGTH;

void RobotMap::init() {
	std::cout << "RobotMap is starting!" << std::endl;

	WHEELBASE_LENGTH = 21;
	WHEELBASE_WIDTH = 26.249;

	LiveWindow::GetInstance()->DisableAllTelemetry();

	swerveSubsystemFLDriveSpark.reset(new rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
	//swerveSubsystemFRDriveSpark.reset(new rev::CANSparkMax(3, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
	//swerveSubsystemBLDriveSpark.reset(new rev::CANSparkMax(4, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
	//swerveSubsystemBRDriveSpark.reset(new rev::CANSparkMax(5, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

	swerveSubsystemFLRotTalon.reset(new can::TalonSRX(6));
	//swerveSubsystemFRRotTalon.reset(new can::TalonSRX(7));
	//swerveSubsystemBLRotTalon.reset(new can::TalonSRX(8));
	//swerveSubsystemBRRotTalon.reset(new can::TalonSRX(9));

	allTalons.push_back(swerveSubsystemFLRotTalon);
	//allTalons.push_back(swerveSubsystemFRRotTalon);
	//allTalons.push_back(swerveSubsystemBLRotTalon);
	//allTalons.push_back(swerveSubsystemBRRotTalon);

	swerveSubsystemFLDriveSpark->RestoreFactoryDefaults();
	//swerveSubsystemFRDriveSpark->RestoreFactoryDefaults();
	//swerveSubsystemBLDriveSpark->RestoreFactoryDefaults();
	//swerveSubsystemBRDriveSpark->RestoreFactoryDefaults();

	RobotMap::resetTalons(allTalons);

	sparkVector.push_back(swerveSubsystemFLDriveSpark);
	//sparkVector.push_back(swerveSubsystemFRDriveSpark);
	//sparkVector.push_back(swerveSubsystemBLDriveSpark);
	//sparkVector.push_back(swerveSubsystemBRDriveSpark);

	talonVector.push_back(swerveSubsystemFLRotTalon);
	//talonVector.push_back(swerveSubsystemFRRotTalon);
	//talonVector.push_back(swerveSubsystemBLRotTalon);
	//talonVector.push_back(swerveSubsystemBRRotTalon);

	robotIMU.reset(new AHRS(frc::SPI::Port::kMXP));
	powerDistributionPanel.reset(new frc::PowerDistributionPanel());
	tigerSwerve.reset(new TigerSwerve(RobotMap::talonVector, RobotMap::sparkVector));
	tigerDrive.reset(new TigerDrive(RobotMap::robotIMU.get()));

	//makes the drive talons drive the right way
	//swerveSubsystemFLDriveTalon->SetInverted(false);
	//swerveSubsystemFRDriveTalon->SetInverted(false);
	//swerveSubsystemBLDriveTalon->SetInverted(false);
	//swerveSubsystemBRDriveTalon->SetInverted(false);

	//zeros the quad encoders on start up
	//swerveSubsystemFLDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//swerveSubsystemFRDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//swerveSubsystemBLDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//swerveSubsystemBRDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);

	//make sure we use the quad encoders when we do PID on them
	//swerveSubsystemFLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	//swerveSubsystemFRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	//swerveSubsystemBLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	//swerveSubsystemBRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);

	//for the talon PID to work we need to make sure that the sensors are in phase
	//this means that when we give the motor a positive voltage the encoder ticks MUST go more positive
	//if this is not set correctly PID will not work at all
	//swerveSubsystemFLDriveTalon->SetSensorPhase(true);
	//swerveSubsystemFRDriveTalon->SetSensorPhase(true);
	//swerveSubsystemBLDriveTalon->SetSensorPhase(true);
	//swerveSubsystemBRDriveTalon->SetSensorPhase(true);

	//swerveSubsystemFLDriveTalon->ConfigOpenloopRamp(0.4, 10);
	//swerveSubsystemFRDriveTalon->ConfigOpenloopRamp(0.4, 10);
	//swerveSubsystemBLDriveTalon->ConfigOpenloopRamp(0.4, 10);
	//swerveSubsystemBRDriveTalon->ConfigOpenloopRamp(0.4, 10);



	//we want to use relative for rotation motors because it is much faster read time
	swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	//swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	//swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	//swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	//we set the rotation sensor pos to the pulse width position masked. The masking makes sure we only are from 0 - 4096
	/*swerveSubsystemFLRotTalon->SetSelectedSensorPosition(swerveSubsystemFLRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF, 0, 10);
	swerveSubsystemFRRotTalon->SetSelectedSensorPosition(swerveSubsystemFRRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF, 0, 10);
	swerveSubsystemBLRotTalon->SetSelectedSensorPosition(swerveSubsystemBLRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF, 0, 10);
	swerveSubsystemBRRotTalon->SetSelectedSensorPosition(swerveSubsystemBRRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF, 0, 10);*/

	//Then we zero them out
	//swerveSubsystemFLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	//swerveSubsystemFRRotTalon->SetSelectedSensorPosition(0, 0, 10);
	//swerveSubsystemBLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	//swerveSubsystemBRRotTalon->SetSelectedSensorPosition(0, 0, 10);

	//see drive motor explanation above
	swerveSubsystemFLRotTalon->SetSensorPhase(false);
	//swerveSubsystemFRRotTalon->SetSensorPhase(false);
	//swerveSubsystemBLRotTalon->SetSensorPhase(false);
	//swerveSubsystemBRRotTalon->SetSensorPhase(false);

	//makes sure we are rotating clockwise with positive voltage
	swerveSubsystemFLRotTalon->SetInverted(true);
	//swerveSubsystemFRRotTalon->SetInverted(true);
	//swerveSubsystemBLRotTalon->SetInverted(true);
	//swerveSubsystemBRRotTalon->SetInverted(true);

	//PIDS
	swerveSubsystemFLRotTalon->Config_kP(0, 12, 10);
	swerveSubsystemFLRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemFLRotTalon->Config_kD(0, 120, 10);

	//swerveSubsystemFRRotTalon->Config_kP(0, 12, 10);
	//swerveSubsystemFRRotTalon->Config_kI(0, 0, 10);
	//swerveSubsystemFRRotTalon->Config_kD(0, 120, 10);

	//swerveSubsystemBLRotTalon->Config_kP(0, 12, 10);
	//swerveSubsystemBLRotTalon->Config_kI(0, 0, 10);
	//swerveSubsystemBLRotTalon->Config_kD(0, 120, 10);

	//swerveSubsystemBRRotTalon->Config_kP(0, 12, 10);
	//swerveSubsystemBRRotTalon->Config_kI(0, 0, 10);
	//swerveSubsystemBRRotTalon->Config_kD(0, 120, 10);

	//Rotation motors are too strong so we limit them to around half total output
	swerveSubsystemFLRotTalon->ConfigPeakOutputForward(1.0, 10); //.416
	//swerveSubsystemFRRotTalon->ConfigPeakOutputForward(1.0, 10);
	//swerveSubsystemBLRotTalon->ConfigPeakOutputForward(1.0, 10);
	//swerveSubsystemBRRotTalon->ConfigPeakOutputForward(1.0, 10);

	//same as above except in reverse
	swerveSubsystemFLRotTalon->ConfigPeakOutputReverse(-1.0, 10); //-.416
	//swerveSubsystemFRRotTalon->ConfigPeakOutputReverse(-1.0, 10);
	//swerveSubsystemBLRotTalon->ConfigPeakOutputReverse(-1.0, 10);
	//swerveSubsystemBRRotTalon->ConfigPeakOutputReverse(-1.0, 10);

	//Tolerance for PID
	swerveSubsystemFLRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	//swerveSubsystemFRRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	//swerveSubsystemBLRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	//swerveSubsystemBRRotTalon->ConfigAllowableClosedloopError(0, 5, 10);

	//This prevents the rotation motors from drawing too much current
	swerveSubsystemFLRotTalon->ConfigContinuousCurrentLimit(10, 10);
	//swerveSubsystemFRRotTalon->ConfigContinuousCurrentLimit(10, 10);
	//swerveSubsystemBLRotTalon->ConfigContinuousCurrentLimit(10, 10);
	//swerveSubsystemBRRotTalon->ConfigContinuousCurrentLimit(10, 10);

	swerveSubsystemFLRotTalon->ConfigPeakCurrentLimit(0, 10);
	//swerveSubsystemFRRotTalon->ConfigPeakCurrentLimit(0, 10);
	//swerveSubsystemBLRotTalon->ConfigPeakCurrentLimit(0, 10);
	//swerveSubsystemBRRotTalon->ConfigPeakCurrentLimit(0, 10);
}

void RobotMap::resetTalons(std::vector<std::shared_ptr<can::TalonSRX>> allTalons) {
	for(int i = 0; i < allTalons.size(); i++) {
		std::shared_ptr<can::TalonSRX> currentTalon = allTalons.at(i);
		currentTalon->SetInverted(false);
		currentTalon->Set(ControlMode::PercentOutput, 0);
		currentTalon->ConfigOpenloopRamp(0, 0);
		currentTalon->ConfigClosedloopRamp(0, 0);
		currentTalon->ConfigNominalOutputForward(0, 0);
		currentTalon->ConfigNominalOutputReverse(0, 0);
		currentTalon->ConfigPeakOutputForward(1, 0);
		currentTalon->ConfigPeakOutputReverse(-1, 0);
		currentTalon->ConfigNeutralDeadband(0.04, 0);
		currentTalon->ConfigVoltageCompSaturation(0, 0);
		currentTalon->ConfigVoltageMeasurementFilter(32, 0);
		currentTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		currentTalon->ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::QuadEncoder, 0);
		currentTalon->ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::QuadEncoder, 0);
		currentTalon->ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::QuadEncoder, 0);
		currentTalon->ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::QuadEncoder, 0);
		currentTalon->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_100Ms, 0);
		currentTalon->ConfigVelocityMeasurementWindow(64, 0);
		currentTalon->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 0);
		currentTalon->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 0);
		currentTalon->ConfigForwardSoftLimitThreshold(0, 0);
		currentTalon->ConfigReverseSoftLimitThreshold(0, 0);
		currentTalon->ConfigForwardSoftLimitEnable(false, 0);
		currentTalon->ConfigReverseSoftLimitEnable(false, 0);
		currentTalon->Config_kP(0, 0, 0);
		currentTalon->Config_kI(0 ,0 ,0);
		currentTalon->Config_kD(0, 0, 0);
		currentTalon->Config_kF(0, 0, 0);
		currentTalon->Config_IntegralZone(0, 0, 0);
		currentTalon->ConfigAllowableClosedloopError(0, 0, 0);
		currentTalon->ConfigMaxIntegralAccumulator(0, 0, 0);
		currentTalon->ConfigMotionCruiseVelocity(0, 0);
		currentTalon->ConfigMotionAcceleration(0, 0);
		currentTalon->ConfigMotionProfileTrajectoryPeriod(0, 0);
		currentTalon->ConfigSetCustomParam(0, 0, 0);
		currentTalon->ConfigPeakCurrentLimit(0, 0);
		currentTalon->ConfigPeakCurrentDuration(10, 0);
		currentTalon->ConfigContinuousCurrentLimit(0, 0);
	}
}
