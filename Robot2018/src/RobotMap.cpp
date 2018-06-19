#include "RobotMap.h"
#include "WPILib.h"

std::shared_ptr<can::TalonSRX> RobotMap::intakeSubsystemLeftMotor;
std::shared_ptr<can::TalonSRX> RobotMap::intakeSubsystemRightMotor;

std::shared_ptr<can::TalonSRX> RobotMap::elevatorClimberSubsystemPrimaryTalon;
std::shared_ptr<can::TalonSRX> RobotMap::elevatorClimberSubsystemFollower01Talon;
std::shared_ptr<can::TalonSRX> RobotMap::elevatorClimberSubsystemFollower02Talon;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRDriveTalon;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRRotTalon;

std::shared_ptr<frc::DoubleSolenoid> RobotMap::climberSubsystemWing02Solenoid;
std::shared_ptr<frc::DoubleSolenoid> RobotMap::climberSubsystemWingSolenoid;
std::shared_ptr<frc::DoubleSolenoid> RobotMap::elevatorClimberSubsystemShifterSolenoid;
std::shared_ptr<frc::Servo> RobotMap::climberSubsystemStopperServo;


std::shared_ptr<AHRS> RobotMap::robotIMU;
std::shared_ptr<frc::PowerDistributionPanel> RobotMap::powerDistributionPanel;
std::shared_ptr<TigerDrive> RobotMap::tigerDrive;
std::shared_ptr<TigerSwerve> RobotMap::tigerSwerve;
std::vector<std::shared_ptr<can::TalonSRX>> RobotMap::talonVector;
std::vector<std::shared_ptr<can::TalonSRX>> RobotMap::allTalons;

int RobotMap::TOP_POSITION_TICKS;
double RobotMap::GROUND_POS_FT;
double RobotMap::SCALE_POS_FT;
double RobotMap::SWITCH_POS_FT;
double RobotMap::CLIMBBAR_POS_FT;
double RobotMap::WHEELBASE_WIDTH;
double RobotMap::WHEELBASE_LENGTH;
double RobotMap::TIMESTEP;
double RobotMap::MAX_VEL;
double RobotMap::MAX_ACCEL;
double RobotMap::MAX_JERK;
int RobotMap::TICKS_PER_REV;
int RobotMap::TICKS_PER_FOOT_DRIVE;
double RobotMap::WHEEL_CIRCUMFERENCE;
double RobotMap::K_P;
double RobotMap::K_I;
double RobotMap::K_D;
double RobotMap::K_V;
double RobotMap::K_A;
double RobotMap::K_T;

void RobotMap::init() {
	std::cout << "RobotMap is starting!" << std::endl;

	TOP_POSITION_TICKS = 129751; //24000; VALUE TO BE TESTED AND CHANGED
	//FEET
	GROUND_POS_FT = -0.6;
	SCALE_POS_FT = -5.9;
	SWITCH_POS_FT = -2;
	CLIMBBAR_POS_FT = -7.0;
	//INCHES - don't question it
	WHEELBASE_LENGTH = 21;
	WHEELBASE_WIDTH = 26.249;

	TIMESTEP = 0.02;
	MAX_VEL = 18;
	MAX_ACCEL = 12;
	MAX_JERK = 60;
	TICKS_PER_REV = 26214;
	TICKS_PER_FOOT_DRIVE = 37187;
	WHEEL_CIRCUMFERENCE = 0.65449867893738;
	K_P = 1;
	K_I = 0.0;
	K_D = .15;
	K_V = .06; //.06
	K_A = .0856;
	K_T = .35; //.35

	LiveWindow::GetInstance()->DisableAllTelemetry();

	intakeSubsystemLeftMotor.reset(new TalonSRX(10));
	intakeSubsystemRightMotor.reset(new TalonSRX(11));

	intakeSubsystemRightMotor->SetInverted(true);


	elevatorClimberSubsystemPrimaryTalon.reset(new TalonSRX(13));
	elevatorClimberSubsystemFollower01Talon.reset(new TalonSRX(14));
	elevatorClimberSubsystemFollower02Talon.reset(new TalonSRX(15));

	swerveSubsystemFLDriveTalon.reset(new can::TalonSRX(2));
	swerveSubsystemFRDriveTalon.reset(new can::TalonSRX(3));
	swerveSubsystemBLDriveTalon.reset(new can::TalonSRX(4));
	swerveSubsystemBRDriveTalon.reset(new can::TalonSRX(5));

	swerveSubsystemFLRotTalon.reset(new can::TalonSRX(6));
	swerveSubsystemFRRotTalon.reset(new can::TalonSRX(7));
	swerveSubsystemBLRotTalon.reset(new can::TalonSRX(8));
	swerveSubsystemBRRotTalon.reset(new can::TalonSRX(9));

	climberSubsystemWing02Solenoid.reset(new frc::DoubleSolenoid(4, 5));
	climberSubsystemWingSolenoid.reset(new frc::DoubleSolenoid(2, 3));
	elevatorClimberSubsystemShifterSolenoid.reset(new frc::DoubleSolenoid(0, 1));

	allTalons.push_back(intakeSubsystemLeftMotor);
	allTalons.push_back(intakeSubsystemRightMotor);
	allTalons.push_back(elevatorClimberSubsystemPrimaryTalon);
	allTalons.push_back(elevatorClimberSubsystemFollower01Talon);
	allTalons.push_back(elevatorClimberSubsystemFollower02Talon);
	allTalons.push_back(swerveSubsystemFLDriveTalon);
	allTalons.push_back(swerveSubsystemFRDriveTalon);
	allTalons.push_back(swerveSubsystemBLDriveTalon);
	allTalons.push_back(swerveSubsystemBRDriveTalon);
	allTalons.push_back(swerveSubsystemFLRotTalon);
	allTalons.push_back(swerveSubsystemFRRotTalon);
	allTalons.push_back(swerveSubsystemBLRotTalon);
	allTalons.push_back(swerveSubsystemBRRotTalon);

	RobotMap::resetTalons(allTalons);

	talonVector.push_back(swerveSubsystemFLDriveTalon);
	talonVector.push_back(swerveSubsystemFRDriveTalon);
	talonVector.push_back(swerveSubsystemBLDriveTalon);
	talonVector.push_back(swerveSubsystemBRDriveTalon);

	talonVector.push_back(swerveSubsystemFLRotTalon);
	talonVector.push_back(swerveSubsystemFRRotTalon);
	talonVector.push_back(swerveSubsystemBLRotTalon);
	talonVector.push_back(swerveSubsystemBRRotTalon);

	robotIMU.reset(new AHRS(frc::SPI::Port::kMXP));
	powerDistributionPanel.reset(new frc::PowerDistributionPanel());
	tigerSwerve.reset(new TigerSwerve(RobotMap::talonVector));
	tigerDrive.reset(new TigerDrive(RobotMap::robotIMU.get()));

	//Configures the elevator talons
	//We want to use the relative encoder because we dont need absolute feedback
	//then we set the top of the elevator to zero i think..
	//sets PIDS
	elevatorClimberSubsystemPrimaryTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	//elevatorClimberSubsystemPrimaryTalon->SetSelectedSensorPosition(TOP_POSITION_TICKS, 0, 10);
	elevatorClimberSubsystemPrimaryTalon->SetSelectedSensorPosition(0, 0, 10);
	elevatorClimberSubsystemPrimaryTalon->SetSensorPhase(true);
	elevatorClimberSubsystemPrimaryTalon->SetInverted(true);
	elevatorClimberSubsystemPrimaryTalon->Config_kP(0, 0.1, 10); //.8
	elevatorClimberSubsystemPrimaryTalon->Config_kI(0, 0, 10); //.8
	elevatorClimberSubsystemPrimaryTalon->Config_kD(0, 0, 10); //1.5
	elevatorClimberSubsystemPrimaryTalon->ConfigAllowableClosedloopError(0, 700, 10);

	//make the other motor follow the primary one
	elevatorClimberSubsystemFollower01Talon->Set(ControlMode::Follower, 13);
	elevatorClimberSubsystemFollower01Talon->SetInverted(true);

	elevatorClimberSubsystemFollower02Talon->Set(ControlMode::Follower, 13);
	elevatorClimberSubsystemFollower02Talon->SetInverted(true);


	elevatorClimberSubsystemFollower01Talon->ConfigVoltageCompSaturation(0, 10);
	elevatorClimberSubsystemFollower02Talon->ConfigVoltageCompSaturation(0, 10);
	elevatorClimberSubsystemPrimaryTalon->ConfigVoltageCompSaturation(0, 10);

	climberSubsystemStopperServo.reset(new frc::Servo(9));


	//makes the drive talons drive the right way
	swerveSubsystemFLDriveTalon->SetInverted(false);
	swerveSubsystemFRDriveTalon->SetInverted(false);
	swerveSubsystemBLDriveTalon->SetInverted(false);
	swerveSubsystemBRDriveTalon->SetInverted(false);

	//zeros the quad encoders on start up
	swerveSubsystemFLDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	swerveSubsystemFRDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	swerveSubsystemBLDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	swerveSubsystemBRDriveTalon->GetSensorCollection().SetQuadraturePosition(0, 10);

	//make sure we use the quad encoders when we do PID on them
	swerveSubsystemFLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemFRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemBLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemBRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);

	//for the talon PID to work we need to make sure that the sensors are in phase
	//this means that when we give the motor a positive voltage the encoder ticks MUST go more positive
	//if this is not set correctly PID will not work at all
	swerveSubsystemFLDriveTalon->SetSensorPhase(true);
	swerveSubsystemFRDriveTalon->SetSensorPhase(true);
	swerveSubsystemBLDriveTalon->SetSensorPhase(true);
	swerveSubsystemBRDriveTalon->SetSensorPhase(true);

	swerveSubsystemFLDriveTalon->ConfigOpenloopRamp(0.4, 10);
	swerveSubsystemFRDriveTalon->ConfigOpenloopRamp(0.4, 10);
	swerveSubsystemBLDriveTalon->ConfigOpenloopRamp(0.4, 10);
	swerveSubsystemBRDriveTalon->ConfigOpenloopRamp(0.4, 10);



	//we want to use relative for rotation motors because it is much faster read time
	swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

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
	swerveSubsystemFRRotTalon->SetSensorPhase(false);
	swerveSubsystemBLRotTalon->SetSensorPhase(false);
	swerveSubsystemBRRotTalon->SetSensorPhase(false);

	//makes sure we are rotating clockwise with positive voltage
	swerveSubsystemFLRotTalon->SetInverted(true);
	swerveSubsystemFRRotTalon->SetInverted(true);
	swerveSubsystemBLRotTalon->SetInverted(true);
	swerveSubsystemBRRotTalon->SetInverted(true);

	//PIDS
	swerveSubsystemFLRotTalon->Config_kP(0, 12, 10);
	swerveSubsystemFLRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemFLRotTalon->Config_kD(0, 120, 10);

	swerveSubsystemFRRotTalon->Config_kP(0, 12, 10);
	swerveSubsystemFRRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemFRRotTalon->Config_kD(0, 120, 10);

	swerveSubsystemBLRotTalon->Config_kP(0, 12, 10);
	swerveSubsystemBLRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemBLRotTalon->Config_kD(0, 120, 10);

	swerveSubsystemBRRotTalon->Config_kP(0, 12, 10);
	swerveSubsystemBRRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemBRRotTalon->Config_kD(0, 120, 10);

	//Rotation motors are too strong so we limit them to around half total output
	swerveSubsystemFLRotTalon->ConfigPeakOutputForward(1.0, 10); //.416
	swerveSubsystemFRRotTalon->ConfigPeakOutputForward(1.0, 10);
	swerveSubsystemBLRotTalon->ConfigPeakOutputForward(1.0, 10);
	swerveSubsystemBRRotTalon->ConfigPeakOutputForward(1.0, 10);

	//same as above except in reverse
	swerveSubsystemFLRotTalon->ConfigPeakOutputReverse(-1.0, 10); //-.416
	swerveSubsystemFRRotTalon->ConfigPeakOutputReverse(-1.0, 10);
	swerveSubsystemBLRotTalon->ConfigPeakOutputReverse(-1.0, 10);
	swerveSubsystemBRRotTalon->ConfigPeakOutputReverse(-1.0, 10);

	//Tolerance for PID
	swerveSubsystemFLRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	swerveSubsystemFRRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	swerveSubsystemBLRotTalon->ConfigAllowableClosedloopError(0, 5, 10);
	swerveSubsystemBRRotTalon->ConfigAllowableClosedloopError(0, 5, 10);

	//This prevents the rotation motors from drawing too much current
	swerveSubsystemFLRotTalon->ConfigContinuousCurrentLimit(10, 10);
	swerveSubsystemFRRotTalon->ConfigContinuousCurrentLimit(10, 10);
	swerveSubsystemBLRotTalon->ConfigContinuousCurrentLimit(10, 10);
	swerveSubsystemBRRotTalon->ConfigContinuousCurrentLimit(10, 10);

	swerveSubsystemFLRotTalon->ConfigPeakCurrentLimit(0, 10);
	swerveSubsystemFRRotTalon->ConfigPeakCurrentLimit(0, 10);
	swerveSubsystemBLRotTalon->ConfigPeakCurrentLimit(0, 10);
	swerveSubsystemBRRotTalon->ConfigPeakCurrentLimit(0, 10);
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
