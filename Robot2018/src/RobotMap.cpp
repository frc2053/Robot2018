#include <RobotMap.h>

std::shared_ptr<can::TalonSRX> RobotMap::intakeSubsystemLeftMotor;
std::shared_ptr<can::TalonSRX> RobotMap::intakeSubsystemRightMotor;

std::shared_ptr<can::TalonSRX> RobotMap::elevatorSubsystemPrimaryMotor;
std::shared_ptr<can::TalonSRX> RobotMap::elevatorSubsystemFollowerMotor;

std::shared_ptr<can::TalonSRX> RobotMap::climberSubsystemPrimaryMotor;
std::shared_ptr<can::TalonSRX> RobotMap::climberSubsystemFollowerMotor;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRDriveTalon;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRRotTalon;

std::shared_ptr<AHRS> RobotMap::robotIMU;
std::shared_ptr<frc::PowerDistributionPanel> RobotMap::powerDistributionPanel;
std::shared_ptr<TigerDrive> RobotMap::tigerDrive;
std::shared_ptr<TigerSwerve> RobotMap::tigerSwerve;
std::vector<std::shared_ptr<can::TalonSRX>> RobotMap::talonVector;

int RobotMap::TOP_POSITION_TICKS;
double RobotMap::GROUND_POS_IN;
double RobotMap::SCALE_POS_IN;
double RobotMap::SWITCH_POS_IN;

void RobotMap::init() {
	std::cout << "RobotMap is starting!" << std::endl;

	TOP_POSITION_TICKS = 24000;
	GROUND_POS_IN = 0;
	SCALE_POS_IN = 78;
	SWITCH_POS_IN = 21;

	intakeSubsystemLeftMotor.reset(new TalonSRX(10));
	intakeSubsystemRightMotor.reset(new TalonSRX(11));

	elevatorSubsystemPrimaryMotor.reset(new TalonSRX(13));
	elevatorSubsystemFollowerMotor.reset(new TalonSRX(14));

	climberSubsystemPrimaryMotor.reset(new TalonSRX(15));
	climberSubsystemFollowerMotor.reset(new TalonSRX(16));

	swerveSubsystemFLDriveTalon.reset(new can::TalonSRX(2));
	swerveSubsystemFRDriveTalon.reset(new can::TalonSRX(3));
	swerveSubsystemBLDriveTalon.reset(new can::TalonSRX(4));
	swerveSubsystemBRDriveTalon.reset(new can::TalonSRX(5));

	swerveSubsystemFLRotTalon.reset(new can::TalonSRX(6));
	swerveSubsystemFRRotTalon.reset(new can::TalonSRX(7));
	swerveSubsystemBLRotTalon.reset(new can::TalonSRX(8));
	swerveSubsystemBRRotTalon.reset(new can::TalonSRX(9));

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

	elevatorSubsystemPrimaryMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	elevatorSubsystemPrimaryMotor->SetSelectedSensorPosition(TOP_POSITION_TICKS, 0, 10);
	elevatorSubsystemPrimaryMotor->SetSensorPhase(false);
	elevatorSubsystemPrimaryMotor->SetInverted(false);
	elevatorSubsystemPrimaryMotor->Set(ControlMode::Position, TOP_POSITION_TICKS);
	elevatorSubsystemPrimaryMotor->Config_kP(0, 1, 10);
	elevatorSubsystemPrimaryMotor->Config_kI(0, 0, 10);
	elevatorSubsystemPrimaryMotor->Config_kD(0, 0, 10);

	elevatorSubsystemFollowerMotor->Set(ControlMode::Follower, 13);
	elevatorSubsystemFollowerMotor->SetInverted(false);

	climberSubsystemPrimaryMotor->SetInverted(false);
	climberSubsystemPrimaryMotor->Set(ControlMode::PercentOutput, 0);

	climberSubsystemFollowerMotor->Set(ControlMode::Follower, 15);
	climberSubsystemFollowerMotor->SetInverted(false);

	swerveSubsystemFLDriveTalon->SetInverted(true);
	swerveSubsystemFRDriveTalon->SetInverted(true);
	swerveSubsystemBLDriveTalon->SetInverted(true);
	swerveSubsystemBRDriveTalon->SetInverted(true);

	swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
	swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
	swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
	swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

	swerveSubsystemFLRotTalon->SetSelectedSensorPosition(swerveSubsystemFLRotTalon->GetSelectedSensorPosition(0)  & 0xFFF, 0, 10);
	swerveSubsystemFRRotTalon->SetSelectedSensorPosition(swerveSubsystemFRRotTalon->GetSelectedSensorPosition(0)  & 0xFFF, 0, 10);
	swerveSubsystemBLRotTalon->SetSelectedSensorPosition(swerveSubsystemBLRotTalon->GetSelectedSensorPosition(0)  & 0xFFF, 0, 10);
	swerveSubsystemBRRotTalon->SetSelectedSensorPosition(swerveSubsystemBRRotTalon->GetSelectedSensorPosition(0)  & 0xFFF, 0, 10);

	swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	swerveSubsystemFLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemFRRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBRRotTalon->SetSelectedSensorPosition(0, 0, 10);

	swerveSubsystemFLRotTalon->SetSensorPhase(true);
	swerveSubsystemFRRotTalon->SetSensorPhase(true);
	swerveSubsystemBLRotTalon->SetSensorPhase(true);
	swerveSubsystemBRRotTalon->SetSensorPhase(true);

	swerveSubsystemFLRotTalon->SetInverted(false);
	swerveSubsystemFRRotTalon->SetInverted(false);
	swerveSubsystemBLRotTalon->SetInverted(false);
	swerveSubsystemBRRotTalon->SetInverted(false);

	swerveSubsystemFLRotTalon->Set(ControlMode::Position, 0);
	swerveSubsystemFRRotTalon->Set(ControlMode::Position, 0);
	swerveSubsystemBLRotTalon->Set(ControlMode::Position, 0);
	swerveSubsystemBRRotTalon->Set(ControlMode::Position, 0);

	swerveSubsystemFLRotTalon->Config_kP(0, 6, 10);
	swerveSubsystemFLRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemFLRotTalon->Config_kD(0, 15, 10);

	swerveSubsystemFRRotTalon->Config_kP(0, 6, 10);
	swerveSubsystemFRRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemFRRotTalon->Config_kD(0, 15, 10);

	swerveSubsystemBLRotTalon->Config_kP(0, 6, 10);
	swerveSubsystemBLRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemBLRotTalon->Config_kD(0, 15, 10);

	swerveSubsystemBRRotTalon->Config_kP(0, 6, 10);
	swerveSubsystemBRRotTalon->Config_kI(0, 0, 10);
	swerveSubsystemBRRotTalon->Config_kD(0, 15, 10);

	swerveSubsystemFLRotTalon->ConfigPeakOutputForward(1, 10);
	swerveSubsystemFRRotTalon->ConfigPeakOutputForward(1, 10);
	swerveSubsystemBLRotTalon->ConfigPeakOutputForward(1, 10);
	swerveSubsystemBRRotTalon->ConfigPeakOutputForward(1, 10);

	swerveSubsystemFLRotTalon->ConfigPeakOutputReverse(-1, 10);
	swerveSubsystemFRRotTalon->ConfigPeakOutputReverse(-1, 10);
	swerveSubsystemBLRotTalon->ConfigPeakOutputReverse(-1, 10);
	swerveSubsystemBRRotTalon->ConfigPeakOutputReverse(-1, 10);

	swerveSubsystemFLRotTalon->ConfigAllowableClosedloopError(0, 35, 10);
	swerveSubsystemFRRotTalon->ConfigAllowableClosedloopError(0, 35, 10);
	swerveSubsystemBLRotTalon->ConfigAllowableClosedloopError(0, 35, 10);
	swerveSubsystemBRRotTalon->ConfigAllowableClosedloopError(0, 35, 10);
}

