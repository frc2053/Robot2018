#include "SwerveSubsystem.h"
#include "../Commands/Drive/DriveCommand.h"

SwerveSubsystem::SwerveSubsystem() : Subsystem("SwerveSubsystem") {
	std::cout << "Constructor for Swerve Subsystem!" << std::endl;
	frontLeftDriveTalon = RobotMap::swerveSubsystemFLDriveTalon;
	frontRightDriveTalon = RobotMap::swerveSubsystemFRDriveTalon;
	backLeftDriveTalon = RobotMap::swerveSubsystemBLDriveTalon;
	backRightDriveTalon = RobotMap::swerveSubsystemBRDriveTalon;

	frontLeftRotationTalon = RobotMap::swerveSubsystemFLRotTalon;
	frontRightRotationTalon = RobotMap::swerveSubsystemFRRotTalon;
	backLeftRotationTalon = RobotMap::swerveSubsystemBLRotTalon;
	backRightRotationTalon = RobotMap::swerveSubsystemBRRotTalon;

	tigerSwerve = RobotMap::tigerSwerve;
	tigerDrive = RobotMap::tigerDrive;
}

void SwerveSubsystem::CalibrateWheels() {
	int currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition();
	int currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition();
	int currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition();
	int currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition();

/*
	std::cout << "currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "currentQuadbr: " << currentQuadbr << std::endl;
*/

	int currentPWMfl = frontLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	int currentPWMfr = frontRightRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	int currentPWMbl = backLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	int currentPWMbr = backRightRotationTalon->GetSensorCollection().GetPulseWidthPosition();

/*
	std::cout << "currentPWMfl: " << currentPWMfl << std::endl;
	std::cout << "currentPWMfr: " << currentPWMfr << std::endl;
	std::cout << "currentPWMbl: " << currentPWMbl << std::endl;
	std::cout << "currentPWMbr: " << currentPWMbr << std::endl;
*/

	int absModfl = AbsMod(currentQuadfl, TICKS_PER_REV);
	int absModfr = AbsMod(currentQuadfr, TICKS_PER_REV);
	int absModbl = AbsMod(currentQuadbl, TICKS_PER_REV);
	int absModbr = AbsMod(currentQuadbr, TICKS_PER_REV);

/*
	std::cout << "absModfl: " << absModfl << std::endl;
	std::cout << "absModfr: " << absModfr << std::endl;
	std::cout << "absModbl: " << absModbl << std::endl;
	std::cout << "absModbr: " << absModbr << std::endl;
*/

	int flSetpoint = ((absModfl) % TICKS_PER_REV) + (FL_CAL - currentPWMfl);
	int frSetpoint = ((absModfr) % TICKS_PER_REV) + (FR_CAL - currentPWMfr);
	int blSetpoint = ((absModbl) % TICKS_PER_REV) + (BL_CAL - currentPWMbl);
	int brSetpoint = ((absModbr) % TICKS_PER_REV) + (BR_CAL - currentPWMbr);

/*
	std::cout << "flSetpoint: " << flSetpoint << std::endl;
	std::cout << "frSetpoint: " << frSetpoint << std::endl;
	std::cout << "blSetpoint: " << blSetpoint << std::endl;
	std::cout << "brSetpoint: " << brSetpoint << std::endl;
*/

/*
	flSetpoint = (SwerveSubsystem::OptimizeRot(flSetpoint, TICKS_PER_REV));
	frSetpoint = (SwerveSubsystem::OptimizeRot(frSetpoint, TICKS_PER_REV));
	blSetpoint = (SwerveSubsystem::OptimizeRot(blSetpoint, TICKS_PER_REV));
	brSetpoint = (SwerveSubsystem::OptimizeRot(brSetpoint, TICKS_PER_REV));

	std::cout << "flSetpoint: " << flSetpoint << std::endl;
	std::cout << "frSetpoint: " << frSetpoint << std::endl;
	std::cout << "blSetpoint: " << blSetpoint << std::endl;
	std::cout << "brSetpoint: " << brSetpoint << std::endl;
*/

	frontLeftRotationTalon->Set(ControlMode::Position, flSetpoint);
	frontRightRotationTalon->Set(ControlMode::Position, frSetpoint);
	backLeftRotationTalon->Set(ControlMode::Position, blSetpoint);
	backRightRotationTalon->Set(ControlMode::Position, brSetpoint);

	currentPWMfl = frontLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	currentPWMfr = frontRightRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	currentPWMbl = backLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition();
	currentPWMbr = backRightRotationTalon->GetSensorCollection().GetPulseWidthPosition();

	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition();
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition();
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition();
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition();

	std::cout << "Calibrated Wheels!" << std::endl;
}

int SwerveSubsystem::AbsMod(int value, int ticks) {
	int retVal = value;
	if(value < 0) {
		retVal = ticks - abs(value);
	}
	return retVal;
}

void SwerveSubsystem::InitDefaultCommand() {
	//SetDefaultCommand(new DriveCommand());
}

double SwerveSubsystem::CalculateRotValue(double setAngle, double setSpeed)
{
	return tigerDrive->CalculateRotationValue(setAngle, setSpeed);
}

bool SwerveSubsystem::GetIsRotDone()
{
	return tigerDrive->GetIsRotDone();
}

double SwerveSubsystem::GetAdjYaw() {
	return tigerDrive->GetAdjYaw();
}

void SwerveSubsystem::SetAdjYaw(double yaw) {
	tigerDrive->SetAdjYaw(yaw);
}

void SwerveSubsystem::SetIsRotDone(bool isDone)
{
	tigerDrive->SetIsRotDone(isDone);
}

void SwerveSubsystem::SetIsRotDoneOverride(bool isDone)
{
	tigerDrive->SetIsRotDoneOverride(isDone);
}

void SwerveSubsystem::SetTimesThroughLoop(int timeLoop)
{
	tigerDrive->SetTimesThroughLoop(timeLoop);
}

bool SwerveSubsystem::GetIsRotDoneOverride() {
	return tigerDrive->GetIsRotDoneOverride();
}

void SwerveSubsystem::ZeroYaw() {
	tigerDrive->imu->ZeroYaw();
}

void SwerveSubsystem::SwerveDrive(double xAxis, double yAxis, double rotAxis, double currentYaw) {
	tigerSwerve->DriveFieldOriented(xAxis, yAxis, rotAxis, currentYaw);
}

std::shared_ptr<TigerSwerve> SwerveSubsystem::GetSwerveStuff() {
	return tigerSwerve;
}

int SwerveSubsystem::OptimizeRot(int value, int ticks) {
	int retVal = value;

	absVal = abs(value);
	halfTicks =  ticks/2;

	if(absVal > halfTicks) {
		retVal = (ticks - absVal);
	}

	if (value > halfTicks) {
		retVal = retVal * -1;
	}

	return retVal;
}
