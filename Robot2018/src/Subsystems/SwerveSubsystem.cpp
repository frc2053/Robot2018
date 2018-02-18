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

void SwerveSubsystem::CalibrateWheelsSimple(){

	RobotMap::swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	RobotMap::swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	RobotMap::swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	RobotMap::swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

	int flpw = RobotMap::swerveSubsystemFLRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int frpw = RobotMap::swerveSubsystemFRRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int blpw = RobotMap::swerveSubsystemBLRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int brpw = RobotMap::swerveSubsystemBRRotTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;

	int flset = flpw - FL_CAL;
	int frset = frpw - FR_CAL;
	int blset = blpw - BL_CAL;
	int brset = brpw - BR_CAL;

	RobotMap::swerveSubsystemFLRotTalon->Set(ControlMode::Position, flset);
	RobotMap::swerveSubsystemFRRotTalon->Set(ControlMode::Position, frset);
	RobotMap::swerveSubsystemBLRotTalon->Set(ControlMode::Position, blset);
	RobotMap::swerveSubsystemBRRotTalon->Set(ControlMode::Position, brset);
}

void SwerveSubsystem::CalibrateWheels() {

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
