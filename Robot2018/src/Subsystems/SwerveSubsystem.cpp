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

void SwerveSubsystem::InitDefaultCommand() {
	SetDefaultCommand(new DriveCommand());
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
