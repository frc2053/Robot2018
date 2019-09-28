#include "Commands/Drive/DriveCommand.h"
#define _USE_MATH_DEFINES
#include <math.h>

DriveCommand::DriveCommand() {
	Requires(Robot::swerveSubsystem.get());

	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	currentYaw = 0;
	finalRotVal = 0;
	setAngle = 0;
	isLeftStickPressed = false;
	isAPressed = false;
	isBPressed = false;
	isXPressed = false;
	isYPressed = false;
	isRotDone = false;
}

void DriveCommand::Initialize() {
	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	isAPressed = 0;
	isBPressed = 0;
	isXPressed = 0;
	isYPressed = 0;
}

void DriveCommand::Execute() {
	//std::cout << "drivecommand!\n";
	SmartDashboard::PutNumber("IMU Yaw", currentYaw);
	SmartDashboard::PutNumber("X Axis", xAxis);
	SmartDashboard::PutNumber("Y Axis", yAxis);
	SmartDashboard::PutNumber("Rot Axis", rotAxis);

	GetInputs();

	currentYaw = Robot::swerveSubsystem->GetAdjYaw();
	isRotDone = Robot::swerveSubsystem->GetIsRotDone();



	//int angle = (RobotMap::swerveSubsystemFrontLeftRotationTalon->getpo & 0xFFF);
	//std::cout << "Talon Current: " << angle << std::endl;

	//RobotMap::swerveSubsystemFrontLeftRotationTalon->Set(Robot::oi->GetDriverJoystick()->GetLeftYAxis());


	SetAngleFromInput();
	RotateCommand();
	CheckRotateOverride();
	CallToSwerveDrive();
}

bool DriveCommand::IsFinished() {
	return false;
}

void DriveCommand::End() {

}

void DriveCommand::Interrupted() {

}

void DriveCommand::GetInputs() {
	xAxis = Robot::oi->GetDriverJoystick()->GetLeftXAxis();
	yAxis = Robot::oi->GetDriverJoystick()->GetLeftYAxis();
	rotAxis = Robot::oi->GetDriverJoystick()->GetRightXAxis();

	isAPressed = Robot::oi->GetDriverJoystick()->aButton->Get();
	isBPressed = Robot::oi->GetDriverJoystick()->bButton->Get();
	isXPressed = Robot::oi->GetDriverJoystick()->xButton->Get();
	isYPressed = Robot::oi->GetDriverJoystick()->yButton->Get();
}

void DriveCommand::SetAngleFromInput() {
	if(isAPressed) {
		RobotMap::tigerDrive->rotateController->SetSetpoint(180);
		setAngle = 180;
	}
	if(isBPressed) {
		RobotMap::tigerDrive->rotateController->SetSetpoint(90);
		setAngle = 90;
	}
	if(isXPressed) {
		RobotMap::tigerDrive->rotateController->SetSetpoint(-90);
		setAngle = -90;
	}
	if(isYPressed) {
		RobotMap::tigerDrive->rotateController->SetSetpoint(0);
		setAngle = 0;
	}
	if(isLeftStickPressed)
	{
		if(xAxis != 0 && yAxis != 0) {
			double rad = atan2(xAxis, yAxis);
			double degrees = rad * (180 / M_PI);
			setAngle = degrees;
			RobotMap::tigerDrive->rotateController->SetSetpoint(degrees);
		}
	}
}

void DriveCommand::RotateCommand()
{
	if(((isLeftStickPressed || isYPressed == true|| isXPressed == true || isAPressed == true || isBPressed == true) && isRotDone == true) || (isRotDone == false))
	{
		finalRotVal = Robot::swerveSubsystem->CalculateRotValue(setAngle, 1);
	}
}

void DriveCommand::CheckRotateOverride() {
	if(Robot::swerveSubsystem->GetIsRotDoneOverride())
	{
		finalRotVal = 0;
	}
}

void DriveCommand::CallToSwerveDrive() {
	if(rotAxis == 0)
	{
		Robot::swerveSubsystem->SetIsRotDoneOverride(false);
		std::cout << "TELE: side: " << xAxis << " fow: " << -yAxis << " rot: " << -rotAxis << " yaw: " << currentYaw << "\n";
		Robot::swerveSubsystem->SwerveDrive(xAxis, -yAxis, -finalRotVal, currentYaw);
	}
	else
	{
		Robot::swerveSubsystem->SetIsRotDoneOverride(true);
		Robot::swerveSubsystem->SetIsRotDone(true);
		Robot::swerveSubsystem->SetTimesThroughLoop(0);
		std::cout << "TELE: side: " << xAxis << " fow: " << -yAxis << " rot: " << -rotAxis << " yaw: " << currentYaw << "\n";
		Robot::swerveSubsystem->SwerveDrive(xAxis, -yAxis, -rotAxis, currentYaw);
	}
}
