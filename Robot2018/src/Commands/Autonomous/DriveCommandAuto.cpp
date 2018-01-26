#include "DriveCommandAuto.h"

DriveCommandAuto::DriveCommandAuto(float side, float fow, float rot, float time, float angle)
{
	Requires(Robot::swerveSubsystem.get());
	inputSide = side;
	inputFow = fow;
	inputRot = rot;
	timeTarget = time;
	inputAngle = angle;
	timeCurrent = 0;
	isDone = false;
	adjustedYaw = 0;
	finalAutoRot = 0;
	isRotDone = false;
	timer.reset(new Timer());
	timer->Reset();
	timer->Start();
}

void DriveCommandAuto::Initialize()
{
	timer->Reset();
	timer->Start();
}

void DriveCommandAuto::Execute()
{
	timeCurrent = timer->Get();
	RobotMap::tigerDrive->rotateController->SetSetpoint(inputAngle);
	adjustedYaw = Robot::swerveSubsystem->GetAdjYaw();
	isRotDone = Robot::swerveSubsystem->GetIsRotDone();
	finalAutoRot = Robot::swerveSubsystem->CalculateRotValue(inputAngle, .75);

	if(isRotDone)
	{
		Robot::swerveSubsystem->SwerveDrive(inputSide, inputFow, 0, adjustedYaw);
	}
	else
	{
		std::cout << "finalAutoRot: " << finalAutoRot << "\n";
		Robot::swerveSubsystem->SwerveDrive(inputSide, inputFow, finalAutoRot, adjustedYaw);
	}

	if((timeCurrent >= timeTarget)) {
		Robot::swerveSubsystem->SwerveDrive(0,0,0,0);
		isDone = true;
	}
	else {
		Robot::swerveSubsystem->SwerveDrive(inputSide, inputFow, finalAutoRot, adjustedYaw);
		isDone = false;
	}
}

bool DriveCommandAuto::IsFinished()
{
	return isDone;
}

void DriveCommandAuto::End()
{
	timer->Stop();
	Robot::swerveSubsystem->SwerveDrive(0,0,0,0);
}

void DriveCommandAuto::Interrupted()
{
	Robot::swerveSubsystem->SwerveDrive(0,0,0,0);
}
