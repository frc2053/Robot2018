#include "ZeroYaw.h"

ZeroYaw::ZeroYaw()
{
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
}

void ZeroYaw::Initialize()
{
	isDone = false;
}

void ZeroYaw::Execute()
{
	isDone = false;
	Robot::swerveSubsystem->ZeroYaw();
	isDone = true;
}

bool ZeroYaw::IsFinished()
{
	return isDone;
}

void ZeroYaw::End()
{

}

void ZeroYaw::Interrupted()
{

}
