#include <Pathfinder/MotionProfilerHelper.h>
#include "../RobotMap.h"

#include <ctre/phoenix/ErrorCode.h>
#include <iomanip>
#include <iostream>
#include "WPILib.h"

MotionProfilerHelper::MotionProfilerHelper() {

}

MotionProfilerHelper::~MotionProfilerHelper() {

}

void MotionProfilerHelper::ConfigureFPID(double kF, double kP, double kI, double kD) {
	RobotMap::swerveSubsystemFLDriveTalon->Config_kF(0, kF, 0);
	RobotMap::swerveSubsystemFLDriveTalon->Config_kF(0, kF, 0);
	RobotMap::swerveSubsystemFLDriveTalon->Config_kF(0, kF, 0);
	RobotMap::swerveSubsystemFLDriveTalon->Config_kF(0, kF, 0);

	RobotMap::swerveSubsystemFRDriveTalon->Config_kP(0, kP, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kP(0, kP, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kP(0, kP, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kP(0, kP, 0);

	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, kI, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, kI, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, kI, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, kI, 0);

	RobotMap::swerveSubsystemBRDriveTalon->Config_kD(0, kD, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kD(0, kD, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kD(0, kD, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kD(0, kD, 0);
}

void MotionProfilerHelper::ConfigureTicksPerRev(unsigned int ticksPerRev) {
	_ticksPerRev = ticksPerRev;
}

void MotionProfilerHelper::ConfigureWheelDiameter(double diameter) {
	wheelDiameter = diameter;
}

void MotionProfilerHelper::ConfigureMpUpdateRate(unsigned int milliseconds) {
	RobotMap::swerveSubsystemFLDriveTalon->ConfigMotionProfileTrajectoryPeriod(milliseconds, 0);
	RobotMap::swerveSubsystemFRDriveTalon->ConfigMotionProfileTrajectoryPeriod(milliseconds, 0);
	RobotMap::swerveSubsystemBLDriveTalon->ConfigMotionProfileTrajectoryPeriod(milliseconds, 0);
	RobotMap::swerveSubsystemBRDriveTalon->ConfigMotionProfileTrajectoryPeriod(milliseconds, 0);

	RobotMap::swerveSubsystemFLDriveTalon->ChangeMotionControlFramePeriod(milliseconds / 2);
	RobotMap::swerveSubsystemFRDriveTalon->ChangeMotionControlFramePeriod(milliseconds / 2);
	RobotMap::swerveSubsystemBLDriveTalon->ChangeMotionControlFramePeriod(milliseconds / 2);
	RobotMap::swerveSubsystemBRDriveTalon->ChangeMotionControlFramePeriod(milliseconds / 2);
}

bool MotionProfilerHelper::LoadPathfinder(Segment* segments, int length) {
	if(_mpLoadOffset == 0) {
		RobotMap::swerveSubsystemFLDriveTalon->ClearMotionProfileTrajectories();
		RobotMap::swerveSubsystemFRDriveTalon->ClearMotionProfileTrajectories();
		RobotMap::swerveSubsystemBLDriveTalon->ClearMotionProfileTrajectories();
		RobotMap::swerveSubsystemBRDriveTalon->ClearMotionProfileTrajectories();
	}

	ctrlMode = ControlMode::MotionProfile;

	double revs_per_m = 1 / (M_PI * wheelDiameter);
	for(int i = _mpLoadOffset; i < length; i++) {
		if(RobotMap::swerveSubsystemFLDriveTalon->IsMotionProfileTopLevelBufferFull()) {
			_mpLoadOffset = i;
			return false;
		}

		Segment* s = &segments[i];
		//this is rads per second
		double rpm = (s->velocity / (wheelDiameter / 2 * .10472));
		double pos = s->position * revs_per_m * _ticksPerRev;
		double vel = (rpm / 60) * _ticksPerRev * 10;
		ctre::phoenix::motion::TrajectoryPoint tp = {
			pos,
			vel,
			0, //heading (only works with ctre imu)
			0, 0, //slot select
			i == (length - 1), //last point?
			i == 0, //zero sensor
			ctre::phoenix::motion::TrajectoryDuration_0ms
		};

		if(RobotMap::swerveSubsystemFLDriveTalon->PushMotionProfileTrajectory(tp) != ctre::phoenix::ErrorCode::OKAY) {
			_mpLoadOffset = i + 1;
			return false;
		}
	}
	_mpLoadOffset = length - 1;
	return true;
}

void MotionProfilerHelper::ResetMp() {
	_mpLoadOffset = 0;
	RobotMap::swerveSubsystemFLDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemFRDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemBLDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemBRDriveTalon->ClearMotionProfileTrajectories();
}

void MotionProfilerHelper::EnableMp()  {
	RobotMap::swerveSubsystemFLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Enable);
	RobotMap::swerveSubsystemFRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Enable);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Enable);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Enable);
}

void MotionProfilerHelper::HoldMp() {
	RobotMap::swerveSubsystemFLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Hold);
	RobotMap::swerveSubsystemFRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Hold);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Hold);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Hold);
}

void MotionProfilerHelper::DisableMp() {
	RobotMap::swerveSubsystemFLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Disable);
	RobotMap::swerveSubsystemFRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Disable);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Disable);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ctrlMode, ctre::phoenix::motion::SetValueMotionProfile::Disable);
}

ctre::phoenix::motion::MotionProfileStatus MotionProfilerHelper::ProcessMp() {
	RobotMap::swerveSubsystemFLDriveTalon->ProcessMotionProfileBuffer();
	RobotMap::swerveSubsystemFRDriveTalon->ProcessMotionProfileBuffer();
	RobotMap::swerveSubsystemBLDriveTalon->ProcessMotionProfileBuffer();
	RobotMap::swerveSubsystemBRDriveTalon->ProcessMotionProfileBuffer();

	MotionProfileStatus status;
	RobotMap::swerveSubsystemFLDriveTalon->GetMotionProfileStatus(status);
	return status;
}

void MotionProfilerHelper::PrintStatus(MotionProfileStatus status, double pos, double vel,
		double heading)
{
	static double timeout = 0;
	static int count = 0;

	const char delim[] = "\t";
	const char endline[] = "\n";

	double now = GetTime();

	if((now-timeout) > 0.2){
		timeout = now;
		/* fire a loop every 200ms */

		if(--count <= 0){
			count = 8;
			/* every 8 loops, print our columns */
			std::cout
						<< "       outEn" << delim
						<< "topBufferCnt" << delim
						<< "topBufferRem" << delim
						<< "btmBufferCnt" << delim
						<< "     IsValid" << delim
						<< " HasUnderrun" << delim
						<< "  IsUnderrun" << delim
						<< "      IsLast" << delim
						<< "     targPos" << delim
						<< "     targVel" << delim
						<< "    SlotSel0" << delim
						<< "   timeDurMs" << delim

						<< endline;
		}
		/* every loop, print our values */
		std::cout	<< std::setw(12)<< status.topBufferCnt << delim
					<< std::setw(12)<< status.topBufferRem << delim
					<< std::setw(12)<< status.btmBufferCnt << delim
					<< std::setw(12)<< (status.activePointValid ? "1" : " ") << delim
					<< std::setw(12)<< (status.hasUnderrun ? "1" : " ") << delim
					<< std::setw(12)<< (status.isUnderrun ? "1" : " ") << delim
					<< std::setw(12)<< (status.isLast ? "1" : " ") << delim
					<< std::setw(12)<< pos << delim
					<< std::setw(12)<< vel << delim
					<< std::setw(12)<< status.profileSlotSelect0 << delim
					<< std::setw(12)<< status.timeDurMs << delim

					<< endline;
	}
}
