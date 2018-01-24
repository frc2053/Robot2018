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

std::vector<TrajectoryPoint> MotionProfilerHelper::convertToSRXTrajectory(Segment* trajectory, int inputLength) {
	int length = inputLength;
	std::vector<TrajectoryPoint> retVal;

	for(int i = 0; i < length; i++) {
		Segment* currentSegment = &trajectory[i];
		TrajectoryPoint point;

		point.position = ConvertFeetToTicks(currentSegment->position);
		point.velocity = (currentSegment->velocity  / MAX_VELOCITY) / 10;
		point.timeDur = TrajectoryDuration_0ms;
		point.profileSlotSelect0 = 0;
		point.zeroPos = (i == 0);
		point.isLastPoint = (i == length - 1);
		point.velocity = point.zeroPos ? point.velocity : 0;

		retVal.push_back(point);
	}

	return retVal;
}

int MotionProfilerHelper::ConvertFeetToTicks(double pos) {
	int retVal = 0;
	retVal = pos * TICKS_PER_FOOT;
	return retVal;
}

void MotionProfilerHelper::PushTrajectoryToTalons(std::vector<TrajectoryPoint> points) {
	for(int i = 0; i < points.size(); i++) {
		RobotMap::swerveSubsystemFLDriveTalon->PushMotionProfileTrajectory(points.at(i));
		RobotMap::swerveSubsystemFRDriveTalon->PushMotionProfileTrajectory(points.at(i));
		RobotMap::swerveSubsystemBLDriveTalon->PushMotionProfileTrajectory(points.at(i));
		RobotMap::swerveSubsystemBRDriveTalon->PushMotionProfileTrajectory(points.at(i));
	}
}

void MotionProfilerHelper::ResetMp() {
	_mpLoadOffset = 0;
	RobotMap::swerveSubsystemFLDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemFRDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemBLDriveTalon->ClearMotionProfileTrajectories();
	RobotMap::swerveSubsystemBRDriveTalon->ClearMotionProfileTrajectories();
}

void MotionProfilerHelper::EnableMp()  {
	ctrlMode = ControlMode::MotionProfile;
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
