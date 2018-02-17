#include "GoDistance.h"

#include "../../Robot.h"
#include "../../RobotMap.h"

GoDistance::GoDistance(double Xdistance, double Ydistance) {
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
	_xDistance = Xdistance;
	_yDistance = Ydistance;
	modules = Robot::swerveSubsystem->GetSwerveStuff()->GetModules();
	RobotMap::swerveSubsystemFLDriveTalon->Config_kP(0, RobotMap::K_P, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kP(0, RobotMap::K_P, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kP(0, RobotMap::K_P, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kP(0, RobotMap::K_P, 0);

	RobotMap::swerveSubsystemFLDriveTalon->Config_kI(0, RobotMap::K_I, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kI(0, RobotMap::K_I, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, RobotMap::K_I, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kI(0, RobotMap::K_I, 0);

	RobotMap::swerveSubsystemFLDriveTalon->Config_kD(0, RobotMap::K_D, 0);
	RobotMap::swerveSubsystemFRDriveTalon->Config_kD(0, RobotMap::K_D, 0);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kD(0, RobotMap::K_D, 0);
	RobotMap::swerveSubsystemBRDriveTalon->Config_kD(0, RobotMap::K_D, 0);

	RobotMap::swerveSubsystemFLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemFRDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBRDriveTalon->SetSelectedSensorPosition(0, 0, 0);

	deltaDistance = sqrt(pow(_xDistance, 2) + pow(_yDistance, 2));
	angleForWheel = std::atan2(_xDistance, _yDistance);
	ticks = ConvertFeetToTicks(deltaDistance);
}

void GoDistance::Initialize() {

}

void GoDistance::Execute() {
	for(int i = 0; i < modules->size(); i++) {
		modules->at(i).SetAngle(Rotation2D::fromDegrees(angleForWheel), true);
	}
	RobotMap::swerveSubsystemFLDriveTalon->Set(ControlMode::Position, ticks);
	RobotMap::swerveSubsystemFRDriveTalon->Set(ControlMode::Position, ticks);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::Position, ticks);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ControlMode::Position, ticks);
	isDone = true;
}

bool GoDistance::IsFinished() {
	return isDone;
}

void GoDistance::End() {
}

void GoDistance::Interrupted() {

}

int GoDistance::ConvertFeetToTicks(double feet) {
	return feet * RobotMap::TICKS_PER_FOOT_DRIVE;
}
