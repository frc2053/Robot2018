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

	RobotMap::swerveSubsystemFLDriveTalon->ConfigAllowableClosedloopError(0, 325, 0);
	RobotMap::swerveSubsystemFRDriveTalon->ConfigAllowableClosedloopError(0, 325, 0);
	RobotMap::swerveSubsystemBLDriveTalon->ConfigAllowableClosedloopError(0, 325, 0);
	RobotMap::swerveSubsystemBRDriveTalon->ConfigAllowableClosedloopError(0, 325, 0);

	RobotMap::swerveSubsystemFLDriveTalon->Set(ControlMode::Follower, 4);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ControlMode::Follower, 4);
	RobotMap::swerveSubsystemFRDriveTalon->Set(ControlMode::Follower, 4);


	deltaDistance = sqrt(pow(_xDistance, 2) + pow(_yDistance, 2));
	angleForWheel = std::atan2(_yDistance, _xDistance);
	ticks = ConvertFeetToTicks(deltaDistance);
	started = false;
}

void GoDistance::Initialize() {
	isDone = false;
	started = false;
	std::cout << "init!\n";
	RobotMap::swerveSubsystemFLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemFRDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBRDriveTalon->SetSelectedSensorPosition(0, 0, 0);
}

void GoDistance::Execute() {
	if(_xDistance == 0 && _yDistance == 0) {
		isDone = true;
	}
	if(RobotMap::swerveSubsystemBLDriveTalon->GetClosedLoopError(0) > 3000) {
		started = true;
	}
	std::cout << "isDone: " << isDone << "\n";
	for(int i = 0; i < modules->size(); i++) {
		modules->at(i).SetAngle(Rotation2D::fromRadians(angleForWheel), false);
	}

	std::cout << "ticks: "<< ticks << std::endl;
	std::cout << "angle: " << angleForWheel << std::endl;
	//RobotMap::swerveSubsystemFLDriveTalon->Set(ControlMode::Position, ticks);
	//RobotMap::swerveSubsystemFRDriveTalon->Set(ControlMode::Position, ticks);
	//RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::Position, ticks);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::Position, ticks);
	if(started == true) {
		int currentError = RobotMap::swerveSubsystemBLDriveTalon->GetClosedLoopError(0);
		bool isWithinTolerance =  currentError < 325;
		std::cout << "currentError: " << currentError << "\n";
		isDone = isWithinTolerance;
	}
}

bool GoDistance::IsFinished() {
	return isDone;
}

void GoDistance::End() {
	std::cout << "end!\n";
	RobotMap::swerveSubsystemFLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemFRDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBLDriveTalon->SetSelectedSensorPosition(0, 0, 0);
	RobotMap::swerveSubsystemBRDriveTalon->SetSelectedSensorPosition(0, 0, 0);
}

void GoDistance::Interrupted() {
	std::cout << "interupted!\n";
}

int GoDistance::ConvertFeetToTicks(double feet) {
	return feet * RobotMap::TICKS_PER_FOOT_DRIVE;
}
