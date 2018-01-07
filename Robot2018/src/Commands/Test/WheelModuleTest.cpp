#include "WheelModuleTest.h"

WheelModuleTest::WheelModuleTest(double setPoint) {
	//Requires(Robot::swerveSubsystem.get());
	testsPassed = false;
	talonSetpoint = setPoint;
}

void WheelModuleTest::Initialize() {
	testsPassed = false;
}

void WheelModuleTest::Execute() {
	//test rotations
	for(int i = 0; i < 365; i = i + 45) {
		for(int j = 0; j < 4; j++) {
			Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(j).Set(0, Rotation2D::fromDegrees(i));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	/*RobotMap::swerveSubsystemBackLeftRotationTalon->SetControlMode(CANTalon::kPercentVbus);
	RobotMap::swerveSubsystemBackRightRotationTalon->SetControlMode(CANTalon::kPercentVbus);
	RobotMap::swerveSubsystemFrontLeftRotationTalon->SetControlMode(CANTalon::kPercentVbus);
	RobotMap::swerveSubsystemFrontRightRotationTalon->SetControlMode(CANTalon::kPercentVbus);
	RobotMap::swerveSubsystemBackLeftRotationTalon->Set(.15);
	RobotMap::swerveSubsystemBackRightRotationTalon->Set(.15);
	RobotMap::swerveSubsystemFrontLeftRotationTalon->Set(.15);
	RobotMap::swerveSubsystemFrontRightRotationTalon->Set(.15);*/

	testsPassed = true;
}

bool WheelModuleTest::IsFinished() {
	return testsPassed;
}

void WheelModuleTest::End() {

}

void WheelModuleTest::Interrupted() {

}
