#include <Pathfinder/TestFollower.h>



TestFollower::TestFollower() {
}

void TestFollower::Generate() {
	RobotMap::swerveSubsystemBLDriveTalon->Config_kF(0, 0.076, kTimeoutMs);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kP(0, 2, kTimeoutMs);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kI(0, 0, kTimeoutMs);
	RobotMap::swerveSubsystemBLDriveTalon->Config_kD(0, 20, kTimeoutMs);
	RobotMap::swerveSubsystemBLDriveTalon->ConfigMotionProfileTrajectoryPeriod(10, kTimeoutMs);
	RobotMap::swerveSubsystemBLDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 10, kTimeoutMs);


}

void TestFollower::FollowPath() {
	_example.control();
	_example.PeriodicTask();

	SetValueMotionProfile setOutput = _example.getSetValue();
	RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::MotionProfile, setOutput);
}

TestFollower::~TestFollower() {

}

