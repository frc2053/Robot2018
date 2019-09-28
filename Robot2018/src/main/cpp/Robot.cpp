#include "Robot.h"
#include "Commands/Drive/DriveCommand.h"


std::unique_ptr<OI> Robot::oi;
std::unique_ptr<SwerveSubsystem> Robot::swerveSubsystem;


void Robot::RobotInit() {
	std::cout << "Robot is starting!" << std::endl;
	RobotMap::init();

	swerveSubsystem.reset(new SwerveSubsystem());
	oi.reset(new OI());

	//calibrate gyro
	Robot::swerveSubsystem->ZeroYaw();
	//if we start at an angle other than zero change this in auto
	Robot::swerveSubsystem->SetAdjYaw(0);
}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	SmartDashboard::PutData("Scheduler", Scheduler::GetInstance());
	Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	std::cout << "Autonomous Init!" << std::endl;
	Robot::swerveSubsystem->CalibrateWheelsSimple();
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;

	Robot::swerveSubsystem->CalibrateWheelsSimple();

	Robot::swerveSubsystem->SetDefaultCommand(new DriveCommand());
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {

}

int main()
{
	return frc::StartRobot<Robot>();
}