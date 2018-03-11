#include "FollowPath.h"
#include "../../Robot.h"

FollowPath::FollowPath(Segment* inputPath, int length, int offset) {

	//std::cout << "MADE IT TO FOLLOWPATH AUTO" << std::endl;

	Requires(Robot::swerveSubsystem.get());
	isDone = false;
	pathLength = length;
	pathToFollow = inputPath;

	angleOffset = offset;

	flFollower->last_error = 0;
	flFollower->segment = 0;
	flFollower->finished = 0;

	frFollower->last_error = 0;
	frFollower->segment = 0;
	frFollower->finished = 0;

	blFollower->last_error = 0;
	blFollower->segment = 0;
	blFollower->finished = 0;

	brFollower->last_error = 0;
	brFollower->segment = 0;
	brFollower->finished = 0;

	lFollower->last_error = 0;
	lFollower->segment = 0;
	lFollower->finished = 0;

	rFollower->last_error = 0;
	rFollower->segment = 0;
	rFollower->finished = 0;

	flTraj = NULL;
	frTraj = NULL;
	blTraj = NULL;
	brTraj = NULL;

	lTraj = NULL;
	rTraj = NULL;


	flconfig = {RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};
	frconfig = {RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};
	blconfig = {RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};
	brconfig = {RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};

	lconfig = {RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};
	rconfig = {RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0), RobotMap::TICKS_PER_REV, RobotMap::WHEEL_CIRCUMFERENCE, RobotMap::K_P, RobotMap::K_I, RobotMap::K_D, RobotMap::K_V, RobotMap::K_A};


	flTraj = (Segment*)malloc(length * sizeof(Segment));
	frTraj = (Segment*)malloc(length * sizeof(Segment));
	blTraj = (Segment*)malloc(length * sizeof(Segment));
	brTraj = (Segment*)malloc(length * sizeof(Segment));

	lTraj = (Segment*) malloc(length * sizeof(Segment));
	rTraj = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_modify_swerve(pathToFollow, pathLength, flTraj, frTraj, blTraj, brTraj, RobotMap::WHEELBASE_WIDTH, RobotMap::WHEELBASE_LENGTH, SWERVE_DEFAULT);
}

void FollowPath::Initialize() {

}

void FollowPath::Execute() {
	std::cout << "MADE IT TO FOLLOWPATH AUTO" << std::endl;


	int flCurrentPos = RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0);
	int frCurrentPos = RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0);
	int blCurrentPos = RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0);
	int brCurrentPos = RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0);

	int lCurrentPos = RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0);
	int rCurrentPos = RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0);

	double fl = pathfinder_follow_encoder(flconfig, flFollower, flTraj, pathLength, flCurrentPos);
	double fr = pathfinder_follow_encoder(frconfig, frFollower, frTraj, pathLength, frCurrentPos);
	double bl = pathfinder_follow_encoder(blconfig, blFollower, blTraj, pathLength, blCurrentPos);
	double br = pathfinder_follow_encoder(brconfig, brFollower, brTraj, pathLength, brCurrentPos);

	double l = pathfinder_follow_encoder(lconfig, lFollower, lTraj, pathLength, lCurrentPos);
	double r = pathfinder_follow_encoder(rconfig, rFollower, rTraj, pathLength, rCurrentPos);

	double currentYaw = Robot::swerveSubsystem->GetAdjYaw();

	//double desired_headingfl = r2d(flFollower->heading);
	//double desired_headingfr = r2d(frFollower->heading);
	//double desired_headingbl = r2d(blFollower->heading);
	//double desired_headingbr = r2d(brFollower->heading);

	double angle_difference = r2d(flFollower->heading) - currentYaw;
	double turn = RobotMap::K_T * angle_difference;

	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(0).Set(fl + turn, Rotation2D::fromDegrees(0), false);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(1).Set(fr - turn, Rotation2D::fromDegrees(0), false);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(2).Set(bl + turn, Rotation2D::fromDegrees(0), false);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(3).Set(br - turn, Rotation2D::fromDegrees(0), false);

	isDone = flFollower->segment >= pathLength;
}

bool FollowPath::IsFinished() {
	return isDone;
}

void FollowPath::End() {
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(0).Set(0, Rotation2D::fromDegrees(0), true);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(1).Set(0, Rotation2D::fromDegrees(0), true);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(2).Set(0, Rotation2D::fromDegrees(0), true);
	Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(3).Set(0, Rotation2D::fromDegrees(0), true);
	//free(flFollower);
	//free(frFollower);
	//free(blFollower);

	//free(flTraj);
	//free(frTraj);
	//free(blTraj);
	//free(brTraj);
	//free(pathToFollow);
}













void FollowPath::Interrupted() {

}
