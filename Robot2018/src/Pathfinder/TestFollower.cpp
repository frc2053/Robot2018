#include <Pathfinder/TestFollower.h>
#include "../Robot.h"

TestFollower::TestFollower() {
	WHEELBASE_WIDTH = RobotMap::WHEELBASE_WIDTH;
	WHEELBASE_LENGTH = RobotMap::WHEELBASE_LENGTH;
	flTraj = NULL;
	frTraj = NULL;
	blTraj = NULL;
	brTraj = NULL;
	trajectory = NULL;
	length = 0;
}

void TestFollower::Generate() {
	modules = RobotMap::tigerSwerve->GetModules();
	Waypoint points[POINT_LENGTH];
	Waypoint p1 = {0, 0, d2r(0)};
	Waypoint p2 = {12, 0, d2r(0)};
	//Waypoint p3 = {10, -4, d2r(0)};
	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	int trajLength = pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VEL, MAX_ACCEL, MAX_JERK, &candidate);
	std::cout << "trajLength: " << trajLength << "\n";
	length = candidate.length;
	trajectory = (Segment*)malloc(length * sizeof(Segment));
	pathfinder_generate(&candidate, trajectory);

	//Modify for swerve
	flTraj = (Segment*)malloc(length * sizeof(Segment));
	frTraj = (Segment*)malloc(length * sizeof(Segment));
	blTraj = (Segment*)malloc(length * sizeof(Segment));
	brTraj = (Segment*)malloc(length * sizeof(Segment));

	SWERVE_MODE mode = SWERVE_DEFAULT;

	pathfinder_modify_swerve(trajectory, length, flTraj, frTraj, blTraj, brTraj, WHEELBASE_WIDTH, WHEELBASE_LENGTH, mode);

	FILE* fp = fopen("/home/lvuser/myfile.csv", "w");
	pathfinder_serialize_csv(fp, trajectory, length);
	fclose(fp);
}

void TestFollower::ConfigureEncoders() {
	flconfig = {RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	frconfig = {RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	blconfig = {RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	brconfig = {RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};

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
}

bool TestFollower::FollowPath() {

	int flCurrentPos = RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0);
	int frCurrentPos = RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0);
	int blCurrentPos = RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0);
	int brCurrentPos = RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0);

	double fl = pathfinder_follow_encoder(flconfig, flFollower, flTraj, length, flCurrentPos);
	double fr = pathfinder_follow_encoder(frconfig, frFollower, frTraj, length, frCurrentPos);
	double bl = pathfinder_follow_encoder(blconfig, blFollower, blTraj, length, blCurrentPos);
	double br = pathfinder_follow_encoder(brconfig, brFollower, brTraj, length, brCurrentPos);

	double currentYaw = Robot::swerveSubsystem->GetAdjYaw();

	double desired_headingfl = r2d(flFollower->heading);
	double desired_headingfr = r2d(frFollower->heading);
	double desired_headingbl = r2d(blFollower->heading);
	double desired_headingbr = r2d(brFollower->heading);

	double angle_difference = r2d(flFollower->heading) - currentYaw;
	double turn = K_T * angle_difference;

	//SmartDashboard::PutNumber("FL ERROR", flFollower->last_error);
	//SmartDashboard::PutNumber("FR ERROR", frFollower->last_error);
	//SmartDashboard::PutNumber("BL ERROR", blFollower->last_error);
	//SmartDashboard::PutNumber("BR ERROR", brFollower->last_error);

	modules->at(0).Set(fl + turn, Rotation2D::fromDegrees(desired_headingfl), false);
	modules->at(1).Set(fr - turn, Rotation2D::fromDegrees(desired_headingfr), false);
	modules->at(2).Set(bl + turn, Rotation2D::fromDegrees(desired_headingbl), false);
	modules->at(3).Set(br - turn, Rotation2D::fromDegrees(desired_headingbr), false);

	bool retVal = flFollower->segment >= length;
	return retVal;
}

void TestFollower::StopFollowing() {
	modules->at(0).Set(0, Rotation2D::fromDegrees(0), true);
	modules->at(1).Set(0, Rotation2D::fromDegrees(0), true);
	modules->at(2).Set(0, Rotation2D::fromDegrees(0), true);
	modules->at(3).Set(0, Rotation2D::fromDegrees(0), true);
}

TestFollower::~TestFollower() {
	free(trajectory);
	free(flTraj);
	free(frTraj);
	free(blTraj);
	free(brTraj);
	free(flFollower);
	free(frFollower);
	free(blFollower);
	free(brFollower);
}
