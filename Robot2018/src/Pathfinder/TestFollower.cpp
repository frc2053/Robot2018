#include <Pathfinder/TestFollower.h>

TestFollower::TestFollower() {
	WHEELBASE_WIDTH = RobotMap::WHEELBASE_WIDTH;
	WHEELBASE_LENGTH = RobotMap::WHEELBASE_LENGTH;
	frontLeft = NULL;
	frontRight = NULL;
	backLeft = NULL;
	backRight = NULL;
	trajectory = NULL;
	length = 0;
}

void TestFollower::Generate() {
	modules = RobotMap::tigerSwerve->GetModules();
	Waypoint points[POINT_LENGTH];
	Waypoint p1 = {0, 0, d2r(0)};
	Waypoint p2 = {5, 0, d2r(0)};
	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VEL, MAX_ACCEL, MAX_JERK, &candidate);
	length = candidate.length;
	trajectory = (Segment*)malloc(length * sizeof(Segment));
	pathfinder_generate(&candidate, trajectory);

	//Modify for swerve
	frontLeft = (Segment*)malloc(length * sizeof(Segment));
	frontRight = (Segment*)malloc(length * sizeof(Segment));
	backLeft = (Segment*)malloc(length * sizeof(Segment));
	backRight = (Segment*)malloc(length * sizeof(Segment));

	SWERVE_MODE mode = SWERVE_DEFAULT;

	pathfinder_modify_swerve(trajectory, length, frontLeft, frontRight, backLeft, backRight, WHEELBASE_WIDTH, WHEELBASE_LENGTH, mode);

	FILE* fp = fopen("/home/lvuser/myfile.csv", "w");
	pathfinder_serialize_csv(fp, trajectory, length);
	fclose(fp);

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

void TestFollower::FollowPath() {

	int flCurrentPos = RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0);
	int frCurrentPos = RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0);
	int blCurrentPos = RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0);
	int brCurrentPos = RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0);

	double fl = pathfinder_follow_encoder(flconfig, flFollower, frontLeft, length, flCurrentPos);
	double fr = pathfinder_follow_encoder(frconfig, frFollower, frontRight, length, frCurrentPos);
	double bl = pathfinder_follow_encoder(blconfig, blFollower, backLeft, length, blCurrentPos);
	double br = pathfinder_follow_encoder(brconfig, brFollower, backRight, length, brCurrentPos);

	/*double desired_headingfl = r2d(K_T * (d2r(Robot::swerveSubsystem->GetAdjYaw() - flFollower->heading)));
	double desired_headingfr = r2d(K_T * (d2r(Robot::swerveSubsystem->GetAdjYaw() - frFollower->heading)));
	double desired_headingbl = r2d(K_T * (d2r(Robot::swerveSubsystem->GetAdjYaw() - blFollower->heading)));
	double desired_headingbr = r2d(K_T * (d2r(Robot::swerveSubsystem->GetAdjYaw() - brFollower->heading)));*/

	double desired_headingfl = r2d(flFollower->heading);
	double desired_headingfr = r2d(frFollower->heading);
	double desired_headingbl = r2d(blFollower->heading);
	double desired_headingbr = r2d(brFollower->heading);

	//SmartDashboard::PutNumber("FL ERROR", flFollower->last_error);
	//SmartDashboard::PutNumber("FR ERROR", frFollower->last_error);
	//SmartDashboard::PutNumber("BL ERROR", blFollower->last_error);
	//SmartDashboard::PutNumber("BR ERROR", brFollower->last_error);

	modules->at(0).Set(fl, Rotation2D::fromDegrees(desired_headingfl), false);
	modules->at(1).Set(fr, Rotation2D::fromDegrees(desired_headingfr), false);
	modules->at(2).Set(bl, Rotation2D::fromDegrees(desired_headingbl), false);
	modules->at(3).Set(br, Rotation2D::fromDegrees(desired_headingbr), false);
}

TestFollower::~TestFollower() {
	free(trajectory);
	free(frontLeft);
	free(frontRight);
	free(backLeft);
	free(backRight);
	free(flFollower);
	free(frFollower);
	free(blFollower);
	free(brFollower);
}

