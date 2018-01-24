#include <Pathfinder/TestFollower.h>



TestFollower::TestFollower() {
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

	helper.ConfigureFPID(K_F, K_P, 0, 0);
	helper.ConfigureMpUpdateRate(10);
	helper.PushTrajectoryToTalons(helper.convertToSRXTrajectory(trajectory, length));
}

void TestFollower::FollowPath() {
	helper.PrintStatus(helper.ProcessMp(), 0, 0, 0);
}

TestFollower::~TestFollower() {

}

