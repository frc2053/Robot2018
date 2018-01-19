#ifndef SRC_PATHFINDER_TESTFOLLOWER_H_
#define SRC_PATHFINDER_TESTFOLLOWER_H_

#include "../RobotMap.h"
#include "../Robot.h"

class TestFollower {
public:
	TestFollower();
	virtual ~TestFollower();
	void Generate();
	void FollowPath();
private:
	//ALL IN FEET PLEASE!!!
	int POINT_LENGTH = 2;
	const double TIMESTEP = 0.01;
	const double MAX_VEL = 18;
	const double MAX_ACCEL = 18;
	const double MAX_JERK = 60;
	const int TICKS_PER_REV = 26214;
	const double WHEEL_CIRCUMFERENCE = 0.65449867893738;
	const double K_P = 1;
	const double K_I = 0.0;
	const double K_D = 0;
	const double K_V = .66;
	const double K_A = .4; //0.0856
	const double K_T = 0;
	double WHEELBASE_WIDTH;
	double WHEELBASE_LENGTH;
	TrajectoryCandidate candidate;
	Segment* trajectory;
	Segment* frontLeft;
	Segment* frontRight;
	Segment* backLeft;
	Segment* backRight;
	int length;
	std::shared_ptr<std::vector<SwerveModule>> modules;
	EncoderFollower* flFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* frFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* blFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* brFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderConfig flconfig = {0, TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	EncoderConfig frconfig = {0, TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	EncoderConfig blconfig = {0, TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	EncoderConfig brconfig = {0, TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
};

#endif
