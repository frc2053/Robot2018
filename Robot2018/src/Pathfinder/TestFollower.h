#ifndef SRC_PATHFINDER_TESTFOLLOWER_H_
#define SRC_PATHFINDER_TESTFOLLOWER_H_

#include "../RobotMap.h"

class TestFollower {
public:
	TestFollower();
	virtual ~TestFollower();
	void Generate();
	void FollowPath();
	void ConfigureEncoders();
private:
	//ALL IN FEET PLEASE!!!
	int POINT_LENGTH = 2;
	const double TIMESTEP = 0.02;
	const double MAX_VEL = 18;
	const double MAX_ACCEL = 12;
	const double MAX_JERK = 60;
	const int TICKS_PER_REV = 26214;
	const double WHEEL_CIRCUMFERENCE = 0.65449867893738;
	const double K_P = 1;
	const double K_I = 0.0;
	const double K_D = .15;
	const double K_V = .06;//.66;
	const double K_A = 0.0856;
	const double K_T = 1;
	double WHEELBASE_WIDTH;
	double WHEELBASE_LENGTH;
	TrajectoryCandidate candidate;
	Segment* trajectory;
	Segment* flTraj;
	Segment* frTraj;
	Segment* blTraj;
	Segment* brTraj;
	int length;
	std::shared_ptr<std::vector<SwerveModule>> modules;
	EncoderFollower* flFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* frFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* blFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* brFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderConfig flconfig;
	EncoderConfig frconfig;
	EncoderConfig blconfig;
	EncoderConfig brconfig;
};

#endif
