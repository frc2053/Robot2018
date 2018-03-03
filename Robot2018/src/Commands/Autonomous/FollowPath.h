#ifndef FollowPath_H
#define FollowPath_H

#include "Commands/Command.h"
#include "pathfinder.h"

class FollowPath : public frc::Command {
public:
	FollowPath(Segment* inputPath, int length, int offset);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	int pathLength;
	Segment* pathToFollow;
	//this keeps track of the motor encoder and tells the motor where it should be
	EncoderFollower* flFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* frFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* blFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* brFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	EncoderFollower* lFollower = (EncoderFollower*) malloc(sizeof(EncoderFollower));
	EncoderFollower* rFollower = (EncoderFollower*) malloc(sizeof(EncoderFollower));
	//this holds the modified trajectories for the swerve drive
	Segment* flTraj;
	Segment* frTraj;
	Segment* blTraj;
	Segment* brTraj;
	Segment* lTraj;
	Segment* rTraj;
	//this holds the encoder info
	EncoderConfig flconfig;
	EncoderConfig frconfig;
	EncoderConfig blconfig;
	EncoderConfig brconfig;
	EncoderConfig lconfig;
	EncoderConfig rconfig;
	int angleOffset;
};

#endif
