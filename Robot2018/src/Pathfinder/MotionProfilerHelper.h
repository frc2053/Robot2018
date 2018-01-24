#ifndef SRC_PATHFINDER_MOTIONPROFILERHELPER_H_
#define SRC_PATHFINDER_MOTIONPROFILERHELPER_H_

#include <ctre/Phoenix.h>
#include <pathfinder.h>

class MotionProfilerHelper {
public:
	MotionProfilerHelper();
	virtual ~MotionProfilerHelper();
	void ConfigureFPID(double kF, double kP, double kI, double kD);
	int ConvertFeetToTicks(double pos);
	void ConfigureMpUpdateRate(unsigned int milliseconds);
	std::vector<TrajectoryPoint> convertToSRXTrajectory(Segment* trajectory, int inputLength);
	void PushTrajectoryToTalons(std::vector<TrajectoryPoint> points);
	void ResetMp();
	void EnableMp();
	void HoldMp();
	void DisableMp();
	void PrintStatus(MotionProfileStatus status, double pos, double vel, double heading);
	MotionProfileStatus ProcessMp();
private:
	ControlMode ctrlMode = ControlMode::PercentOutput;
	double val;
	int stStage;
	double TICKS_PER_FOOT = 40053;
	double MAX_VELOCITY = 18;
	//motion profiling stuff
	unsigned int _ticksPerRev, _mpLoadOffset = 0;
	double wheelDiameter;
};

#endif
