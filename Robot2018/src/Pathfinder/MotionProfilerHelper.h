#ifndef SRC_PATHFINDER_MOTIONPROFILERHELPER_H_
#define SRC_PATHFINDER_MOTIONPROFILERHELPER_H_

#include <ctre/Phoenix.h>
#include <pathfinder.h>

class MotionProfilerHelper {
public:
	MotionProfilerHelper();
	virtual ~MotionProfilerHelper();
	void ConfigureFPID(double kF, double kP, double kI, double kD);
	void ConfigureTicksPerRev(unsigned int ticksPerRev);
	void ConfigureWheelDiameter(double diameter);
	void ConfigureMpUpdateRate(unsigned int milliseconds);
	bool LoadPathfinder(Segment* segments, int length);
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

	//motion profiling stuff
	unsigned int _ticksPerRev, _mpLoadOffset = 0;
	double wheelDiameter;
};

#endif
