#include "ClimbRoutine.h"
#include "../Elevator/GoToElevatorPosition.h"
#include "../Climber/ChangeGearbox.h"
#include "../Climber/DeployWings.h"

ClimbRoutine::ClimbRoutine() {
	AddSequential(new ChangeGearbox(true));
	//AddSequential(new DeployWings(true));
}
