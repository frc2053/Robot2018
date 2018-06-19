#include "ClimbRoutine.h"
#include "../Elevator/GoToElevatorPosition.h"
#include "../Climber/ChangeGearbox.h"
#include "../Climber/DeployWings.h"

ClimbRoutine::ClimbRoutine(bool gearbox, bool wings) {
	AddSequential(new ChangeGearbox(gearbox));
	AddSequential(new DeployWings(wings));
}
