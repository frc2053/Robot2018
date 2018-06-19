#pragma once

#include <Commands/CommandGroup.h>

class ClimbRoutine : public frc::CommandGroup {
public:
	ClimbRoutine(bool gearbox, bool wings);
};

