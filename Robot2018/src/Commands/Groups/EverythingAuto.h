#pragma once

#include <Commands/CommandGroup.h>

class EverythingAuto : public frc::CommandGroup {
public:
	EverythingAuto(char switchSide, char scaleSide, char robotSide, bool doScale);
};

