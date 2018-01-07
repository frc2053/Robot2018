#pragma once

#include <math.h>
#include <Util/Math/Rotation2D.h>
#include "ctre/Phoenix.h"
#include "CTREMagEncoder.h"

class SwerveModule {
public:
	SwerveModule(std::shared_ptr<can::TalonSRX> driveController, std::shared_ptr<can::TalonSRX> rotateController);
	virtual ~SwerveModule();

	Rotation2D GetAngle() const;
	void SetAngle(Rotation2D angle);
	void Set(double speed, Rotation2D angle);
	void Stop();
private:
	std::shared_ptr<can::TalonSRX> _driveController;
	std::shared_ptr<can::TalonSRX> _rotateController;
	std::shared_ptr<CTREMagEncoder> _angleEncoder;
	bool isOptimizedAngle;
};
