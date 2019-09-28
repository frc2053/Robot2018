#pragma once

#include <math.h>
#include "Utilities/Math/Rotation2D.h"
#include "ctre/Phoenix.h"
#include "CTREMagEncoder.h"
#include <rev/CANSparkMax.h>


class SwerveModule {
public:
	SwerveModule(std::shared_ptr<rev::CANSparkMax> driveController, std::shared_ptr<can::TalonSRX> rotateController);
	virtual ~SwerveModule();

	Rotation2D GetAngle() const;
	void SetAngle(Rotation2D angle, bool doOptimization);
	void Set(double speed, Rotation2D angle, bool doOptimization);
	void Stop();
private:
	std::shared_ptr<rev::CANSparkMax> _driveController;
	std::shared_ptr<can::TalonSRX> _rotateController;
	std::shared_ptr<CTREMagEncoder> _angleEncoder;
	bool isOptimizedAngle;
};