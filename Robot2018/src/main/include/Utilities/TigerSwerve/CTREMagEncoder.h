#pragma once

#include "Utilities/Math/Rotation2D.h"
#include "ctre/Phoenix.h"
#include <string>

using namespace ctre::phoenix::motorcontrol;

class CTREMagEncoder {
private:
	can::TalonSRX* m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Rotation2D m_offset;
public:
	CTREMagEncoder(can::TalonSRX *talon);
	virtual ~CTREMagEncoder();
	Rotation2D GetRawAngle() const;
	Rotation2D GetAngle() const;
	int GetRotations() const;
	int GetEncoderTicks(bool overflow = false) const;
	void Calibrate();
	int ConvertAngleToSetpoint(Rotation2D targetAngle);
	int ConvertAngleToEncoderTicks(Rotation2D angle);
	void SetEncoderRaw(int ticks);
};