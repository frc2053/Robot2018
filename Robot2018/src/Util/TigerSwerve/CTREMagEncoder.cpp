#include <sstream>
#include "CTREMagEncoder.h"
#include <WPILib.h>

CTREMagEncoder::CTREMagEncoder(can::TalonSRX* talon) : m_talon(talon) {
}

CTREMagEncoder::~CTREMagEncoder() {
}

Rotation2D CTREMagEncoder::GetRawAngle() const {
	return Rotation2D::fromRadians(GetEncoderTicks(false) / 4096.0 * 2 * M_PI);
}

Rotation2D CTREMagEncoder::GetAngle() const {
	//return m_offset.rotateBy(GetRawAngle());
	return GetRawAngle();
}

int CTREMagEncoder::GetRotations() const {
	return GetEncoderTicks(true) / 4096;
}

int CTREMagEncoder::GetEncoderTicks(bool overflow) const {
	int ticks = m_talon->GetSelectedSensorPosition(0);
	ticks = ticks * -1;
	if (!overflow) {
		ticks = ticks & 0xFFF;
	}
	return ticks;
}

void CTREMagEncoder::Calibrate() {
	m_offset = GetRawAngle().inverse();
	Preferences::GetInstance()->PutDouble(m_calibrationKey, m_offset.getDegrees());
}

int CTREMagEncoder::ConvertAngleToSetpoint(Rotation2D targetAngle) {
	Rotation2D angle = targetAngle.rotateBy(m_offset);
	int ticks = ConvertAngleToEncoderTicks(angle); // 0 - 4096

	int encoderTicks = GetEncoderTicks(true);
	ticks = ticks + (encoderTicks/4096)*4096;

	int error = encoderTicks - ticks;

	if (error < -2048) {
		ticks = ticks - 4096;
	}
	else if (error > 2048) {
		ticks = ticks + 4096;
	}

	return ticks;
}

int CTREMagEncoder::ConvertAngleToEncoderTicks(Rotation2D angle) {
	double degrees = angle.getDegrees();
	int ticks = degrees / 360.0 * 4096;
	return ticks;
}
