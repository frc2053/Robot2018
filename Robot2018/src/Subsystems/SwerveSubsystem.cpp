#include "SwerveSubsystem.h"
#include "../Commands/Drive/DriveCommand.h"

SwerveSubsystem::SwerveSubsystem() : Subsystem("SwerveSubsystem") {
	std::cout << "Constructor for Swerve Subsystem!" << std::endl;
	frontLeftDriveTalon = RobotMap::swerveSubsystemFLDriveTalon;
	frontRightDriveTalon = RobotMap::swerveSubsystemFRDriveTalon;
	backLeftDriveTalon = RobotMap::swerveSubsystemBLDriveTalon;
	backRightDriveTalon = RobotMap::swerveSubsystemBRDriveTalon;

	frontLeftRotationTalon = RobotMap::swerveSubsystemFLRotTalon;
	frontRightRotationTalon = RobotMap::swerveSubsystemFRRotTalon;
	backLeftRotationTalon = RobotMap::swerveSubsystemBLRotTalon;
	backRightRotationTalon = RobotMap::swerveSubsystemBRRotTalon;

	tigerSwerve = RobotMap::tigerSwerve;
	tigerDrive = RobotMap::tigerDrive;
}

void SwerveSubsystem::CalibrateWheels() {

	/*
	 * Current mod: Set quad to value of PWM, rotate to CAL values, re-zero quad after set
	 */

	frontLeftRotationTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	frontRightRotationTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	backLeftRotationTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	backRightRotationTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	//frontLeftRotationTalon->SetSensorPhase(true);
	//frontRightRotationTalon->SetSensorPhase(true);
	//backLeftRotationTalon->SetSensorPhase(true);
	//backRightRotationTalon->SetSensorPhase(true);

	frontLeftRotationTalon->SetSelectedSensorPosition(0,0,10);
	frontRightRotationTalon->SetSelectedSensorPosition(0,0,10);
	backLeftRotationTalon->SetSelectedSensorPosition(0,0,10);
	backRightRotationTalon->SetSelectedSensorPosition(0,0,10);

	int currentPWMfl = frontLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int currentPWMfr = frontRightRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int currentPWMbl = backLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int currentPWMbr = backRightRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;

	std::cout << "start: currentPWMfl: " << currentPWMfl << std::endl;
	std::cout << "start: currentPWMfr: " << currentPWMfr << std::endl;
	std::cout << "start: currentPWMbl: " << currentPWMbl << std::endl;
	std::cout << "start: currentPWMbr: " << currentPWMbr << std::endl;


	int currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	int currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	int currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	int currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "start: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "start: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "start: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "start: currentQuadbr: " << currentQuadbr << std::endl;

	int selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	int selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	int selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	int selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "start: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "start: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "start: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "start: SelectedSensorbr: " << selSensorbr << std::endl;

	std::cout << "---------------------" << std::endl;

	frontLeftRotationTalon->SetSelectedSensorPosition(currentPWMfl,0,10);
	frontRightRotationTalon->SetSelectedSensorPosition(currentPWMfr,0,10);
	backLeftRotationTalon->SetSelectedSensorPosition(currentPWMbl,0,10);
	backRightRotationTalon->SetSelectedSensorPosition(currentPWMbr,0,10);

	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "new: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "new: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "new: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "new: currentQuadbr: " << currentQuadbr << std::endl;

	selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "new: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "new: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "new: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "new: SelectedSensorbr: " << selSensorbr << std::endl;

/*
	int absModfl = AbsMod(currentQuadfl, TICKS_PER_REV);
	int absModfr = AbsMod(currentQuadfr, TICKS_PER_REV);
	int absModbl = AbsMod(currentQuadbl, TICKS_PER_REV);
	int absModbr = AbsMod(currentQuadbr, TICKS_PER_REV);

	std::cout << "absModfl: " << absModfl << std::endl;
	std::cout << "absModfr: " << absModfr << std::endl;
	std::cout << "absModbl: " << absModbl << std::endl;
	std::cout << "absModbr: " << absModbr << std::endl;
*/

	std::cout << "FL_CAL: " << FL_CAL << std::endl;
	std::cout << "FR_CAL: " << FR_CAL << std::endl;
	std::cout << "BL_CAL: " << BL_CAL << std::endl;
	std::cout << "BR_CAL: " << BR_CAL << std::endl;

/*
	int flSetpoint = ((absModfl) % TICKS_PER_REV) + (FL_CAL - currentPWMfl);
	int frSetpoint = ((absModfr) % TICKS_PER_REV) + (FR_CAL - currentPWMfr);
	int blSetpoint = ((absModbl) % TICKS_PER_REV) + (BL_CAL - currentPWMbl);
    int brSetpoint = ((absModbr) % TICKS_PER_REV) + (BR_CAL - currentPWMbr);
*/

    int flSetpoint = FL_CAL;
	int frSetpoint = FR_CAL;
	int blSetpoint = BL_CAL;
	int brSetpoint = BR_CAL;

	std::cout << "flSetpoint: " << flSetpoint << std::endl;
	std::cout << "frSetpoint: " << frSetpoint << std::endl;
	std::cout << "blSetpoint: " << blSetpoint << std::endl;
	std::cout << "brSetpoint: " << brSetpoint << std::endl;

/*
	flSetpoint = (SwerveSubsystem::OptimizeRot(flSetpoint, TICKS_PER_REV));
	frSetpoint = (SwerveSubsystem::OptimizeRot(frSetpoint, TICKS_PER_REV));
	blSetpoint = (SwerveSubsystem::OptimizeRot(blSetpoint, TICKS_PER_REV));
	brSetpoint = (SwerveSubsystem::OptimizeRot(brSetpoint, TICKS_PER_REV));


	std::cout << "OptflSetpoint: " << flSetpoint << std::endl;
	std::cout << "OptfrSetpoint: " << frSetpoint << std::endl;
	std::cout << "OptblSetpoint: " << blSetpoint << std::endl;
	std::cout << "OptbrSetpoint: " << brSetpoint << std::endl;
*/


	frontLeftRotationTalon->Set(ControlMode::Position, flSetpoint);
	frontRightRotationTalon->Set(ControlMode::Position, frSetpoint);
	backLeftRotationTalon->Set(ControlMode::Position, blSetpoint);
	backRightRotationTalon->Set(ControlMode::Position, brSetpoint);

	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "1: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "1: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "1: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "1: currentQuadbr: " << currentQuadbr << std::endl;

	selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "1: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "1: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "1: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "1: SelectedSensorbr: " << selSensorbr << std::endl;
	std::cout << "---------------------" << std::endl;
	//frc::Wait(3);

	//frontLeftRotationTalon->Set(ControlMode::Position, flSetpoint);
	//frontRightRotationTalon->Set(ControlMode::Position, frSetpoint);
	//backLeftRotationTalon->Set(ControlMode::Position, blSetpoint);
	//backRightRotationTalon->Set(ControlMode::Position, brSetpoint);

	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "2: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "2: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "2: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "2: currentQuadbr: " << currentQuadbr << std::endl;

	selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "2: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "2: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "2: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "2: SelectedSensorbr: " << selSensorbr << std::endl;
	std::cout << "---------------------" << std::endl;


	//frc::Wait(3);

	//frontLeftRotationTalon->Set(ControlMode::Position, flSetpoint);
	//frontRightRotationTalon->Set(ControlMode::Position, frSetpoint);
	//backLeftRotationTalon->Set(ControlMode::Position, blSetpoint);
	//backRightRotationTalon->Set(ControlMode::Position, brSetpoint);
	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "3: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "3: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "3: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "3: currentQuadbr: " << currentQuadbr << std::endl;

	selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "3: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "3: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "3: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "3: SelectedSensorbr: " << selSensorbr << std::endl;
	std::cout << "---------------------" << std::endl;

	//frc::Wait(3);

	//frontLeftRotationTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//frontRightRotationTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//backLeftRotationTalon->GetSensorCollection().SetQuadraturePosition(0, 10);
	//backRightRotationTalon->GetSensorCollection().SetQuadraturePosition(0, 10);

	frontLeftRotationTalon->SetSelectedSensorPosition(0,0,10);
	frontRightRotationTalon->SetSelectedSensorPosition(0,0,10);
	backLeftRotationTalon->SetSelectedSensorPosition(0,0,10);
	backRightRotationTalon->SetSelectedSensorPosition(0,0,10);

	currentPWMfl = frontLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	currentPWMfr = frontRightRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	currentPWMbl = backLeftRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	currentPWMbr = backRightRotationTalon->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;

	std::cout << "end: currentPWMfl: " << currentPWMfl << std::endl;
	std::cout << "end: currentPWMfr: " << currentPWMfr << std::endl;
	std::cout << "end: currentPWMbl: " << currentPWMbl << std::endl;
	std::cout << "end: currentPWMbr: " << currentPWMbr << std::endl;

	currentQuadfl = frontLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadfr = frontRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbl = backLeftRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;
	currentQuadbr = backRightRotationTalon->GetSensorCollection().GetQuadraturePosition() & 0xFFF;

	std::cout << "end: currentQuadfl: " << currentQuadfl << std::endl;
	std::cout << "end: currentQuadfr: " << currentQuadfr << std::endl;
	std::cout << "end: currentQuadbl: " << currentQuadbl << std::endl;
	std::cout << "end: currentQuadbr: " << currentQuadbr << std::endl;

	selSensorfl = frontLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorfr = frontRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbl = backLeftRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;
	selSensorbr = backRightRotationTalon->GetSelectedSensorPosition(0) & 0xFFF;

	std::cout << "end: SelectedSensorfl: " << selSensorfl << std::endl;
	std::cout << "end: SelectedSensorfr: " << selSensorfr << std::endl;
	std::cout << "end: SelectedSensorbl: " << selSensorbl << std::endl;
	std::cout << "end: SelectedSensorbr: " << selSensorbr << std::endl;

	std::cout << "Calibrated Wheels!" << std::endl;

}

int SwerveSubsystem::AbsMod(int value, int ticks) {
	int retVal = value;
	if(value < 0) {
		retVal = ticks - abs(value);
	}
	return retVal;
}

void SwerveSubsystem::InitDefaultCommand() {
	//SetDefaultCommand(new DriveCommand());
}

double SwerveSubsystem::CalculateRotValue(double setAngle, double setSpeed)
{
	return tigerDrive->CalculateRotationValue(setAngle, setSpeed);
}

bool SwerveSubsystem::GetIsRotDone()
{
	return tigerDrive->GetIsRotDone();
}

double SwerveSubsystem::GetAdjYaw() {
	return tigerDrive->GetAdjYaw();
}

void SwerveSubsystem::SetAdjYaw(double yaw) {
	tigerDrive->SetAdjYaw(yaw);
}

void SwerveSubsystem::SetIsRotDone(bool isDone)
{
	tigerDrive->SetIsRotDone(isDone);
}

void SwerveSubsystem::SetIsRotDoneOverride(bool isDone)
{
	tigerDrive->SetIsRotDoneOverride(isDone);
}

void SwerveSubsystem::SetTimesThroughLoop(int timeLoop)
{
	tigerDrive->SetTimesThroughLoop(timeLoop);
}

bool SwerveSubsystem::GetIsRotDoneOverride() {
	return tigerDrive->GetIsRotDoneOverride();
}

void SwerveSubsystem::ZeroYaw() {
	tigerDrive->imu->ZeroYaw();
}

void SwerveSubsystem::SwerveDrive(double xAxis, double yAxis, double rotAxis, double currentYaw) {
	tigerSwerve->DriveFieldOriented(xAxis, yAxis, rotAxis, currentYaw);
}

std::shared_ptr<TigerSwerve> SwerveSubsystem::GetSwerveStuff() {
	return tigerSwerve;
}

int SwerveSubsystem::OptimizeRot(int value, int ticks) {
	int retVal = value;

	absVal = abs(value);
	halfTicks =  ticks/2;

	if(absVal > halfTicks) {
		retVal = (ticks - absVal);
	}

	if (value > halfTicks) {
		retVal = retVal * -1;
	}

	return retVal;
}
