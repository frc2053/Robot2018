#include "Utilities/TigerSwerve/TigerSwerve.h"
#include "Robot.h"
#define _USE_MATH_DEFINES
#include <math.h>

TigerSwerve::TigerSwerve(std::vector<std::shared_ptr<can::TalonSRX>>& talons, std::vector<std::shared_ptr<rev::CANSparkMax>>& sparks) {

	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	currentYaw = 0;
	modules.reset(new std::vector<SwerveModule>());

	frontLeftDrive = sparks.at(0);
	//frontRightDrive = sparks.at(1);
	//backLeftDrive = sparks.at(2);
	//backRightDrive = sparks.at(3);

	frontLeftRot = talons.at(0);
	//frontRightRot = talons.at(1);
	//backLeftRot = talons.at(2);
	//backRightRot = talons.at(3);

	modules->push_back(SwerveModule(frontLeftDrive, frontLeftRot));
	//modules->push_back(SwerveModule(frontRightDrive, frontRightRot));
	//modules->push_back(SwerveModule(backLeftDrive, backLeftRot));
	//modules->push_back(SwerveModule(backRightDrive, backRightRot));

	angleTimer.reset(new frc::Timer());
	angleTimer->Reset();
	prevFLAngle = Rotation2D::fromDegrees(0);
	prevFRAngle = Rotation2D::fromDegrees(0);
	prevBLAngle = Rotation2D::fromDegrees(0);
	prevBRAngle = Rotation2D::fromDegrees(0);

}

TigerSwerve::~TigerSwerve() {

}

void TigerSwerve::SetCenterOfRotation(double x, double y) {
	//centerOfRotation->SetX(x);
	//centerOfRotation->SetY(y);
}

void TigerSwerve::Drive(double xSpeed, double ySpeed, double rotSpeed, double headingOffset) {
	rotSpeed = rotSpeed * 0.05;
	Translation2D trans(ySpeed, xSpeed);
	Rotation2D rot = Rotation2D::fromDegrees(rotSpeed);
	Rotation2D gyroAngle = Rotation2D::fromDegrees(-headingOffset);
	currentYaw = headingOffset;
	//std::cout << "gyroAngle: " << gyroAngle.getDegrees() << std::endl;
	std::cout << "trans: (" << trans.getX() << ", " << trans.getY() << ")" << std::endl;
	trans = trans.rotateBy(gyroAngle);
	//std::cout << "trans: (" << trans.getX() << ", " << trans.getY() << ")" <<std::endl;


	double flWheelSpeed;
	double frWheelSpeed;
	double blWheelSpeed;
	double brWheelSpeed;
	Rotation2D flWheelAngle;
	Rotation2D frWheelAngle;
	Rotation2D blWheelAngle;
	Rotation2D brWheelAngle;

	SwerveInverseKinematics(trans, rotSpeed, frWheelSpeed, flWheelSpeed, brWheelSpeed, blWheelSpeed, flWheelAngle, frWheelAngle, blWheelAngle, brWheelAngle);
	angleTimer->Reset();
	angleTimer->Start();

	modules->at(0).Set(flWheelSpeed, flWheelAngle, true);
	//modules->at(1).Set(frWheelSpeed, frWheelAngle, true);
	//modules->at(2).Set(blWheelSpeed, blWheelAngle, true);
	//modules->at(3).Set(brWheelSpeed, brWheelAngle, true);


	prevFLAngle = flWheelAngle;
	prevFRAngle = frWheelAngle;
	prevBLAngle = blWheelAngle;
	prevBRAngle = brWheelAngle;
}

void TigerSwerve::SetBrakeMode() {
	for(int i = 0; i < (signed) modules->size(); i++) {
		//modules->at(i).Set(modules->at(i).GetLocation().GetAngle(), 0);
	}
}

double TigerSwerve::deg2rad(double deg) {
	return deg * M_PI / 180.0;
}

void TigerSwerve::DriveRobotOriented(double x, double y, double rotation) {
	Drive(x, y, rotation, 0);
}

void TigerSwerve::DriveFieldOriented(double x, double y, double rotation, double gyro) {
	Drive(x, y, rotation, gyro);
}

std::shared_ptr<std::vector<SwerveModule>> TigerSwerve::GetModules() {
	return modules;
}

void TigerSwerve::SwerveInverseKinematics(Translation2D &translation,
		double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
		Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR) {

	double A = translation.getX() - rotation * (RobotMap::WHEELBASE_LENGTH / 2.0);
	double B = translation.getX() + rotation * (RobotMap::WHEELBASE_LENGTH / 2.0);
	double C = translation.getY() - rotation * (RobotMap::WHEELBASE_WIDTH / 2.0);
	double D = translation.getY() + rotation * (RobotMap::WHEELBASE_WIDTH / 2.0);
	wheelSpeedFL = sqrt(pow(A, 2) + pow(D, 2)); //sqrt(pow(B, 2) + pow(D, 2));
	wheelSpeedFR = sqrt(pow(B, 2) + pow(C, 2)); //sqrt(pow(B, 2) + pow(C, 2));
	wheelSpeedBR = sqrt(pow(B, 2) + pow(D, 2)); //sqrt(pow(A, 2) + pow(D, 2));
	wheelSpeedBL = sqrt(pow(A, 2) + pow(C, 2)); //sqrt(pow(A, 2) + pow(C, 2));
	wheelAngleFL = Rotation2D(A,C,true); //Rotation2D(B,D,true);
	wheelAngleFR = Rotation2D(B,C,true); //Rotation2D(B,C,true);
	wheelAngleBL = Rotation2D(A,D,true); //Rotation2D(A,D,true);
	wheelAngleBR = Rotation2D(B,D,true); //Rotation2D(A,C,true);

	double maxWheelSpeed = std::max(wheelSpeedFL,std::max(wheelSpeedFR,std::max(wheelSpeedBL,wheelSpeedBR)));
	if (maxWheelSpeed > 1) {
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
	}
}
