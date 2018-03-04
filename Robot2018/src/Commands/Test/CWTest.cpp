#include <Commands/Test/CWTest.h>

CWTest::CWTest() {
	Requires(Robot::swerveSubsystem.get());
	testsPassed = false;
}

void CWTest::Initialize() {
	testsPassed = false;
}

void CWTest::Execute() {
	//test rotations
	for(int i = 0; i <= 315; i = i + 45) {
		for(int j = 0; j < 4; j++) {
			Robot::swerveSubsystem->GetSwerveStuff()->GetModules()->at(j).Set(0, Rotation2D::fromDegrees(i), true);
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	testsPassed = true;
}

bool CWTest::IsFinished() {
	return testsPassed;
}

void CWTest::End() {
	std::cout << "ended" << std::endl;
}

void CWTest::Interrupted() {

}
