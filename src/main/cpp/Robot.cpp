// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Robot.h"

#include <hal/FRCUsageReporting.h>
#include <cameraserver/CameraServer.h>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
std::shared_ptr<DriveTrain> Robot::driveTrain;
std::shared_ptr<Intake> Robot::intake;
std::shared_ptr<Shooter> Robot::shooter;
std::shared_ptr<Magazine> Robot::magazine;
std::shared_ptr<Climber> Robot::climber;
std::unique_ptr<OI> Robot::oi;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION

void Robot::RobotInit() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
driveTrain.reset(new DriveTrain());
intake.reset(new Intake());
shooter.reset(new Shooter());
magazine.reset(new Magazine());
climber.reset(new Climber());
driveTrain->setLimeLED(false);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their Requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi.reset(new OI());

	HAL_Report(HALUsageReporting::kResourceType_Framework,
		HALUsageReporting::kFramework_RobotBuilder);

	// Add commands to Autonomous Sendable Chooser
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS


	chooser.SetDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
	frc::SmartDashboard::PutData("Auto Modes", &chooser);
	frc::CameraServer::GetInstance()->StartAutomaticCapture();
	driveTrain->calibrateGyro();
}

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */
void Robot::DisabledInit(){
	driveTrain->setLimeLED(false);
}

void Robot::DisabledPeriodic() {
	
	frc::Scheduler::GetInstance()->Run();
	frc::SmartDashboard::PutNumber("joystick x", oi->getDriverGamepad()->GetX());
	frc::SmartDashboard::PutNumber("joystick y", oi->getDriverGamepad()->GetY());
	
}

void Robot::AutonomousInit() {
	autonomousCommand = chooser.GetSelected();
	if (autonomousCommand != nullptr)
		autonomousCommand->Start();
}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr)
		autonomousCommand->Cancel();
}

void Robot::TeleopPeriodic() {
	
	frc::Scheduler::GetInstance()->Run();
	if (oi->getOperatorGamepad()->GetPOV() == 180) { //Reverse intake
		intake->SetHopperPower(-0.5);
		intake->SetPusherPower(-0.8);
		intake->SetIndexerPower(0.2);
		intake->SetRollerPower(-0.8);
	}

	if (oi->getOperatorGamepad()->GetRawButton(8)) { // force shoot
		shooter->SetHoodHighGoal();
		if (shooter->SetShooterVelocity(4000, 50)){
			intake->SetIndexerPower(-0.4);
		}
	}

	if (oi->getOperatorGamepad()->GetRawButton(5)) { // Set ballCount = 0
		intake->SetBallCount(0);
	}
}

#ifndef RUNNING_FRC_TESTS
int main(int argc, char** argv) {
    return frc::StartRobot<Robot>();
}
#endif
