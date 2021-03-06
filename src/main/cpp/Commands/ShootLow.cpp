// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/ShootLow.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ShootLow::ShootLow(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::shooter.get());
    Requires(Robot::driveTrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
auto shootLowTimer = 0_s;
void ShootLow::Initialize() {
    Robot::shooter->SetHoodFarShot();
    Robot::intake->SetRollerPower(0.8);
    Robot::intake->SetPusherPower(0.8);
    shootLowTimer = frc::Timer::GetFPGATimestamp();
}
int shootLowCnt = 0;
// Called repeatedly when this Command is scheduled to run
void ShootLow::Execute() {
   // Robot::shooter->Set();
   frc::SmartDashboard::PutNumber("shoot low count", shootLowCnt);
   shootLowCnt++;
    if (frc::Timer::GetFPGATimestamp() - shootLowTimer > 0.5_s) {
        Robot::intake->SetRollerPower(0);
        Robot::intake->SetPusherPower(0);
        Robot::intake->RetractIntake();
    }
    Robot::shooter->SetHoodMediumShot();
    Robot::shooter->SetShooterVelocity(1750, 200);
    if (Robot::shooter->GetCurrentHoodPosition() == 2) {
        if(Robot::shooter->SetShooterVelocity(1750, 200)<100){
            Robot::intake->SetIsShooting(true);
            Robot::intake->SetIndexerPower(-0.7);
            Robot::intake->SetHopperPower(0.8);
        }
    }
}

// Make this return true when this Command no longer needs to run execute()
bool ShootLow::IsFinished() {

    return false;
}

// Called once after isFinished returns true
void ShootLow::End() {
    Robot::intake->SetHopperPower(0);
    Robot::intake->SetIndexerPower(0);
    Robot::shooter->StopShooterMotor();
    Robot::intake->SetIsShooting(false);
    Robot::driveTrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootLow::Interrupted() {
    End();
}
