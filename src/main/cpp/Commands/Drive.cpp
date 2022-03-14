// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Drive.h"
#include <frc/SmartDashboard/SmartDashboard.h>
const double multi = 0.21;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Drive::Drive(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveTrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

double lowerShooterRPMSet = 0;
double higherShooterRPMSet = 0;
//double lowerShooterRPMDashboard = 0;
//double higherShooterRPMDashboard = 0;
// Called just before this Command runs the first time
void Drive::Initialize() {
    //frc::SmartDashboard::PutNumber("Higher Shooter RPM", higherShooterRPMDashboard);
    //frc::SmartDashboard::PutNumber("Lower Shooter RPM", lowerShooterRPMDashboard);
}

// Called repeatedly when this Command is scheduled to run
double jx = 0;
double jy = 0;
double rrpm = 0;
double lrpm = 0;
double setServoPos = 0;
bool povButtonReleased = true;
void Drive::Execute() {

    //code to test the hood positions, will remove later

    if (Robot::oi->getDriverGamepad()->GetPOV() == 90) {
        Robot::shooter->SetHoodFarShot();
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 0 && povButtonReleased == true) {
        setServoPos += 0.05;
        frc::SmartDashboard::PutNumber("servo pos",setServoPos);
        Robot::shooter->SetServoPosition(setServoPos);
        povButtonReleased = false;
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 180) {
        Robot::shooter->SetHoodMediumShot();
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 270) {
        Robot::shooter->SetHoodCloseShot();
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == -1) {
        povButtonReleased = true;
    }
    jx = Robot::oi->getDriverGamepad()->GetX();
    jy = Robot::oi->getDriverGamepad()->GetY();

    frc::SmartDashboard::PutNumber("Joy X",jx);
    frc::SmartDashboard::PutNumber("Joy Y",jy);

    if((jx > -0.05)&&(jx < 0.05))
        jx = 0;
    if((jy > -0.05)&&(jy < 0.05))
        jy = 0;

    jx *= 0.7;

    rrpm = (jy + jx);
    lrpm = (jy - jx);

    rrpm = multi*(rrpm) + (1-multi)*pr_rpm;
    lrpm = multi*(lrpm) + (1-multi)*pl_rpm;

    pr_rpm = rrpm;
    pl_rpm = lrpm;

    Robot::driveTrain->SetLeftPower(lrpm);
    Robot::driveTrain->SetRightPower(rrpm);
}

// Make this return true when this Command no longer needs to run execute()
bool Drive::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void Drive::End() {
    Robot::driveTrain->SetRightPower(0);
    Robot::driveTrain->SetLeftPower(0); 
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Drive::Interrupted() {
    End();
}
