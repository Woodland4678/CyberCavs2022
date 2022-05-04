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
   // Requires(Robot::intake.get());
    //Requires(Robot::shooter.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

double lowerShooterRPMSet = 0;
double higherShooterRPMSet = 0;
//double lowerShooterRPMDashboard = 0;
//double higherShooterRPMDashboard = 0;
// Called just before this Command runs the first time
void Drive::Initialize() {
    //Robot::driveTrain->setLimeLED(true);
    //frc::SmartDashboard::PutNumber("Higher Shooter RPM", higherShooterRPMDashboard);
    //frc::SmartDashboard::PutNumber("Lower Shooter RPM", lowerShooterRPMDashboard);
}

// Called repeatedly when this Command is scheduled to run
double jx = 0;
double jy = 0;
double jx2 = 0;
double jy2 = 0;
double rrpm = 0;
double lrpm = 0;
double setServoPos = 0;
double tempShooterRPM = 2000;
bool povButtonReleased = true;
bool startedShooter = false;
void Drive::Execute() {
    /*Robot::intake->SetIsShooting(true);
    if (Robot::oi->getDriverGamepad()->GetRawButton(4)) {
        Robot::intake->SetHopperPower(0.8);
        Robot::intake->SetIndexerPower(-0.4);
    }
    else {
        Robot::intake->SetHopperPower(0);
        Robot::intake->SetIndexerPower(0);
    }
    Robot::shooter->SetShooterVelocity(tempShooterRPM, 100);
    if (Robot::oi->getDriverGamepad()->GetPOV() == 0 && povButtonReleased == true) {
        tempShooterRPM += 50;
        povButtonReleased = false;
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 180 && povButtonReleased == true) {
        tempShooterRPM -= 50;
        povButtonReleased = false;
    }
     else if (Robot::oi->getDriverGamepad()->GetPOV() == -1) {
        povButtonReleased = true;
    }
    //code to test the hood positions, will remove later

    if (Robot::oi->getDriverGamepad()->GetPOV() == 90) {
        Robot::shooter->SetHoodMediumShot();
    }
/*    else if (Robot::oi->getDriverGamepad()->GetPOV() == 0 && povButtonReleased == true) {
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
    }*/
    jx2 = Robot::oi->getDriverGamepad()->GetX();
    jy2 = Robot::oi->getDriverGamepad()->GetY();
    
    if((jx2 > -0.05)&&(jx2 < 0.05))
        jx2 = 0;
    if((jy2 > -0.05)&&(jy2 < 0.05))
        jy2 = 0;

    jx = jx2 * jx2;
    jy = jy2 * jy2;

    if (jx2 < 0) {
        jx = jx * -1;
    }
    if (jy2 < 0) {
        jy = jy * -1;
    }

    //****Test code for better high gear***///
    // if (abs(jy) > 0.8 && (Robot::driveTrain->GetIsHighGear())) {
    //     int factor = 1.8 - abs(jy); //if y is big we don't want to be able to turn too fast so we multiply by a factor
    //     jx = jx * factor;
    // }
    //*************************************///
    frc::SmartDashboard::PutNumber("Joy X",jx);
    frc::SmartDashboard::PutNumber("Joy Y",jy);

    

    //jx *= 0.7; //was here as of march 28 2022, not sure if needed, just seems to be to slow down turning

   // rrpm = (jy + jx);
   // lrpm = (jy - jx);
   rrpm = jy + 0.75 * jx;
   lrpm = jy - 0.75 * jx;

    // rrpm = multi*(rrpm) + (1-multi)*pr_rpm;
    // lrpm = multi*(lrpm) + (1-multi)*pl_rpm;


    // if (abs(pr_rpm) - abs(rrpm) > 0.1) {
    //     rrpm = rrpm + pr_rpm / 1.2;
    // }
    // if (abs(pl_rpm) - abs(lrpm) > 0.1) {
    //     lrpm = lrpm + pl_rpm / 1.2;
    // }
    
    pr_rpm = rrpm;
    pl_rpm = lrpm;

    if (Robot::climber->GetClimberMode()) { //limit drive speed when we are going for the climb
        lrpm *= 0.3;
        rrpm *= 0.3;
    }
    frc::SmartDashboard::PutNumber("Right drive power",rrpm);
    frc::SmartDashboard::PutNumber("Left drive power",lrpm);
    Robot::driveTrain->SetLeftPower(lrpm);
    Robot::driveTrain->SetRightPower(rrpm);

    if (Robot::intake->GetBallCount() > 0 && !startedShooter) {
        Robot::shooter->SetPIDToSpinup(true);
        Robot::shooter->SetShooterVelocity(3800, 100);
        Robot::driveTrain->setLimeLED(true);
        startedShooter = true;
    }
    else if (Robot::intake->GetBallCount() == 0) {
        startedShooter = false;
    }

    
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
