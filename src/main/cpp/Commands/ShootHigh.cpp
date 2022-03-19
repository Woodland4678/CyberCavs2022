// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/ShootHigh.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ShootHigh::ShootHigh(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::shooter.get());
    Requires(Robot::driveTrain.get());
    Requires(Robot::intake.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
double kP = 0.000001, kI = 0.000000, kD = 0, kF = 0.002, setSpeed = 3700; //close shot 3375 is good //med shots in high position at 3850 are good
auto shootingTimer = 0_s;
bool switchPID = false;
int setPIDSlot = 0;
// Called just before this Command runs the first time
void ShootHigh::Initialize() {
    Robot::shooter->SetHoodFarShot();
    frc::SmartDashboard::PutNumber("Shooter P", kP);
    frc::SmartDashboard::PutNumber("Shooter I", kI);
    frc::SmartDashboard::PutNumber("Shooter D", kD);
    frc::SmartDashboard::PutNumber("Shooter FF", kF);
    frc::SmartDashboard::PutNumber("Shooter Set RPM", setSpeed);
    Robot::driveTrain->setLimeLED(true);
    
    //Robot::intake->SetRollerPower(0.8);
    //Robot::intake->SetPusherPower(0.8);
    Robot::intake->SetIsShooting(true);
    switchPID = false;

}

// Called repeatedly when this Command is scheduled to run
bool canShoot = false;
double calculatedShooterSpeed = 0;
double targetVertical = 0;
int isAimedCount = 0;
double shooterSetSpeed = 0;
float hoodHighClosestValue = -4.8; //value become more negative the further away we get
float hoodMediumHighestValue = -6;
int hoodTargetPos = 0;

void ShootHigh::Execute() {
    if (frc::Timer::GetFPGATimestamp() - shootingTimer > 1.5_s) {
        Robot::intake->SetRollerPower(0);
        Robot::intake->SetPusherPower(0);
        Robot::intake->RetractIntake();
    }
    if (!canShoot || (Robot::shooter->GetCurrentHoodPosition() != hoodTargetPos)) {
        targetVertical = Robot::driveTrain->getLimeVertical();
        if (targetVertical <= hoodMediumHighestValue) {
            hoodTargetPos = 0;
            Robot::shooter->SetHoodFarShot();
            calculatedShooterSpeed = 4.2858 * targetVertical * targetVertical + 4.206434 * targetVertical + 3639.6577;
        }
        else if (targetVertical >= hoodHighClosestValue) {
            hoodTargetPos = 2;
            Robot::shooter->SetHoodMediumShot();
            calculatedShooterSpeed = 2.36858 * targetVertical * targetVertical + -48.24201 * targetVertical + 3489.9; //need equation here later
        }
        else {
            if (Robot::shooter->GetCurrentHoodPosition() == 1 || Robot::shooter->GetCurrentHoodPosition() == 0) {
                Robot::shooter->SetHoodFarShot();
                hoodTargetPos = 0;
                calculatedShooterSpeed = 4.2858 * targetVertical * targetVertical + 4.206434 * targetVertical + 3639.6577;
                
            }
            else {
                hoodTargetPos = 2;
                calculatedShooterSpeed = 2.36858 * targetVertical * targetVertical + -48.24201 * targetVertical + 3489.9; //need equation here later
            }
        }
        Robot::shooter->SetShooterVelocity(calculatedShooterSpeed, 100, setPIDSlot);
    }
    if((Robot::driveTrain->autoAim(0) < 0.1) && !canShoot) {
        isAimedCount++;
    }
    else {
        isAimedCount = 0;
    }
    if (isAimedCount > 15) {
        canShoot = true;
    }
    if (canShoot && (Robot::shooter->GetCurrentHoodPosition() == hoodTargetPos)) {
        Robot::intake->SetIsShooting(true);
        if(Robot::shooter->SetShooterVelocity(calculatedShooterSpeed, 50, setPIDSlot)){
            setPIDSlot = 1;
            Robot::intake->SetIndexerPower(-0.4);
            Robot::intake->SetHopperPower(0.7);
        }
    }
    /*if (Robot::oi->getDriverGamepad()->GetRawButton(1)) {
        Robot::intake->SetHopperPower(0.8);
        Robot::intake->SetIndexerPower(-0.4);
    }
    double p = frc::SmartDashboard::GetNumber("Shooter P", 0.1);
    double i = frc::SmartDashboard::GetNumber("Shooter I", 0.000001);
    double d = frc::SmartDashboard::GetNumber("Shooter D", 0);
    double f = frc::SmartDashboard::GetNumber("Shooter FF", 0.000002);
    // = frc::SmartDashboard::GetNumber("Shoot Set RPM", 1500);
    if (p != kP || i != kI || d != kD || f != kF) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }
    //if (setSpeed != shooterSetSpeed) {
        
      //  setSpeed = shooterSetSpeed;
    //}
    if (Robot::oi->getDriverGamepad()->GetPOV() == 0) {
        //if (switchPID == false) {
            //Robot::shooter->SetPID(0.000001, 0.000002, 0.000, 1.0);
        //}
        //else {
        //    Robot::shooter->SetPID(0.001, 0.000000, 0, 0);
        //}
        if (Robot::shooter->SetShooterVelocity(shooterSetSpeed, 100)) {
            isAimedCount++;
            Robot::shooter->SetPID(0.0001, 0.000000, 0, 0);
        }

        if (isAimedCount > 30) {
             switchPID = true;
             
        }
        
        shooterSetSpeed = 4000;
        //Robot::shooter->SetShooterVelocity(shooterSetSpeed, 100);
        //Robot::intake->SetHopperPower(0.5);
        //Robot::intake->SetIndexerPower(-0.3);
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 180) {
        Robot::shooter->StopShooterMotor();
        Robot::intake->SetHopperPower(0);
        Robot::intake->SetIndexerPower(0);
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 90) {
        Robot::shooter->SetPID(p, i, d, f);
    }*/
    
}

// Make this return true when this Command no longer needs to run execute()
bool ShootHigh::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void ShootHigh::End() {
    canShoot = false;
    isAimedCount = 0;
    setPIDSlot = 0;
    Robot::intake->SetHopperPower(0);
    Robot::intake->SetIndexerPower(0);
    Robot::shooter->StopShooterMotor();
    Robot::intake->SetIsShooting(false);
    Robot::driveTrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootHigh::Interrupted() {
    End();
}
