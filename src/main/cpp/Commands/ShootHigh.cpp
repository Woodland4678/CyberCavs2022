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

auto shootingTimer = 0_s;
bool switchPID = false;
int setPIDSlot = 0;
bool isShootingCargo = false;
// Called just before this Command runs the first time
int hoodTargetPos = 0;
void ShootHigh::Initialize() {
    setPIDSlot = 0;
    Robot::driveTrain->ShiftDown();
    //Robot::shooter->SetHoodFarShot();
    
    Robot::driveTrain->setLimeLED(true);
    
    //Robot::intake->SetRollerPower(0.8);
    Robot::intake->SetPusherPower(0.7);
    Robot::intake->SetHopperPower(0.6);
    Robot::intake->SetIsShooting(true);
    shootingTimer = frc::Timer::GetFPGATimestamp();
    switchPID = false;
    

}

// Called repeatedly when this Command is scheduled to run
bool canShoot = false;
double calculatedShooterSpeed = 0;
double targetVertical = 0;
int isAimedCount = 0;
double shooterSetSpeed = 0;
float hoodHighClosestValue = -5; //value become more negative the further away we get -4.8
float hoodMediumHighestValue = -7.2; //-6.2


void ShootHigh::Execute() {
    if (Robot::driveTrain->getLimeValidObject() || canShoot) {
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
                //calculatedShooterSpeed = 4.2858 * targetVertical * targetVertical + 4.206434 * targetVertical + 3494.6577; //3429 was 3429.6577 at waterloo day 1
                calculatedShooterSpeed = Robot::shooter->CalcRPMFarShot(targetVertical);
            }
            else if (targetVertical >= hoodHighClosestValue) {
                hoodTargetPos = 2;
                Robot::shooter->SetHoodMediumShot();
                //calculatedShooterSpeed = 2.36858 * targetVertical * targetVertical + -48.24201 * targetVertical + 3472.9; //3439.9 was 3439.9 at waterloo day 1
                calculatedShooterSpeed = Robot::shooter->CalcRPMMediumShot(targetVertical);
            }
            else {
                if (Robot::shooter->GetCurrentHoodPosition() == 1 || Robot::shooter->GetCurrentHoodPosition() == 0) {
                    Robot::shooter->SetHoodFarShot();
                    hoodTargetPos = 0;
                   // calculatedShooterSpeed = 4.2858 * targetVertical * targetVertical + 4.206434 * targetVertical + 3494.6577;
                   calculatedShooterSpeed = Robot::shooter->CalcRPMFarShot(targetVertical);
                    
                }
                else {
                    hoodTargetPos = 2;
                    //calculatedShooterSpeed = 2.36858 * targetVertical * targetVertical + -48.24201 * targetVertical + 3472.9; //need equation here later
                    calculatedShooterSpeed = Robot::shooter->CalcRPMMediumShot(targetVertical);
                }
            }
            Robot::shooter->SetShooterVelocity(calculatedShooterSpeed, 100, setPIDSlot);
        }
        if (!canShoot || !isShootingCargo) {
            if((Robot::driveTrain->autoAim(-1) < 0.07)) {
                isAimedCount++;
            }
            else {
                isAimedCount = 0;
            }
        }
        
        if (isAimedCount > 25) {
            canShoot = true;
            Robot::driveTrain->SetLeftPower(0);
            Robot::driveTrain->SetRightPower(0);
        }
        if (Robot::shooter->GetCurrentHoodPosition() != hoodTargetPos) {
            if (hoodTargetPos == 2) {
                Robot::shooter->SetHoodMediumShot();
            }
            else if (hoodTargetPos == 0) {
                Robot::shooter->SetHoodFarShot();
            }
        }
        frc::SmartDashboard::PutNumber("target hood pos", hoodTargetPos);
        if (canShoot && (Robot::shooter->GetCurrentHoodPosition() == hoodTargetPos)) {
            Robot::intake->SetIsShooting(true);
            double curVel = Robot::shooter->GetCurrentRPM();
            frc::SmartDashboard::PutNumber("shooterError", abs(calculatedShooterSpeed - curVel));
            Robot::shooter->SetShooterVelocity(calculatedShooterSpeed, 45, setPIDSlot);
            if(abs(calculatedShooterSpeed - curVel)<50){ /// 
                setPIDSlot = 1;
                isShootingCargo = true;
                Robot::intake->SetIndexerPower(-0.85);
                Robot::intake->SetHopperPower(0.7);
            }
            else {
                if (calculatedShooterSpeed < 4000) {
                    Robot::intake->SetIndexerPower(-0.15);
                }
                else {
                    Robot::intake->SetIndexerPower(0);
                }
            }
            
        }
    }
    // else { // if no valid target assume we are against the hub
    //    Robot::shooter->SetHoodCloseShot();
    //     if (Robot::shooter->GetCurrentHoodPosition() == 1) { //wait for hood to be in close shot position
    //         if (Robot::shooter->SetShooterVelocity(3475, 50)<=35) {
    //             Robot::intake->SetIsShooting(true);
    //             Robot::intake->SetIndexerPower(-0.7);
    //         }
    //         else {
    //             Robot::intake->SetIndexerPower(0);
    //         }
    //     } 
    // }
    
   
    
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
    Robot::intake->SetPusherPower(0);
    Robot::shooter->StopShooterMotor();
    Robot::intake->SetIsShooting(false);
    Robot::driveTrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootHigh::Interrupted() {
    End();
}
