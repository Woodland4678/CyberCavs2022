// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/ShooterPIDTester.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ShooterPIDTester::ShooterPIDTester(): frc::Command() {
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
auto shootingTimerTest = 0_s;
bool switchPIDTest = false;
int setPIDSlotTest = 0;
bool isShootingCargoTest = false;
// Called just before this Command runs the first time
void ShooterPIDTester::Initialize() {
    setPIDSlotTest = 0;
    Robot::driveTrain->ShiftDown();
    //Robot::shooter->SetHoodFarShot();
    frc::SmartDashboard::PutNumber("Shooter P", kP);
    frc::SmartDashboard::PutNumber("Shooter I", kI);
    frc::SmartDashboard::PutNumber("Shooter D", kD);
    frc::SmartDashboard::PutNumber("Shooter FF", kF);
    frc::SmartDashboard::PutNumber("Shooter Set RPM", setSpeed);
    
    //Robot::intake->SetRollerPower(0.8);
    //Robot::intake->SetPusherPower(0.7);
    Robot::intake->SetHopperPower(0.6);
    Robot::intake->SetIsShooting(true);
    shootingTimerTest = frc::Timer::GetFPGATimestamp();
    switchPIDTest = false;
    

}

// Called repeatedly when this Command is scheduled to run

int isAimedCountTest = 0;

void ShooterPIDTester::Execute() {
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
    if (Robot::oi->getDriverGamepad()->GetPOV() == 0) {
        Robot::shooter->SetPID(0.00017, 0.000001, 0.003, 0.000002, true);
        Robot::shooter->SetShooterVelocity(4000, 100, 0);

    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 180) {
        Robot::shooter->StopShooterMotor();
        Robot::intake->SetHopperPower(0);
        Robot::intake->SetIndexerPower(0);
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 90) {
        Robot::shooter->SetPID(kP, kI/100, kD, kF, false);
    }
    else if (Robot::oi->getDriverGamepad()->GetPOV() == 270) {
        Robot::intake->SetIndexerPower(-0.4);
        Robot::intake->SetHopperPower(0.5);
    }
    
}

// Make this return true when this Command no longer needs to run execute()
bool ShooterPIDTester::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void ShooterPIDTester::End() {
    Robot::intake->SetHopperPower(0);
    Robot::intake->SetIndexerPower(0);
    Robot::intake->SetPusherPower(0);
    Robot::shooter->StopShooterMotor();
    Robot::intake->SetIsShooting(false);
    Robot::driveTrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShooterPIDTester::Interrupted() {
    End();
}