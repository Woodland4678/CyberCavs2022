// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Shooter.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
rev::CANSparkMax leaderMotor{11, rev::CANSparkMax::MotorType::kBrushless};


rev::CANSparkMax followMotor{10, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxPIDController shooterPidController = leaderMotor.GetPIDController();
rev::SparkMaxRelativeEncoder shooterEncoder = leaderMotor.GetEncoder();

Shooter::Shooter() : frc::Subsystem("Shooter") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
hoodSolenoid.reset(new frc::Solenoid(0, frc::PneumaticsModuleType::CTREPCM, 0));
//AddChild("hoodSolenoid", hoodSolenoid);
shooterPositionServo.reset(new frc::Servo(1));
AddChild("ShooterPositionServo", shooterPositionServo);



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    followMotor.Follow(leaderMotor);
    
    shooterPidController.SetP(0.00017, 0);
    shooterPidController.SetI(0.000001, 0);
    shooterPidController.SetD(0.003, 0);
    shooterPidController.SetFF(0.000002, 0);

    shooterPidController.SetP(0.0001, 1);
    shooterPidController.SetI(0.0, 1);
    shooterPidController.SetD(0.0, 1);
    shooterPidController.SetFF(0.0, 1);

    

    shooterPidController.SetOutputRange(-1, 1);
}

void Shooter::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Shooter::Periodic() {
    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("Shooter Actual RPM", shooterEncoder.GetVelocity());
    
}

int shooterSpeeedUpCount = 0;
bool Shooter::SetShooterVelocity(double velocity, double shooterError=150, int slot) {
    shooterPidController.SetReference(velocity, rev::ControlType::kVelocity, slot);
    if (abs(shooterEncoder.GetVelocity()-velocity)<=shooterError){
        shooterSpeeedUpCount++;
        if (shooterSpeeedUpCount>=8){
             return true;
        }
    }
    else {
        shooterSpeeedUpCount = 0;
    }

    return false;
    //leaderMotor.Set(-1);
}
void Shooter::SetPID(double p, double i, double d, double f) {
    shooterPidController.SetP(p);
    shooterPidController.SetI(i);
    shooterPidController.SetD(d);
    shooterPidController.SetFF(f);
}
void Shooter::SetHoodHighGoal() {
    hoodSolenoid->Set(true);
}
void Shooter::SetHoodLowGoal() {
    hoodSolenoid->Set(false);
}
void Shooter::StopShooterMotor() {
    leaderMotor.StopMotor();
}

void Shooter::SetServoPosition(float position) {
    shooterPositionServo->Set(position);
}
int hoodMoveToMediumState = 0;
int hoodMoveToCloseState = 0;
auto hoodMoveOriginalTime = 0_s;

double ServoCloseAndFarShotPosition = 0.8;
double ServoMediumShotPosition = 0.00;
void Shooter::SetHoodFarShot() {
    hoodSolenoid->Set(true);
    SetServoPosition(ServoCloseAndFarShotPosition);
    currentHoodPosition = HOODFARSHOTPOSITION;
    hoodMoveToMediumState = 2; // if we're already in the far shot position we can skip the first 2 states of the move to medium shot routing
}
void Shooter::SetHoodCloseShot() {
    if (currentHoodPosition == HOODMEDIUMSHOTPOSITION) {
        switch(hoodMoveToCloseState) {
            case 0:
                hoodSolenoid->Set(true);
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
                hoodMoveToCloseState++;
            break;
            case 1:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.2_s) {
                    hoodMoveToCloseState++;
                }
            break;
            case 2:
                SetServoPosition(ServoCloseAndFarShotPosition); //position for holding medium shot
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
                hoodMoveToCloseState++;
            break;
            case 3:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.35_s) {
                    hoodMoveToCloseState++;
                }
            break;
            case 4:
                hoodSolenoid->Set(false);
                hoodMoveToCloseState = 0;
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
                hoodMoveToCloseState++;
            break;
            case 5:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.2_s) {
                    currentHoodPosition = HOODCLOSESHOTPOSITION;
                    shooterPositionServo->SetDisabled();
                    hoodMoveToMediumState = 0;
                    hoodMoveToCloseState = 0;
                }
            break;
        }
    }
    else {
        SetServoPosition(ServoCloseAndFarShotPosition);
        hoodSolenoid->Set(false);
        hoodMoveToMediumState = 0; //reset this if we call medium shot again
        currentHoodPosition = HOODCLOSESHOTPOSITION;
    }
}
void Shooter::SetHoodMediumShot() {
    if (currentHoodPosition != HOODMEDIUMSHOTPOSITION) {
        switch(hoodMoveToMediumState) {
            case 0:
                hoodSolenoid->Set(true);
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
                hoodMoveToMediumState++;
            break;
            case 1:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.2_s) {
                    hoodMoveToMediumState++;
                    
                }
            break;
            case 2:
                SetServoPosition(ServoMediumShotPosition); //position for holding medium shot
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
                hoodMoveToMediumState++;
            break;
            case 3:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.35_s) {
                    hoodMoveToMediumState++;
                    shooterPositionServo->SetDisabled();
                    
                }
            break;
            case 4:
                hoodSolenoid->Set(false);
                hoodMoveToMediumState++;
                hoodMoveOriginalTime = frc::Timer::GetFPGATimestamp();
            break;
            case 5:
                if (frc::Timer::GetFPGATimestamp() - hoodMoveOriginalTime > 0.15_s) {
                    currentHoodPosition = HOODMEDIUMSHOTPOSITION;
                    shooterPositionServo->SetDisabled();
                }
            break;
        }
    }
}
int Shooter::GetCurrentHoodPosition() {
    return currentHoodPosition;
}
double Shooter::GetCurrentRPM() {
    return shooterEncoder.GetVelocity();
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

