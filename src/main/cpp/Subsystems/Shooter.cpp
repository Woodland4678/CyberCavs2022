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




    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    followMotor.Follow(leaderMotor);
    //shooterPidController.SetOutputRange(-1, 1);
    shooterPidController.SetP(0.0003);
    shooterPidController.SetI(0.000001);
    shooterPidController.SetD(0);
    shooterPidController.SetFF(0);
    shooterPidController.SetOutputRange(-0.85, 0.85);
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
void Shooter::SetShooterVelocity(double velocity) {
    shooterPidController.SetReference(velocity, rev::ControlType::kVelocity);
    //leaderMotor.Set(-1);
}
void Shooter::SetPID(double p, double i, double d) {
    shooterPidController.SetP(p);
    shooterPidController.SetI(i);
    shooterPidController.SetD(d);
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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

