// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Climb.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "stdio.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Climb::Climb(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::climber.get());
    //Requires(Robot::shooter.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
bool done = false;
auto m_originalTime = 0_s;

int Climb::getDriverPOV()
{
    return Robot::oi->getDriverGamepad()->GetPOV();
}

// Called just before this Command runs the first time
void Climb::Initialize() {
    m_originalTime = frc::Timer::GetFPGATimestamp();
    //Robot::climber->OpenFile();
    Robot::climber->RaiseClimber(); // Activate the lift air cylinder.
    Robot::climber->SetClimberMode(true);
}

int count = 0;
bool doneCalibrate = false;
// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
    Robot::shooter->SetHoodCloseShot(); // Hood needs to be all the way down for climb to work.  Just keep calling this and it will go down.
    if (Robot::shooter->GetCurrentHoodPosition() == 1) {
        if (!doneCalibrate && Robot::climber->CalibrateClimber()) {
            doneCalibrate = true;
        }
        else if (doneCalibrate) {
            Robot::climber->Climb();
        }
    }
   
//     if (Robot::oi->getDriverGamepad()->GetPOV() == 0) {
//         //fputs("Hello Up\n",fpt);
//         //wpi::outs() << "up\n";
//         frc::SmartDashboard::PutNumber("Pov pressed", 0);
//     }
//     else if (Robot::oi->getDriverGamepad()->GetPOV() == 180) {
//         //fputs("Hello Down\n",fpt);
// //        wpi::outs() << "down\n";
//         frc::SmartDashboard::PutNumber("Pov pressed", 180);
//     }

}

// Make this return true when this Command no longer needs to run execute()
bool Climb::IsFinished() {
    if (Robot::oi->getDriverGamepad()->GetPOV() == 270)
        {
        //Robot::climber->CloseFile();
        return true;
        }
    else
        {
        return false;
        }
}

// Called once after isFinished returns true
void Climb::End() {
    doneCalibrate = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Climb::Interrupted() {
    End();
}
