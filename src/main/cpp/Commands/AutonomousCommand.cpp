// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/AutonomousCommand.h"
#include "Commands/Autonomous/Auto5Ball.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousCommand::AutonomousCommand(): frc::Command() {
  Requires(Robot::driveTrain.get());
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutonomousCommand::Initialize() {
    autonomousMode = new Auto5Ball();
    //path1 = new PathFinder(3,2,0.4,1);
    //path1->setStartPoint(0, 0, 0); 
    //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    //path1->splineTo(1,3, 1, 0,2.2,2.0,0,5000); //2.44, 0, 0 - meters
    //autoStep = 0;
    Robot::driveTrain->resetGyro();
    autonomousMode->Start();
    
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() {
   
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void AutonomousCommand::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousCommand::Interrupted() {

}
