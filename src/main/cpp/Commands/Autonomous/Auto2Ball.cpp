/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/Auto2Ball.h"
#include "Subsystems/LidarViewer.h"
#include <frc/Timer.h>

Auto2Ball::Auto2Ball(): frc::Command() {
  Requires(Robot::driveTrain.get());
  Requires(Robot::shooter.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Auto2Ball::Initialize() {
    // path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(-1.080, 2.275, 0,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    // path1->addWayPoint(-1.539, 2.275, 0,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(-2.286, 3.152, -90,0.007); //2.44, 0, 0 - meters
    // path1->makePath();
    
    // path2 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path2->createNewPath();
    // path2->addWayPoint(-2.286, 3.152, -90,0.007); 
    // path2->addWayPoint(-3.048, 1.524, -120,0.007); 
    // path2->addWayPoint(-3.810, 0.762, -180,0.007); 
    // path2->addWayPoint(-4.571, 1.524, -270,0.007); 
    // path2->addWayPoint(-4.575, 3.258, -270,0.007); 
    // path2->makePath();

    // path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path3->createNewPath();
    // path3->addWayPoint(-4.575, 3.258, -270,0.007); 
    // path3->addWayPoint(-4.571, 1.524, -270,0.007); 
    // path3->addWayPoint(-5.336, 0.762, -360,0.007); 
    // path3->addWayPoint(-6.096, 0.762, -360,0.007); 
    // path3->addWayPoint(-6.858, 1.524, -450,0.007); 
    // path3->addWayPoint(-6.858, 3.186, -450,0.007); 
    // path3->makePath();

    // path4 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path4->createNewPath();
    // path4->addWayPoint(-6.858, 3.186, -450,0.007); 
    // path4->addWayPoint(-7.277, 2.261, -480,0.007); 
    // path4->makePath();
    path1 = new PathFinder(2,2,0.4,1);
    path1->setStartPoint(0,0, 0); 
    //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(1,-3, 0, 0, -2.2,-0.5,0,5000); //2.44, 0, 0 - meters

    path2 = new PathFinder(2.5,2.0,0.37,1);
    path2->setStartPoint(-3, 0, 0); 
    path2->splineTo(1,-1,0, 0,2.5,2.0,0,5000); //-3.248, 1.524
    //path2->splineTo(2,-0.58, 2.013, 106.09,-2.0,-2.0,0,5000); //-3.810, 0.762, -180
    //path2->splineTo(3,-4.471, 1.904, -269.9,-2.0,-2.0,0,5000); //-4.571, 1.524, -269.9
    //path2->splineTo(4,-4.475, 4.858, -270,-3,-0.5,0,5000); //-4.575, 3.258, -270

    
    LidarViewer::Get()->m_numScoring = 0;
    Robot::driveTrain->resetGyro();
    cnt = 0;
    Robot::shooter->SetHoodFarShot();
    autoStep = 0;

}

// Called repeatedly when this Command is scheduled to run
auto auto2BallOriginalTime = frc::Timer::GetFPGATimestamp();
int autoAimCnt = 0;
double twoBallCalculatedAutoShooterSpeed = 0;
double twoBallTargetVertical = 0;
void Auto2Ball::Execute() {
    switch(autoStep) {
    case BACKUPTOFIRSTBALL:
      Robot::intake->DeployIntake();
      Robot::intake->SetRollerPower(0.8);
      Robot::intake->SetPusherPower(0.8);
      Robot::intake->SetHopperPower(0.5);
      Robot::intake->SetIsDeployed(true);
      Robot::shooter->SetShooterVelocity(3800, 150);
      if(path1->processPath()) {
        autoStep = AUTOAIM;
        Robot::driveTrain->setLimeLED(true);
      }
    break;
    case DRIVEFORWARDTOSHOOT:
      if (path2->processPath()) {
        autoStep = SHOOT;
      }
    break;
    case AUTOAIM:
      twoBallTargetVertical = Robot::driveTrain->getLimeVertical();
      twoBallCalculatedAutoShooterSpeed = 4.2858 * twoBallTargetVertical * twoBallTargetVertical + 4.206434 * twoBallTargetVertical + 3539.6577;
      if (Robot::driveTrain->autoAim(0) < 0.05) {
        autoAimCnt++;
      }
      else {
        autoAimCnt = 0;
      }
      if (autoAimCnt > 10) {
        autoStep = SHOOT;
      }
    break;
    case SHOOT:
    
      if ( Robot::shooter->SetShooterVelocity(twoBallCalculatedAutoShooterSpeed, 150)) {
        Robot::intake->SetIndexerPower(-0.4);
      }
      if (Robot::intake->GetBallCount() == 0) {
        done = true;
        Robot::intake->RetractIntake();
        Robot::shooter->StopShooterMotor();
      }
    break;
    
  }
}

// Called once the command ends or is interrupted.
void Auto2Ball::End() {}

// Returns true when the command should end.
bool Auto2Ball::IsFinished() { return done; }
void Auto2Ball::Interrupted() {

}
