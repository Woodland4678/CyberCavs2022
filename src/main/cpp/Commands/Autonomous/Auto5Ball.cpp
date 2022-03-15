/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/Auto5Ball.h"
#include "Subsystems/LidarViewer.h"
#include <frc/Timer.h>

Auto5Ball::Auto5Ball(): frc::Command() {
  Requires(Robot::driveTrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Auto5Ball::Initialize() {
  Robot::driveTrain->ShiftUp();
  path1 = new PathFinder(2,1.5,0.71,1);
  path1->setStartPoint(-2.275,-0.674, -1.5); 
  //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
  path1->splineTo(1,-3.441,-0.674, 0, -3,-1,0,5000); //2.44, 0, 0 - meters
  

  path2 = new PathFinder(1,1,0.71,1);
  path2->setStartPoint(-3.441,-0.674, 0); 
  path2->splineTo(1,-1.53,-0.352, 46.77,3,0.3,0,5000); 

  path3 = new PathFinder(1,1,0.71,1);
  path3->setStartPoint(-1.53,-0.352, 46.77); 
  path3->splineTo(1,-2.081,-2.577, 63.01,-3,-0.5,0,5000);

  path4 = new PathFinder(1,1,0.71,1);
  path4->setStartPoint(-2.081,-2.577, 62.01); 
  path4->splineTo(1,-2.004,-2.875, 52.01,-2,-1,0,5000);

  path5 = new PathFinder(1,1,0.71,1);
  path5->setStartPoint(-2.004,-2.875, 42.01); 
  path5->splineTo(1,-2.408,-6.898,42.25,-2,-1,0,5000); 

  path6 = new PathFinder(1,1,0.71,1);
  path6->setStartPoint(-2.408,-6.898,46.25); 
  path6->splineTo(1,-2.134,-1.804, 41.76,2,1,0,5000);

    /*path1 = new PathFinder(3,2,0.4,1);
    path1->setStartPoint(-2.275,-0.674, -1.5); 
    //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(1,-3.141,-0.674, 0, -1.0,-1.0,0,5000); //2.44, 0, 0 - meters

    path2 = new PathFinder(2.5,2.0,0.37,1);
    path2->setStartPoint(-3.141,-0.674, -1.5); 
    path2->splineTo(1,-2.253,-0.441, 10,1.5,1.0,0,5000); //-3.248, 1.524
    

    path3 = new PathFinder(3,2,0.37,1);
    path3->setStartPoint(-2.253,-0.441, 8.5); 
    path3->splineTo(1,-3.199,-1.452, 73.55,-1.5,-1.0,0,5000); 
    path3->splineTo(2,-2.333,-2.688, 107.16,-1.5,-1.0,0,5000); 

    path4 = new PathFinder(2,2,0.37,1);
    path4->setStartPoint(-2.333,-2.688, 107.16); 
    path4->splineTo(1,-2.134,-1.804, 41.76,1.5,1.0,0,5000); 

    path5 = new PathFinder(2,2,0.37,1);
    path5->setStartPoint(-2.134,-1.804, 41.76); 
    path5->splineTo(1,-2.408,-6.698,46.25,-3.5,-2.0,0,5000); 

    path6 = new PathFinder(2,2,0.37,1);
    path6->setStartPoint(-2.408,-6.698,46.25); 
    path6->splineTo(1,-2.134,-1.804, 41.76,3.5,2.0,0,5000); */

    LidarViewer::Get()->m_numScoring = 0;
    Robot::driveTrain->resetGyro();
    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
auto autoOriginalTime = frc::Timer::GetFPGATimestamp();
void Auto5Ball::Execute() {
    switch(autoStep) {
      case GETFIRSTBALL:
       // Robot::intake->DeployIntake();
        Robot::intake->SetIsDeployed(true);
        //Robot::shooter->SetShooterVelocity(shooterSpeedFirstTwoBalls, 150);
        if(path1->processPath()) {
          autoStep++;
        }
      break;
      case FIRSTARCTOSHOOTINGPOSITION:
        if(path2->processPath()) {
          autoStep++;
        }
      break;
      case SECONDARCTOSHOOTPOSITION:
        if(path3->processPath()) {
          autoStep++;
          autoOriginalTime = frc::Timer::GetFPGATimestamp();
        }
      break;
      case SHOOTFIRSTTWOBALLS:
        Robot::intake->SetIsShooting(true);
        if (Robot::shooter->SetShooterVelocity(shooterSpeedFirstTwoBalls, 150)) {
          Robot::intake->SetIndexerPower(0.5);
        }
        if (Robot::intake->GetBallCount() == 0 && ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 2_s)) {
          Robot::intake->SetIsShooting(false);
          autoStep = GRABTHIRDBALL;
        }

      break;
      case GRABTHIRDBALL:
        if(path4->processPath()) {
          autoStep++;
        }
      break;
      case SHOOTTHIRDBALL:
        Robot::intake->SetIsShooting(true);
        if (Robot::shooter->SetShooterVelocity(shooterSpeedThirdBall, 150)) {
          Robot::intake->SetIndexerPower(1);
        }
        if (Robot::intake->GetBallCount() == 0  || ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 2_s)) {
          autoStep = MOVETOGRABFINALBALLS;
        }
      break;
      case MOVETOGRABFINALBALLS:
        if(path5->processPath()) {
          autoStep = DELAYTOGETFINABALLS;
          autoOriginalTime = frc::Timer::GetFPGATimestamp();
        }
      
      break;
      case DELAYTOGETFINABALLS:
        if (frc::Timer::GetFPGATimestamp() - autoOriginalTime > 2_s) {
          autoStep= MOVETOSHOOTFINALBALLS;
        }
      break;
      case MOVETOSHOOTFINALBALLS:
        Robot::shooter->SetShooterVelocity(shooterSpeedFinalBalls, 150);
        if (path6->processPath()) {
          autoStep = SHOOTFINALBALLS;
        }
      break;
      case SHOOTFINALBALLS:
        Robot::intake->SetIsShooting(true);
        if (Robot::shooter->SetShooterVelocity(shooterSpeedFinalBalls, 150)) {
          Robot::intake->SetIndexerPower(1); //shoot
        }
        if (Robot::intake->GetBallCount() == 0) {
          done = true;
          Robot::driveTrain->SetLeftPower(0);
          Robot::driveTrain->SetRightPower(0);
        }
      break;
    }
    /*switch(autoStep) {
    case GETFIRSTBALL:
      Robot::intake->DeployIntake();
      Robot::magazine->SetIsDeployed(true);
      Robot::shooter->SetShooterVelocity(shooterSpeedFirstTwoBalls, 150);
      if(path1->processPath()) {
        autoStep = MOVETOFIRSTSHOOTPOSITION;
      }
    break;
    case MOVETOFIRSTSHOOTPOSITION:
      if(path2->processPath()) {
        autoStep = SHOOTFIRSTTWOBALLS;
        autoOriginalTime = frc::Timer::GetFPGATimestamp();
      }
    break;
    case SHOOTFIRSTTWOBALLS:
      Robot::magazine->SetIsShooting(true);
      if (Robot::shooter->SetShooterVelocity(shooterSpeedFirstTwoBalls, 150)) {
        Robot::magazine->SetIndexerPower(1);
      }
      if (Robot::magazine->GetBallCount() == 0 && ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 2_s)) {
        Robot::magazine->SetIsShooting(false);
        autoStep = GRABTHIRDBALL;
      }
    break;
    case GRABTHIRDBALL:
      if(path3->processPath()) {
        autoStep = MOVETOSHOOTTHIRDBALL;
      }
    break;
    case MOVETOSHOOTTHIRDBALL:
      Robot::shooter->SetShooterVelocity(shooterSpeedThirdBall, 150);
      if(path4->processPath()) {
        //autoStep = SHOOTTHIRDBALL;
        done = true;
        autoOriginalTime = frc::Timer::GetFPGATimestamp();
      }
    break;
    case SHOOTTHIRDBALL:
      Robot::magazine->SetIsShooting(true);
      if (Robot::shooter->SetShooterVelocity(shooterSpeedThirdBall, 150)) {
        Robot::magazine->SetIndexerPower(1);
      }
      if (Robot::magazine->GetBallCount() == 0  || ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 2_s)) {
        autoStep = MOVETOGRABFINALBALLS;
      }
    break;
    case MOVETOGRABFINALBALLS:
      if(path5->processPath()) {
        autoStep = DELAYTOGETFINABALLS;
        autoOriginalTime = frc::Timer::GetFPGATimestamp();
      }
     
    break;
    case DELAYTOGETFINABALLS:
      if (frc::Timer::GetFPGATimestamp() - autoOriginalTime > 1_s) {
        autoStep= MOVETOSHOOTFINALBALLS;
      }
    break;
    case MOVETOSHOOTFINALBALLS:
      Robot::shooter->SetShooterVelocity(shooterSpeedFinalBalls, 150);
      if (path6->processPath()) {
        autoStep = SHOOTFINALBALLS;
      }
    break;
    case SHOOTFINALBALLS:
      Robot::magazine->SetIsShooting(true);
      if (Robot::shooter->SetShooterVelocity(shooterSpeedFinalBalls, 150)) {
        Robot::magazine->SetIndexerPower(1); //shoot
      }
      if (Robot::magazine->GetBallCount() == 0) {
        done = true;
        Robot::driveTrain->SetLeftPower(0);
        Robot::driveTrain->SetRightPower(0);
      }
    break;
  }*/
}

// Called once the command ends or is interrupted.
void Auto5Ball::End() {}

// Returns true when the command should end.
bool Auto5Ball::IsFinished() { return done; }
void Auto5Ball::Interrupted() {

}
