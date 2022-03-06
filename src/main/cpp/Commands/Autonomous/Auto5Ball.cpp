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
    path1 = new PathFinder(3,2,0.4,0);
    path1->setStartPoint(-2.275,-0.674, -1.5); 
    //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(1,-3.141,-0.674, 0, -2.2,-2.0,0,5000); //2.44, 0, 0 - meters

    path2 = new PathFinder(2.5,2.0,0.37,0);
    path2->setStartPoint(-3.141,-0.674, -1.5); 
    path2->splineTo(1,-2.253,-0.441, 10,2.5,2.0,0,5000); //-3.248, 1.524
    //path2->splineTo(2,-0.58, 2.013, 106.09,-2.0,-2.0,0,5000); //-3.810, 0.762, -180
    //path2->splineTo(3,-4.471, 1.904, -269.9,-2.0,-2.0,0,5000); //-4.571, 1.524, -269.9
    //path2->splineTo(4,-4.475, 4.858, -270,-3,-0.5,0,5000); //-4.575, 3.258, -270

    path3 = new PathFinder(3,2,0.37,0);
    path3->setStartPoint(-2.253,-0.441, 8.5); 
    path3->splineTo(1,-3.199,-1.452, 73.55,3,2.0,0,5000); 
    path3->splineTo(2,-2.333,-2.688, 107.16,3,2.0,0,5000); 
    //path3->splineTo(2,-5.336, 2.162, -359.9,2.0,2.0,0,5000); 
    //path3->splineTo(3,-5.996, 2.162, -360,2.0,2.0,0,5000);
    //path3->splineTo(4,-6.358, 2.524, -449.9,2.0,2.0,0,5000);
    //path3->splineTo(5,-6.358, 5.986, -450,3,0.75,0,5000);

    path4 = new PathFinder(2,2,0.37,0);
    path4->setStartPoint(-2.333,-2.688, 107.16); 
    path4->splineTo(1,-2.134,-1.804, 41.76,3.5,2.0,0,5000); 

    path5 = new PathFinder(2,2,0.37,0);
    path5->setStartPoint(-2.134,-1.804, 41.76); 
    path5->splineTo(1,-2.408,-6.698,46.25,3.5,2.0,0,5000); 

    path6 = new PathFinder(2,2,0.37,0);
    path6->setStartPoint(-2.408,-6.698,46.25); 
    path6->splineTo(1,-2.134,-1.804, 41.76,3.5,2.0,0,5000); 

    LidarViewer::Get()->m_numScoring = 0;
    Robot::driveTrain->resetGyro();
    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
auto autoOriginalTime = frc::Timer::GetFPGATimestamp();
void Auto5Ball::Execute() {
    switch(autoStep) {
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
      if (Robot::magazine->GetBallCount() == 0 || ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 1.5_s)) {
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
        autoStep = SHOOTTHIRDBALL;
        autoOriginalTime = frc::Timer::GetFPGATimestamp();
      }
    break;
    case SHOOTTHIRDBALL:
      Robot::magazine->SetIsShooting(true);
      if (Robot::shooter->SetShooterVelocity(shooterSpeedThirdBall, 150)) {
        Robot::magazine->SetIndexerPower(1);
      }
      if (Robot::magazine->GetBallCount() == 0  || ((frc::Timer::GetFPGATimestamp() - autoOriginalTime) > 1.5_s)) {
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
  }
}

// Called once the command ends or is interrupted.
void Auto5Ball::End() {}

// Returns true when the command should end.
bool Auto5Ball::IsFinished() { return done; }
void Auto5Ball::Interrupted() {

}
