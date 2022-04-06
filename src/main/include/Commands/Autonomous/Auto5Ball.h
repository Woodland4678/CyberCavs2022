/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/commands/Command.h"
#include "frc/commands/Subsystem.h"
#include "Subsystems/PathFinder/Path.h"
#include "Robot.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Auto5Ball
    : public frc::Command {
 public:
  Auto5Ball();

  void Initialize() override;

  void Execute() override;

  void End() override;

  bool IsFinished() override;
  void Interrupted() override;
  PathFinder *path1;
  PathFinder *path2;
  PathFinder *path3;
  PathFinder *path4,*path5,*path6;
  int autoStep = 0;
  double rVel = 0;
	double lVel = 0;
  int cnt = 0;
  bool done = false;

  double shooterSpeedFirstTwoBalls = 3720; //3500 waterloo day 1 //3740 windsor morning
  double shooterSpeedThirdBall = 3855; //3615 waterloo day 1
  double shooterSpeedFinalBalls = 3800;

  enum {
		GETFIRSTBALL,
    FIRSTARCTOSHOOTINGPOSITION,
		SECONDARCTOSHOOTPOSITION,
    DRIVETOFIRSTSHOOT,
    FIRSTGYROTURN,
    TURNTOTARGET,
    FIRSTAUTOAIM,
		SHOOTFIRSTTWOBALLS,
		GRABTHIRDBALL,
    SHOOTTHIRDBALL,
    MOVETOGRABFINALBALLS,
    DELAYTOGETFINABALLS,
    MOVETOSHOOTFINALBALLS,
    FINALAUTOAIM,
    SHOOTFINALBALLS
	};
};
