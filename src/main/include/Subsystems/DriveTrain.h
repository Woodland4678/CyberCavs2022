// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#pragma once

#include "frc/commands/Subsystem.h"
#include "frc/Relay.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "rev/CANSparkMax.h"
#include <frc/PneumaticsModuleType.h>
#include <frc/Solenoid.h>
#include "frc/ADXRS450_Gyro.h"
#include "frc/AnalogInput.h"
#include "Subsystems/PathFinder/Path.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveTrain: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
/*rev::CANSparkMax leftLeaderMotor{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rightLeaderMotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax leftFollowMotor{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rightFollowMotor{6, rev::CANSparkMax::MotorType::kBrushless};*/
//std::shared_ptr<frc::Solenoid> shifter;
std::shared_ptr<frc::Relay> shifter;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	frc::ADXRS450_Gyro *m_Gyro;
	std::shared_ptr<frc::AnalogInput> pressureSensor;

public:
DriveTrain();
	void InitDefaultCommand() override;
	void Periodic() override;
	void ShiftUp();
	void ShiftDown();
	void SetLeftPower(double power);
	void SetRightPower(double power);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	//Set Functions
	void setLeftRPM(double rpm);
	void setRightRPM(double rpm);
	void setLeftVelocity(double mps);
	void setRightVelocity(double mps);
	void setLeftPosition(double encoder);
	void setRightPosition(double encoder);

	//Get Functions
	double getLeftEncoder();
	double getRightEncoder();
	double getLeftRPM();
	double getRightRPM();
	double getLeftVelocity();
	double getRightVelocity();

	//Limelight
	std::shared_ptr<nt::NetworkTable> limelight;
	bool ml_ValidTarget = false;
	double ml_targetHorizontial;
	double ml_targetVertical;
	double ml_TargetDistance;

	void setLimeLED(bool state);
	void SetLimeFar();
	void SetLimeZoomed();
	bool getLimeValidObject();
	double getLimeHorizontial();
	double getLimeVertical();
	double calculateLimeDist();
	bool GyroTurn(double current, double turnAmount, double p, double i, double d, double allowError);
	//Advanced Control Functions
	bool turnAmount(double degrees, int direction, double vel, double acc);
	double mt_acc;
	double mt_vel;
	double mt_tarDeg;
	double mt_tarDir;
	double mt_OEncLeft;
	double mt_OEncRight;
	int mt_state = 0;
	int mt_Cycles = 0;
	double d[400];
	int traverseCnt = 0;
	double encPrevLeft,encPrevRight,encLeft,encRight;

	//PathFinder *m_Path;
	PathFinder *A_Paths[7];
	bool testPath();
	void initPath();
	bool pTest = false;
	int pathState = 0;
	double rVel = 0;
	double lVel = 0;
	int tCnt = 0;

	//Auto Controls
	double autoAim(double target);

	double r_Set = 0;
	double l_Set = 0;
	
	double origTimeStamp;

	//Gear Shift Functions
	void shiftUp();
	void shiftDown();
	
	//Gryo
	double getGyroReading();
	void resetGyro();
	void calibrateGyro();

	//PDP
	double readPDPCurrent(int channel);
	double readTotalCurrent();

	void resetEncoders();
	void resetPosition();

	

	void setStart(double x, double y, double angle);
	double rightRef,leftRef;
	int prevEncRight,prevEncLeft;
	double thetaHeading;
	double positionX,positionY;

	typedef struct {
	double locx; // drivetrain x position (in cm)
	double locy; // drivetrain y position (in cm)
	double heading; // which way robot is pointed.  0 deg is considered to be
	// straight out from cargo ship towards center driver station.
	double rightEncoder; // previous right encoder value.
	double leftEncoder; // previous left encoder value.
	double gyroOffset; // Value to add to Gyro reading to get correct heading.
	bool inLowSpeed; // true in low gear, false in high.
	} locationtp;

	locationtp location; // Structure that holds the robot's location information.
	double pressure; // Tank pressure.

};

