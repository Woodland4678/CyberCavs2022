// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Intake.h"
#include "frc/motorcontrol/PWMVictorSPX.h"
#include "frc/pidwrappers/PIDMotorController.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <PicoColorSensor.h>
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
rev::CANSparkMax pusherMotor{1, rev::CANSparkMax::MotorType::kBrushless};


rev::CANSparkMax rollerMotor{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax indexerMotor{8, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder indexerEncoder = indexerMotor.GetEncoder();
rev::SparkMaxPIDController indexerPidController = indexerMotor.GetPIDController();
pico::ColorSensor colourSensor;
Intake::Intake() : frc::Subsystem("Intake") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
deploySolenoid.reset(new frc::Solenoid(0, frc::PneumaticsModuleType::CTREPCM, 1));
AddChild("deploySolenoid", deploySolenoid);
indexerPidController.SetP(0.05);
indexerPidController.SetI(0);
indexerPidController.SetD(0);

hopperMotor.reset(new frc::PIDMotorController(hopperMotorTemp));
AddChild("hopperPWM", std::static_pointer_cast<frc::PIDMotorController>(hopperMotor));
hopperMotor->SetInverted(false);

lowSensor.reset(new frc::DigitalInput(0));
AddChild("lowSensor", lowSensor);

highSensor.reset(new frc::DigitalInput(1));
AddChild("highSensor", highSensor);

indexerPidController.SetOutputRange(-0.5, 0.5);



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
}

void Intake::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

frc::Color detectedColour;
frc::Color matchedColour;
bool isIntakeDeployed = false;
bool wrongBallDetected = false;
int reverseIntakeCount = 0;
bool isShooting = false;

int ballCount=0;
int totalCargoShot = 0;
double error=0;
double positionFirstBall=-10.189; 
double positionSecondBall=-12.1;

int ballCounterState = 0;
int ballShotState = 0;

void Intake::Periodic() {
    if (ballCount < 0) {
        ballCount = 0;
    }
    switch (ballCounterState) {
        case 0:
            if (!GetLowSensor()) {
                ballCount++;
                ballCounterState++;
                if (ballCount > 2) {
                    ballCount = 2;
                }
            }
        break;
        case 1:
            if (GetLowSensor()) {
                ballCounterState = 0;
            }
        break;
    }
    switch (ballShotState) {
        case 0:
            if (!GetHighSensor()) {
                ballShotState++;
            }
        break;
        case 1:
            if (GetHighSensor()) {
                totalCargoShot++;
                ballCount--;
                ballShotState = 0;
            }
        break;
    }
    if (isIntakeDeployed) {
        if (colourSensor.GetProximity0() > 300) {
            if (GetAllianceColour() == 0) { //on Red alliance
                if (detectedColour.blue > 0.4 && detectedColour.red < 0.3) {
                    //wrongBallDetected = true;
                }
                // if (colourSensor.GetRawColor0().blue > colourSensor.GetRawColor0().red) {
                //      wrongBallDetected = true;
                //  }
            }
            else if (GetAllianceColour() == 1) { //on Blue alliance
                if (detectedColour.red > 0.4 && detectedColour.blue < 0.3) {
                    //wrongBallDetected = true;
                }
                // if (colourSensor.GetRawColor0().red > colourSensor.GetRawColor0().blue) {
                //      wrongBallDetected = true;
                // }
            }
        }
    }
    if (wrongBallDetected) {
        pusherMotor.Set(-0.8);
        rollerMotor.Set(-0.8);
        SetHopperPower(-0.8);
        reverseIntakeCount++;
        if (reverseIntakeCount > 30) {
            pusherMotor.Set(0);
            rollerMotor.Set(0);
        }
        if (reverseIntakeCount > 40) {
            pusherMotor.Set(0.8);
            rollerMotor.Set(0.8);
            SetHopperPower(0.8);
            reverseIntakeCount = 0;
            wrongBallDetected = false;
        }
    }
    frc::SmartDashboard::PutNumber("Index Stage", indexStage);
    frc::SmartDashboard::PutNumber("Indexer Position", GetIndexerMotorPosition());
    frc::SmartDashboard::PutNumber("Ball Count", ballCount);
    frc::SmartDashboard::PutNumber("Balls Shot",  totalCargoShot);
    // Put code here to be run every loop
        //hopperMotor->Set(1);
        if ((isIntakeDeployed) && (ballCount==1) && (indexStage == FINISH)){
            indexerMotor.Set(0.5);
            ballCount--; // ballCount == 0
            indexStage = WAITINGFIRSTBALL;
        }

        
        //if (ballCount < 2 && !isShooting) {
            
        if (!isShooting) { 
            Index();        
        }    
        if (ballCount == 0) {
            indexStage = WAITINGFIRSTBALL;
        }
        // if (ballCount == 2) {
        //     hopperMotor->Set(0);
        // }
        //}

        if (isShooting){
            //CheckNumberOfBallOut();
        }

        
    //detectedColour = m_ColourSensor.GetColor();
    //matchedColour = m_ColourMatcher.MatchClosestColor(detectedColour, m_Confidence);
    //m_Proximity = m_ColourSensor.GetProximity();
    frc::SmartDashboard::PutNumber("Detected Colour R", colourSensor.GetRawColor0().red);
    frc::SmartDashboard::PutNumber("Detected Colour G", detectedColour.green);
    frc::SmartDashboard::PutNumber("Detected Colour B", detectedColour.blue);
    frc::SmartDashboard::PutNumber("Color Sensor Proximity", GetColourSensorProximity());
    // Put code here to be run every loop

}
int Intake::GetAllianceColour() {
    return frc::DriverStation::GetInstance().GetAlliance();
}
void Intake::DeployIntake() {
    // if (ballCount == 1) {
    //     indexStage = WAITINGANDINDEXBOTHBALLS;
    // }
    isIntakeDeployed = true;
    deploySolenoid->Set(true);
}
void Intake::RetractIntake() {
    isIntakeDeployed = false;
    deploySolenoid->Set(false);
}
void Intake::SetPusherPower(double power) {
    pusherMotor.Set(power);
}
void Intake::SetRollerPower(double power) {
    rollerMotor.Set(power);
}
void Intake::SetIndexerPower(double power) {
    indexerMotor.Set(power);
}

void Intake::SetHopperPower(double power){
    hopperMotor->Set(power);
}

bool Intake::GetHighSensor(){ 
    return highSensor->Get();
}

bool Intake::GetLowSensor(){
    return lowSensor->Get();
    
}

double Intake::GetIndexerMotorPosition(){
    return indexerEncoder.GetPosition();
}

void Intake::ResetIndexerMotorPosition(){
    indexerEncoder.SetPosition(0);
}


void Intake::SetIndexerMotorPosition(double position){
    indexerPidController.SetReference(position,rev::ControlType::kPosition);
}

void Intake::SetIsDeployed(bool setDeployStatus){
    isIntakeDeployed = setDeployStatus;
}
bool Intake::GetIsDeployed(){
   return isIntakeDeployed;
}
int Intake::GetColourSensorProximity() {
    return colourSensor.GetProximity0();
}
void Intake::SetIsShooting(bool setShootingStatus){
    isShooting = setShootingStatus;
}
auto indexTimer = 0_s;
bool Intake::Index(){
    switch(indexStage){
        case WAITINGFIRSTBALL: // waiting
            if(GetLowSensor() == false){  // when we see the ball
                indexerMotor.Set(-1);
                indexStage=INDEXFIRSTBALL;
                // if (ballCount==0){
                //     ballCount++;
                // }
            }
            break;

        case INDEXFIRSTBALL: // index
            if (GetLowSensor() == true){  
                ResetIndexerMotorPosition();
                SetIndexerMotorPosition(positionFirstBall);  // move to position one
                indexStage=ENSUREFIRSTBALLPOSITION;
            }
            break;
        
        case ENSUREFIRSTBALLPOSITION:
            error = abs(positionFirstBall-GetIndexerMotorPosition());
            if (error<=1){
                indexStage=WAITINGANDINDEXBOTHBALLS;
            }
            break;
        

        case WAITINGANDINDEXBOTHBALLS:

            if(GetLowSensor() == false){  // when we see the ball
                ResetIndexerMotorPosition();
                SetIndexerMotorPosition(positionSecondBall);
                indexStage=INDEXCOMPLETE;
                indexTimer = frc::Timer::GetFPGATimestamp();
                //ballCount++;
            }
            break;

        case INDEXCOMPLETE: // done
            if(frc::Timer::GetFPGATimestamp() - indexTimer > 0.5_s) {
                SetHopperPower(0);
                isIntakeDeployed=false;
                indexStage = FINISH;
                //indexStage=WAITINGFIRSTBALL;
                
            }
        break;
        case FINISH:
            return true;
        break;
    }

    return false;
}

void Intake::CheckNumberOfBallOut(){
    if (ballCount < 0) {
        ballCount = 0;
    }
    if (ballCount > 0) {
     switch(CheckBallStage){
        case CHECKFIRSTBALLSTATUS:
            if(GetHighSensor()==true){ // if highsensor can't see the ball anymore, the first ball is out
                ballCount--;
                totalCargoShot++;
                CheckBallStage=CHECKREMAININGBALL;
            }
            break;
            
        case CHECKREMAININGBALL:
            if(GetHighSensor()==false){ // high sensor sees the second ball (only one ball left)
                CheckBallStage=CHECKSECONDBALLSTATUS;
            }

            if (ballCount == 0){
                indexStage=WAITINGFIRSTBALL;
                CheckBallStage=CHECKFIRSTBALLSTATUS;
            }

            break;

        
        case CHECKSECONDBALLSTATUS:
            if(GetHighSensor()==true){ // if second ball is out
                ballCount--;
                totalCargoShot++;
            }

            if (ballCount == 0){
                indexStage=WAITINGFIRSTBALL;
                CheckBallStage=CHECKFIRSTBALLSTATUS;
            }

            break;
     }
    }
}


int Intake::GetBallCount() {
    return ballCount;
}
int Intake::GetTotalCargoShot() {
    return totalCargoShot;
}
void Intake::SetBallCount(int ballCnt) {
    ballCount = ballCnt;
}
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

