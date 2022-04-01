// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/AutoAim.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/Climb.h"
#include "Commands/Drive.h"
#include "Commands/HighGear.h"
#include "Commands/IntakeDeploy.h"
#include "Commands/IntakeRetract.h"
#include "Commands/LowGear.h"
#include "Commands/ShootHigh.h"
#include "Commands/ShootLow.h"
#include "Commands/ClimbReset.h"
#include "Commands/RemoveWrongBall.h"
#include "Commands/ReverseIntake.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
operatorGamepad.reset(new frc::Joystick(1));

driverGamepad.reset(new frc::Joystick(0));

autoSwitch.reset(new frc::Joystick(2));

// driver_Btn1_X.reset(new frc::JoystickButton(driverGamepad.get(), 1));
// driver_Btn1_X->WhileHeld(new Climb());
driver_Btn2_A.reset(new frc::JoystickButton(driverGamepad.get(), 2));
driver_Btn2_A->WhenPressed(new IntakeDeploy());
driver_Btn3_B.reset(new frc::JoystickButton(driverGamepad.get(), 3));
driver_Btn3_B->WhenPressed(new IntakeRetract());
driver_Btn4_Y.reset(new frc::JoystickButton(driverGamepad.get(), 4));
driver_Btn4_Y->WhileHeld(new ShootLow());
driver_Btn5_LB.reset(new frc::JoystickButton(driverGamepad.get(), 5));
driver_Btn5_LB->WhenPressed(new LowGear());
driver_Btn6_RB.reset(new frc::JoystickButton(driverGamepad.get(), 6));
driver_Btn6_RB->WhenPressed(new HighGear());
driver_Btn7_LT.reset(new frc::JoystickButton(driverGamepad.get(), 7));
driver_Btn7_LT->WhileHeld(new AutoAim());
driver_Btn8_RT.reset(new frc::JoystickButton(driverGamepad.get(), 8));
driver_Btn8_RT->WhileHeld(new ShootHigh());

operator_Btn1_X.reset(new frc::JoystickButton(operatorGamepad.get(), 1));
operator_Btn1_X->WhenPressed(new Climb());
operator_Btn3_B.reset(new frc::JoystickButton(operatorGamepad.get(), 3));
operator_Btn3_B->WhenPressed(new ClimbReset());
operator_Btn7_LT.reset(new frc::JoystickButton(operatorGamepad.get(), 7));
operator_Btn7_LT->WhileHeld(new RemoveWrongBall());
operator_Btn6_RB.reset(new frc::JoystickButton(operatorGamepad.get(), 6));
operator_Btn6_RB->WhileHeld(new ReverseIntake());




    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("ShootLow", new ShootLow());
    frc::SmartDashboard::PutData("ShootHigh", new ShootHigh());
    frc::SmartDashboard::PutData("IntakeRetract", new IntakeRetract());
    frc::SmartDashboard::PutData("IntakeDeploy", new IntakeDeploy());
    frc::SmartDashboard::PutData("LowGear", new LowGear());
    frc::SmartDashboard::PutData("HighGear", new HighGear());
    frc::SmartDashboard::PutData("Climb", new Climb());
    frc::SmartDashboard::PutData("AutoAim", new AutoAim());
    frc::SmartDashboard::PutData("Drive", new Drive());
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getDriverGamepad() {
   return driverGamepad;
}

std::shared_ptr<frc::Joystick> OI::getOperatorGamepad() {
   return operatorGamepad;
}
std::shared_ptr<frc::Joystick> OI::getAutoSwitch() {
   return autoSwitch;
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
