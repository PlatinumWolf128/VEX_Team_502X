/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2886932                                                   */
/*    Created:      9/26/2025, 1:42:17 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "hardware.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  // Setting the velocity of the intake motors.
  IntakeFrontBottom.setVelocity(intakeMotorSpeed, pct);
  IntakeFrontTop.setVelocity(intakeMotorSpeed, pct);
  IntakeBackBottom.setVelocity(intakeMotorSpeed, pct);
  IntakeBackTop.setVelocity(intakeMotorSpeed, pct);

  // Turning on the LED light in the optical sensor.
  OpticalSensor.setLight(ledState::on);
  OpticalSensor.setLightPower(100);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  Brain.Screen.setCursor(0, 0);
  Brain.Screen.print("Hello World!");

  Extender.set(true);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  
  // Records whether or not the pneumatics piston at the bottom of the intake
  // system is extended.
  static bool lowerRampExtended = false;
  static bool extenderExtended = true;
  Extender.set(true);
  OpticalSensor.setLight(ledState::on);
  OpticalSensor.setLightPower(100);
  IntakeState intakeState = NEUTRAL;
  int detectedColor = 0;

  while (1) {

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // (Added by the VEX gods themselves)

    double leftJoystickFrontBackPosition = Controller.Axis3.position();
    double rightJoystickLeftRightPosition = Controller.Axis1.position();
    
    // This value below represents the sensitivity when turning the drivetrain.
    // Increase it to increase the sensitivity. Increasing it past 1 will not
    // change the max turning speed. Making it negative will invert the turning
    // direction.
    double turningSensitivity = 0.5;

    robotDrive(leftJoystickFrontBackPosition, rightJoystickLeftRightPosition * turningSensitivity);

    // If we are intaking, the color-sorter code goes into action.
    if (Controller.ButtonL1.pressing()) {
      
      IntakeFrontBottom.spin(fwd);
      
      // Based on what color block was detected and whether or not that block
      // belongs to our alliance, we decide what to do.
      detectedColor = colorDetector();
      switch (detectedColor) {
        case RED_DETECTED:
          if (weAreTheRedAlliance) {
            // The block is from our alliance and is going to the hopper.
            intakeState = INTAKE;
          } else {
            // The block is from the opposing alliance and is being rejected.
            intakeState = OUTTAKE_TO_MIDDLE;
          }
          break;
        case BLUE_DETECTED:
          if (weAreTheRedAlliance) {
            // The block is from the opposing alliance and is being rejected.
            intakeState = OUTTAKE_TO_MIDDLE;
          } else {
            // The block is from our alliance and is going to the hopper.
            intakeState = INTAKE;
          }
          break;
        case NOTHING_DETECTED:
          // https://pbs.twimg.com/media/GuK0lO7XoAEGtzf.jpg
          break;
      }

    } else if (Controller.ButtonL2.pressing()) {
      intakeState = OUTTAKE_TO_BOTTOM;
    } else if (Controller.ButtonR1.pressing()) {
      intakeState = OUTTAKE_TO_TOP;
      // When scoring in the long goal, retract the extender to let the bot get
      // closer to the goal.
      if (extenderExtended == true) {
        Extender.set(false);
        extenderExtended = false;
      }
    } else if (Controller.ButtonR2.pressing()) {
      intakeState = OUTTAKE_TO_MIDDLE;
    } else {
      intakeState = NEUTRAL;
      IntakeFrontBottom.stop(brake);
    }
    intakeMechanism(intakeState);
    
    // The code to control the lower ramp in the intake with pneumatics.
    if (Controller.ButtonUp.pressing() && lowerRampExtended == false) {
      BottomRampPneumatics.set(true);
      lowerRampExtended = true;
    } else if (Controller.ButtonDown.pressing() && lowerRampExtended == true) {
      BottomRampPneumatics.set(false);
      lowerRampExtended = false;
    } 
    
    // The code to control the extender mechanism with pneumatics.
    if (Controller.ButtonLeft.pressing() && extenderExtended == true) {
      Extender.set(false);
      extenderExtended = false;
    } else if (Controller.ButtonRight.pressing() && extenderExtended == false) {
      Extender.set(true);
      extenderExtended = true;
    }
    
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print(detectedColor);

    // Sleep the task for a short amount of time to prevent wasted resources.
    // (Added by the VEX gods themselves)
    wait(20, msec); 

  }
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
