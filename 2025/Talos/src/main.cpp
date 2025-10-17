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
  IntakeFrontMiddle.setVelocity(intakeMotorSpeed, pct);
  IntakeFrontTop.setVelocity(intakeMotorSpeed, pct);
  IntakeBack.setVelocity(intakeMotorSpeed, pct);


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
  while (1) {

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    double leftJoystickFrontBackPosition = Controller.Axis3.position();
    double leftJoystickLeftRightPosition = Controller.Axis4.position();
    
    robotDrive(leftJoystickFrontBackPosition, leftJoystickLeftRightPosition);

    /*intakeState = NEUTRAL;
    if (Controller.ButtonL1.pressing()) {
      intakeState = INTAKE;
    } else if (Controller.ButtonR1.pressing()) {
      intakeState = OUTTAKE_TO_TOP;
    } else if (Controller.ButtonR2.pressing()) {
      intakeState = OUTTAKE_TO_BOTTOM;
    } else {
      intakeState = NEUTRAL;
    }

    intakeMechanism(intakeState);*/

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
