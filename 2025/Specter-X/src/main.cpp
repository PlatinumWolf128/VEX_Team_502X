/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2886932                                                   */
/*    Created:      12/17/2025, 5:02:10 PM                                    */
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

  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
    //
  }
  Inertial.setHeading(0, degrees);

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
  
  Inertial.setHeading(0, degrees);
  wait(500, msec);

  drive(0, -50, 0, false);
  wait(500, msec);
  drive(0, 0, 0, false);
  /*int timeElapsed = 0;

  // Drive fwd 16 inches
  while (timeElapsed < distanceToTime(32)) {
    drive(100, 0, aligner(0), false);
    wait(20, msec);
    timeElapsed += 20;
  }

  // Stop
  timeElapsed = 0;
  drive(0, 0, 0, false);
  wait(100, msec);

  // Strafe left 27 inches
  while (timeElapsed < distanceToTime(54)) {
    drive(0, -100, aligner(0), false);
    wait(20, msec);
    timeElapsed += 20;
  }

  // Stop
  timeElapsed = 0;
  drive(0, 0, 0, false);
  wait(100, msec);

  // Rotate 180 degrees
  while (fabs(Inertial.heading()) > 185 || fabs(Inertial.heading()) < 175) {
    drive(0, 0, aligner(180), false);
    wait(20, msec);
  }

  // Stop
  timeElapsed = 0;
  drive(0, 0, 0, false);
  wait(250, msec);

  // Drive fwd 16 inches (towards the loader)
  while (timeElapsed < distanceToTime(32)) {
    drive(100, 0, aligner(180), true);
    wait(20, msec);
    timeElapsed += 20;
  }

  // Stop
  timeElapsed = 0;
  drive(0, 0, 0, false);
  wait(100, msec);
  
  // Intake for 5s
  intake(100);
  wait(5000, msec);
  intake(0);
 */
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
  double targetHeading = 0;
  bool robotOriented = false;
  AlignmentState alignment = NORTH;

  double intakeVelocity = 0;

  while (1) {

    bool prevLeftPressed = false;
    bool prevRightPressed = false;
    bool prevXPressed = false;
    
    double forwardVelocity = Controller.Axis3.position();
    double strafeVelocity = Controller.Axis4.position();
    double turnVelocity = Controller.Axis1.position();

    double currentHeading = Inertial.heading(degrees);

    if (Controller.ButtonUp.pressing()) {
      // ALign to "North" (the position you calibrated to)
      alignment = NORTH;
    } else if (Controller.ButtonDown.pressing()) {
      // Align to "South" (the opposite of North)
      alignment = SOUTH;
    } else if (Controller.ButtonRight.pressing() && !prevRightPressed) {
      // Rotate the target heading 45 degrees clockwise.
      targetHeading += 45;
      alignment = CUSTOM;
      prevRightPressed = true;
    } else if (Controller.ButtonLeft.pressing() && !prevLeftPressed) {
      // Rotate the target heading 45 degrees counterclockwise.
      targetHeading -= 45;
      alignment = CUSTOM;
      prevLeftPressed = true;
    } else if (Controller.ButtonY.pressing()) {
      // Set the current heading as the target to align to.
      alignment = MAINTAIN_CURRENT;
      targetHeading = currentHeading;
    } else if (Controller.ButtonX.pressing() && !prevXPressed) {
      // Toggle whether or not the drivetrain is robot-oriented.
      robotOriented = !robotOriented;
      prevXPressed = true;
      wait(1000, msec);
    } else if (fabs(turnVelocity) >= DEADZONE) {
      // Quit alignment mode.
      alignment = NEUTRAL;
    }

    switch (alignment) {
      case NORTH:
        targetHeading = 0;
        turnVelocity = aligner(targetHeading);
        break;
      case SOUTH:
        targetHeading = 180;
        robotOriented = false;
        turnVelocity = aligner(targetHeading);
        break;
      case CUSTOM:
        turnVelocity = aligner(targetHeading);
        break;
      case MAINTAIN_CURRENT:
        turnVelocity = aligner(targetHeading);
        break;
      case NEUTRAL:
        integral = 0;
        previousError = 0;
        break;
    }

    while (Controller.ButtonB.pressing()) {
      AllDriveMotors.stop(hold);
    }

    if (Controller.ButtonL1.pressing()) {
      // Outtake
      intakeVelocity = -100;
    } else if (Controller.ButtonR1.pressing()) {
      // Intake
      intakeVelocity = 100;
    } else {
      intakeVelocity = 0;
    }

    if (!Controller.ButtonLeft.pressing()) {
      prevLeftPressed = false;
    }
    if (!Controller.ButtonRight.pressing()) {
      prevRightPressed = false;
    }
    if (!Controller.ButtonX.pressing()) {
      prevXPressed = false;
    }

    if (Controller.ButtonR2.pressing()) {
      // Raise the intake
      LiftPneumatics.set(true);
    } else if (Controller.ButtonL2.pressing()) {
      // Lower the intake
      LiftPneumatics.set(false);
    } 

    drive(forwardVelocity, strafeVelocity, turnVelocity, robotOriented);
    intake(intakeVelocity);

    // Sleep the task for a short amount of time to
    // prevent wasted resources.
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
