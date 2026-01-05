/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2886932                                                   */
/*    Created:      12/9/2025, 2:38:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
#include <algorithm>

using namespace vex;
using std::max;
using std::min;

brain MyMind;
controller Controller;

motor FrontLeft(PORT12, ratio6_1, true);
motor FrontRight(PORT13, ratio6_1);
motor BackLeft(PORT14, ratio6_1, true);
motor BackRight(PORT19, ratio6_1);

inertial Inertial(PORT7);

enum AlignmentStatus {ALIGN_WITH_FIELD,
                      INVERT,
                      MAINTAIN_CURRENT,
                      NEUTRAL};
AlignmentStatus alignment;

const double DEADZONE = 4;
const double kP = 0.57;
const double kI = 0.054;
const double kD = 0.24;

double error;
double previousError;
double integral;
double derivative;

double aligner(double targetHeading, double currentHeading) {

    error = targetHeading - currentHeading;

    // Makes it so that the error is along the shortest possible route to the
    // target. 
    // For example, if the target is 0 and your current heading is 270,
    // you would want the error to be 90 and not -270. This way, you spin 90
    // degrees clockwise and not 270 degrees counter-clockwise.
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral = integral + error;
    if (fabs(error) <= 30) integral = 0;
    
    if (fabs(error) <= 5) return 0;
    
    derivative = error - previousError;
    previousError = error; 

    return ((error * kP) + (integral * kI) + (derivative * kD));
}

void drive(double forward, double strafe, double turn) {

    if (fabs(forward) <= DEADZONE)  forward = 0;
    if (fabs(strafe) <= DEADZONE) strafe = 0;

    // Holonomic drive formula
    double frontLeftSpeed = forward + strafe + turn;
    double frontRightSpeed = forward - strafe - turn;
    double backLeftSpeed = forward - strafe + turn;
    double backRightSpeed = forward + strafe - turn;

    // Capping the motor speeds between -100 and +100
   /* frontLeftSpeed = max(-100.0, min(frontLeftSpeed, 100.0));
    frontRightSpeed = max(-100.0, min(frontRightSpeed, 100.0));
    backLeftSpeed = max(-100.0, min(backLeftSpeed, 100.0));
    backRightSpeed = max(-100.0, min(backRightSpeed, 100.0));*/

    // This segment may not be needed. Do some testing with and without it.
    if (fabs(frontLeftSpeed) <= 5) FrontLeft.stop(brake);
    if (fabs(frontRightSpeed) <= 5) FrontRight.stop(brake);
    if (fabs(backLeftSpeed) <= 5) BackLeft.stop(brake);
    if (fabs(backRightSpeed) <= 5) BackRight.stop(brake);

    FrontLeft.spin(fwd, frontLeftSpeed, velocityUnits::pct);
    FrontRight.spin(fwd, frontRightSpeed, velocityUnits::pct);
    BackLeft.spin(fwd, backLeftSpeed, velocityUnits::pct);
    BackRight.spin(fwd, backRightSpeed, velocityUnits::pct);

}

int main() {
   
    Inertial.calibrate(2);
    wait(2000, msec);
    Inertial.setHeading(0, degrees);

   // bool alignWithField = true;
    bool previousUpPressed = false;
    //bool invert = false;

    alignment = NEUTRAL;

    double forward = 0;
    double strafe = 0;
    double turn = 0;
    double targetHeading = 0;

    while(1) {

        forward = Controller.Axis3.position();
        strafe = Controller.Axis4.position();
        turn = Controller.Axis1.position();
        
        double currentHeading = Inertial.heading(degrees);

        // Toggle alignment mode if the up button is pressed.
        if (Controller.ButtonUp.pressing() == true) {
            alignment = ALIGN_WITH_FIELD;
        } else if (Controller.ButtonDown.pressing() == true) {
            alignment = INVERT;
        }
        
        // If the right joystick (used for rotating) is moved, then the bot
        // snaps out of auto-alignment mode. If the joystick is not moved and
        // the up button is toggled, the bot snaps into auto-align mode.
        if (fabs(turn) > DEADZONE) {
            // The right joystick is moved
            alignment = NEUTRAL;
            integral = 0;
            previousError = 0;
        }
        
        if (Controller.ButtonB.pressing()) {
            Inertial.calibrate();
            wait(2000, msec);
        }

        switch (alignment) {
                case ALIGN_WITH_FIELD:
                    targetHeading = 0;
                    turn = aligner(targetHeading, currentHeading);
                    break;
                case INVERT:
                    targetHeading = 180;
                    forward = -forward;
                    strafe = -strafe;
                    turn = aligner(targetHeading, currentHeading);
                    break;
                case MAINTAIN_CURRENT:
                    targetHeading = currentHeading;
                    turn = aligner(targetHeading, currentHeading);
                    break;
                case NEUTRAL:
                    break;
            }

        previousUpPressed = Controller.ButtonUp.pressing();

        drive(forward, strafe, turn);

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

