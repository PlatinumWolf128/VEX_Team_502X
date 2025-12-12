/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2886932                                                   */
/*    Created:      12/9/2025, 2:38:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <algorithm>

using namespace vex;
using std::max;
using std::min;
using std::abs;

brain MyMind;
controller Controller;

motor FrontLeft(PORT12, ratio36_1);
motor FrontRight(PORT13, ratio36_1, true);
motor BackLeft(PORT14, ratio36_1);
motor BackRight(PORT19, ratio36_1, true);

inertial Inertial(PORT7);

const double DEADZONE = 4;
const double kP = 0.4;
const double kI = 0.005;
const double kD = 0.3;

double error;
double previousError;
double integral;
double derivative;

double headingMaintainer(double targetHeading, double currentHeading) {

    error = targetHeading - currentHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral = integral + error;
    
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
    frontLeftSpeed = max(-100.0, min(frontLeftSpeed, 100.0));
    frontRightSpeed = max(-100.0, min(frontRightSpeed, 100.0));
    backLeftSpeed = max(-100.0, min(backLeftSpeed, 100.0));
    backRightSpeed = max(-100.0, min(backRightSpeed, 100.0));

    if (fabs(frontLeftSpeed) <= 10) {FrontLeft.stop(brake);}
    if (fabs(frontRightSpeed) <= 10) {FrontRight.stop(brake);}
    if (fabs(backLeftSpeed) <= 10) {BackLeft.stop(brake);}
    if (fabs(frontRightSpeed) <= 10) {FrontRight.stop(brake);}

    FrontLeft.spin(fwd, frontLeftSpeed, velocityUnits::pct);
    FrontRight.spin(fwd, frontRightSpeed, velocityUnits::pct);
    BackLeft.spin(fwd, backLeftSpeed, velocityUnits::pct);
    BackRight.spin(fwd, backRightSpeed, velocityUnits::pct);

}

int main() {
   
    Inertial.calibrate(2);
    wait(2000, msec);
    Inertial.setHeading(0, degrees);

    while(1) {

        double forward = Controller.Axis3.position();
        double strafe = Controller.Axis4.position();
        double turn = Controller.Axis1.position();
        double currentHeading = Inertial.heading(degrees);

        if (fabs(turn) < DEADZONE) {
            turn = headingMaintainer(0, currentHeading);
        } else {
            integral = 0;
            previousError = 0;
        }

        drive(forward, strafe, turn);

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

