#include "vex.h"
#include "hardware.h"
#include <algorithm>
#include <cmath>

// The drivetrain motors.
motor FrontLeft(FRONT_LEFT_PORT, ratio6_1, true);
motor FrontRight(FRONT_RIGHT_PORT, ratio6_1);
motor BackLeft(BACK_LEFT_PORT, ratio6_1, true);
motor BackRight(BACK_RIGHT_PORT, ratio6_1);

void robotOrientedDrive(double forward, double strafe, double turn) {

    // Applying the deadzone
    if (fabs(forward) <= DEADZONE) forward = 0;
    if (fabs(strafe) <= DEADZONE) strafe = 0;

    // The holonomic drive formula
    double frontLeftSpeed = forward + strafe + turn;
    double frontRightSpeed = forward - strafe - turn;
    double backLeftSpeed = forward - strafe + turn;
    double backRightSpeed = forward + strafe - turn;

    // If the motor speeds are zero, apply the brakes.
    if (fabs(frontLeftSpeed) == 0) FrontLeft.stop(brake);
    if (fabs(frontRightSpeed) == 0) FrontRight.stop(brake);
    if (fabs(backLeftSpeed) == 0) BackLeft.stop(brake);
    if (fabs(backRightSpeed) == 0) BackRight.stop(brake);

    FrontLeft.spin(fwd, frontLeftSpeed, velocityUnits::pct);
    FrontRight.spin(fwd, frontRightSpeed, velocityUnits::pct);
    BackLeft.spin(fwd, backLeftSpeed, velocityUnits::pct);
    BackRight.spin(fwd, backRightSpeed, velocityUnits::pct);


}