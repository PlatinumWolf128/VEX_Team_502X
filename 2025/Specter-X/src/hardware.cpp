#include "vex.h"
#include "hardware.h"
#include <algorithm>
#include <cmath>

// The drivetrain motors.
motor FrontLeft(FRONT_LEFT_PORT, ratio6_1, true);
motor FrontRight(FRONT_RIGHT_PORT, ratio6_1);
motor BackLeft(BACK_LEFT_PORT, ratio6_1, true);
motor BackRight(BACK_RIGHT_PORT, ratio6_1);

// All the sensors.
inertial Inertial(INERTIAL_SENSOR_PORT);

void robotOrientedDrive(double forward, double strafe, double turn) {

    // Applying the deadzone.
    if (fabs(forward) <= DEADZONE) forward = 0;
    if (fabs(strafe) <= DEADZONE) strafe = 0;

    // The holonomic drive formula.
    double frontLeftSpeed = forward + strafe + turn;
    double frontRightSpeed = forward - strafe - turn;
    double backLeftSpeed = forward - strafe + turn;
    double backRightSpeed = forward + strafe - turn;

    // If the motor speeds are zero, apply the brakes.
    if (fabs(frontLeftSpeed) == 0) FrontLeft.stop(brake);
    if (fabs(frontRightSpeed) == 0) FrontRight.stop(brake);
    if (fabs(backLeftSpeed) == 0) BackLeft.stop(brake);
    if (fabs(backRightSpeed) == 0) BackRight.stop(brake);

    // Spin all the motors based on the speed values that are passed in.
    FrontLeft.spin(fwd, frontLeftSpeed, velocityUnits::pct);
    FrontRight.spin(fwd, frontRightSpeed, velocityUnits::pct);
    BackLeft.spin(fwd, backLeftSpeed, velocityUnits::pct);
    BackRight.spin(fwd, backRightSpeed, velocityUnits::pct);

}

double aligner(double targetHeading) {

    // The PID constants.
    const double kP = 0.62;
    const double kI = 0.044;
    const double kD = 0.24;

    double currentHeading = Inertial.heading(degrees);

    error = targetHeading - currentHeading;

    // Makes it so that the error is along the shortest possible route to the
    // target. 
    // For example, if the target is 0 and your current heading is 270,
    // you would want the error to be 90 and not -270. This way, you spin 90
    // degrees clockwise and not 270 degrees counter-clockwise.
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral = integral + error;
    if (fabs(error) <= 10) integral = 0;
    
    // If the current heading is close enough to the target heading, set the
    // error to 0 to reduce the effects of the PID and prevent overshoot.
    if (fabs(error) <= 5) return 0;
    
    derivative = error - previousError;
    previousError = error; 

    // The PID formula.
    return ((error * kP) + (integral * kI) + (derivative * kD));

}