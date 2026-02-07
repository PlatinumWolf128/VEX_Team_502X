#include "vex.h"
#include "hardware.h"
#include <algorithm>
#include <cmath>

brain Brain;
controller Controller;

// The drivetrain motors.
motor FrontLeft(FRONT_LEFT_PORT, ratio6_1, true);
motor FrontRight(FRONT_RIGHT_PORT, ratio6_1);
motor BackLeft(BACK_LEFT_PORT, ratio6_1, true);
motor BackRight(BACK_RIGHT_PORT, ratio6_1);

// The motor group for all the drive motors.
motor_group AllDriveMotors(FrontLeft, FrontRight, BackLeft, BackRight);

// The motors for the front flex-wheels in the intake.
motor FrontLeftFlexwheel(LEFT_FLEXWHEEL_PORT);
motor FrontRightFlexwheel(RIGHT_FLEXWHEEL_PORT, true);
motor LowerIntake(LOWER_INTAKE_PORT, true);
motor UpperIntakeLeft(UPPER_INTAKE_LEFT);
motor UpperIntakeRight(UPPER_INTAKE_RIGHT, true);
motor IntakeExit(INTAKE_EXIT_PORT);

// The motor group for the flexwheel motors.
motor_group IntakeMotors(FrontLeftFlexwheel, FrontRightFlexwheel, LowerIntake, UpperIntakeLeft, UpperIntakeRight, IntakeExit);

// All the sensors.
inertial Inertial(INERTIAL_SENSOR_PORT);

// The pneumatics solenoids.
pneumatics LiftPneumatics(Brain.ThreeWirePort.A);

double error = 0;
double previousError = 0;
double integral = 0;

double aligner(double targetHeading) {

    // The PID constants.
    const double kP = 0.43;
    const double kI = 0.014;
    const double kD = 0.27;

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
    
    double derivative = error - previousError;
    previousError = error; 

    // The PID formula.
    return ((error * kP) + (integral * kI) + (derivative * kD));

}

void drive(double forward, double strafe, double turn, bool robotOrientedDrive) {

    // Applying the deadzone.
    if (fabs(forward) <= DEADZONE) forward = 0;
    if (fabs(strafe) <= DEADZONE) strafe = 0;

    double currentHeading = Inertial.heading();

    double x = strafe;
    double y = forward;

    if (!robotOrientedDrive) {
        strafe = (x * cos(currentHeading * DEG_TO_RAD)) - (y * sin(currentHeading * DEG_TO_RAD));
        forward = (x * sin(currentHeading * DEG_TO_RAD)) + (y * cos(currentHeading * DEG_TO_RAD));
    }

    // The holonomic drive formula.
    double frontLeftSpeed = forward + strafe + turn;
    double frontRightSpeed = forward - strafe - turn;
    double backLeftSpeed = forward - strafe + turn;
    double backRightSpeed = forward + strafe - turn;

    // If the motor speeds are zero, apply the brakes.
    if (fabs(frontLeftSpeed) + fabs(frontRightSpeed) == 0) {
        if (fabs(backLeftSpeed) + fabs(backRightSpeed) == 0) {
            AllDriveMotors.stop(brake);
        }
    }

    // Spin all the motors based on the speed values that are passed in.
    FrontLeft.spin(fwd, frontLeftSpeed, velocityUnits::pct);
    FrontRight.spin(fwd, frontRightSpeed, velocityUnits::pct);
    BackLeft.spin(fwd, backLeftSpeed, velocityUnits::pct);
    BackRight.spin(fwd, backRightSpeed, velocityUnits::pct);

}

void intake(double intakeVelocity) {

    if (intakeVelocity == 0) {
        // If intakeVelocity is 0, stop the motors
        IntakeMotors.stop(brake);
    } else {
        // If intakeVelocity is +100, then the blocks get intaked.
        // If intakeVelocity is -100, then the blocks get outtaked.
        IntakeMotors.spin(fwd, intakeVelocity, pct);
    }


}

double distanceToTime(double distanceInInches) {

    return distanceInInches * 16.3;

}