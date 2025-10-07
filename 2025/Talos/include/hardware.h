#ifndef HARDWARE_H
#define HARDWARE_H

#include "vex.h"

extern vex::brain Brain;
extern vex::controller Controller;

// The port numbers for the motors on the left side of the drivetrain.
const int LEFT_FRONT_PORT = 1;
const int LEFT_MIDDLE_PORT = 2;
const int LEFT_BACK_PORT = 3;

// The port numbers for the motors on the right side of the drivetrain.
const int RIGHT_FRONT_PORT = 4;
const int RIGHT_MIDDLE_PORT = 5;
const int RIGHT_BACK_PORT = 6;

// The port numbers for the motors used in the intake mechanism.
const int INTAKE_FRONT_MIDDLE_PORT = 7;
const int INTAKE_FRONT_TOP_PORT = 8;
const int INTAKE_BACK_PORT = 9;

// The motors on the left side of the drivetrain.
extern vex::motor LeftFront;
extern vex::motor LeftMiddle;
extern vex::motor LeftBack;

// The motors on the right side of the drivetrain.
extern vex::motor RightFront;
extern vex::motor RightMiddle;
extern vex::motor RightBack;

// The motors used in the intake mechanism.
extern vex::motor IntakeFrontMiddle;
extern vex::motor IntakeFrontTop;
extern vex::motor IntakeBack; 

// These are the motor group objects for the drivetrain, each one representing
// all the motors on one side of the drivetrain.
extern vex::motor_group Left;
extern vex::motor_group Right;

// The speed at which all the motors in the intake subsystem move at,
// represented as a percentage of the maximum possible speed.
const double intakeMotorSpeed = 90;

enum IntakeState {NEUTRAL, INTAKE, OUTTAKE_TO_TOP, OUTTAKE_TO_BOTTOM};
extern IntakeState intakeState;

/**
 * Controls the drivetrain by adjusting the speed of the motors based on the
 * values for forwards velocity and angular velocity that are passed in. The
 * velocity values don't correspond to an actual speed value, but instead
 * basically represent what percentage of its maximum speed you want the robot
 * to move at. 
 * 
 * @param frontBackSpeed
 * The velocity of the robot in the forwards and backwards directions, ranging
 * from +100 (full speed forwards) to -100 (full speed reverse).
 * 
 * @param turnSpeed
 * The angular velocity of the robot as it turns about its own center, ranging
 * from +100 (full speed clockwise) to -100 (full speed counter-clockwise). 
 * 
 */
void robotDrive(double frontBackSpeed, double turnSpeed);

void intakeMechanism(IntakeState intakeState);

#endif