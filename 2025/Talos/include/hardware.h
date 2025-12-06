#ifndef HARDWARE_H
#define HARDWARE_H

#include "vex.h"

// This represents whether or not we are the red alliance during a match, which
// is vital for the color-sorter to operate correctly. Make sure to double check
// this before EVERY MATCH.
const bool weAreTheRedAlliance = true;

// The brain and controller objects.
extern vex::brain Brain;
extern vex::controller Controller;

// The port number for the optical sensor.
const int OPTICAL_SENSOR_PORT = vex::PORT21;

// The port numbers for the motors on the left side of the drivetrain.
const int LEFT_FRONT_PORT = vex::PORT19;
const int LEFT_MIDDLE_PORT = vex::PORT15;
const int LEFT_BACK_PORT = vex::PORT17;

// The port numbers for the motors on the right side of the drivetrain.
const int RIGHT_FRONT_PORT = vex::PORT6;
const int RIGHT_MIDDLE_PORT = vex::PORT5;
const int RIGHT_BACK_PORT = vex::PORT4;

// The port numbers for the motors used in the intake mechanism.
const int INTAKE_FRONT_BOTTOM_PORT = vex::PORT12;
const int INTAKE_FRONT_TOP_PORT = vex::PORT13;
const int INTAKE_BACK_BOTTOM_PORT = vex::PORT14;
const int INTAKE_BACK_TOP_PORT = vex::PORT18;

// The motors on the left side of the drivetrain.
extern vex::motor LeftFront;
extern vex::motor LeftMiddle;
extern vex::motor LeftBack;

// The motors on the right side of the drivetrain.
extern vex::motor RightFront;
extern vex::motor RightMiddle;
extern vex::motor RightBack;

// The motors used in the intake mechanism.
extern vex::motor IntakeFrontBottom;
extern vex::motor IntakeFrontTop;
extern vex::motor IntakeBackBottom; 
extern vex::motor IntakeBackTop;

// These are the motor group objects for the drivetrain, each one representing
// all the motors on one side of the drivetrain.
extern vex::motor_group Left;
extern vex::motor_group Right;

// These are the solenoids that control our pneumatics systems.
extern vex::pneumatics BottomRampPneumatics;
extern vex::pneumatics TopRampPneumatics;
extern vex::pneumatics Extender;

// The optical sensor object.
extern vex::optical OpticalSensor;

// The possible return values for the colorDetector() function.
const int NOTHING_DETECTED = 0;
const int RED_DETECTED = 1;
const int BLUE_DETECTED = 2;

// The speed at which all the motors in the intake subsystem move at,
// represented as a percentage of the maximum possible speed.
const double intakeMotorSpeed = 100;

// The possible states for the intake mechanism state machine and the variable
// that represents them. 
enum IntakeState {NEUTRAL, 
                  INTAKE_TO_TOP,
                  INTAKE_TO_BOTTOM, 
                  OUTTAKE_TO_TOP, 
                  OUTTAKE_TO_MIDDLE, 
                  OUTTAKE_TO_BOTTOM};
    
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

/**
 * Controls the intake mechanism based on which "state" the user puts the robot
 * into:
 * 
 * Pressing button L1 causes the robot to be put into the "intake" state
 * and it intakes blocks. 
 * 
 * Pressing button L2 puts the robot into the "outtake to the bottom" state, and
 * it outtakes blocks from the bottom of the intake system.
 * 
 * Pressing button R1 puts the robot in the "outtake to
 * the top" state and it outtakes blocks from the top of the intake system.
 * 
 * Pressing button R2 puts the robot into the "outtake to the middle" state, and
 * it outtakes blocks from the middle of the intake system.
 * 
 * The states are listed in order of priority. If L1 and R1 are both pressed for
 * example, then the bot would still only be put into the "intake" state.
 * Releasing all of these buttons puts the bot into the "neutral" state where
 * the intake is not moving.
 * 
 * @param intakeState
 * An enumeration of type IntakeState that represents the current state that the
 * intake system is in.
 * 
 */
void intakeMechanism(IntakeState intakeState);

/**
 * Detects the color of the block entering the intake system, and reports it to
 * the bot so that the block can be sorted.
 * 
 * @return
 * Returns an integer representing the color of the block.
 * 
 * If 1 is returned, a red block was detected.
 * 
 * If 2 is returned, a blue block was detected.
 * 
 * If 0 is returned, neither red nor blue was detected. 
 */
int colorDetector();

#endif