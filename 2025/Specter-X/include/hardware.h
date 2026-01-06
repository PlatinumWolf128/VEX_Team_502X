#ifndef HARDWARE_H
#define HARDWARE_H

using namespace vex;

// The brain and controller objects.
extern brain Brain;
extern controller Controller;

// The motor ports for the drivetrain motors.
const int FRONT_LEFT_PORT = PORT12; 
const int FRONT_RIGHT_PORT = PORT13;
const int BACK_LEFT_PORT = PORT14;
const int BACK_RIGHT_PORT = PORT19;

// The motors for the drivetrain.
extern motor FrontLeft;
extern motor FrontRight;
extern motor BackLeft;
extern motor BackRight;

// This is the deadzone value. If the joystick's position along an axis is less
// than or equal to this value, then a value of zero is returned for that axis
// instead. 
// This is done because the joysticks are imperfect and don't always
// return to position zero, so this establishes a range of position values that
// act as position zero.
const double DEADZONE = 3;

// The possible states for the drivetrain's alignment system.
enum AlignmentStatus {NORTH,
                      SOUTH,
                      MAINTAIN_CURRENT,
                      NEUTRAL};

/**
 * Controls the drivetrain using the holonomic drive formula and the values for
 * forwards/backwards velocity, strafing velocity, and angular velocity that are
 * passed in. 
 */
void robotOrientedDrive(double forward, double strafe, double turn);


/**
 * Controls the drivetrain using the holonomic drive formula and the values for
 * forwards/backwards velocity, strafing velocity, and angular velocity that are
 * passed in. 
 * 
 * Unlike the other drive function however, this also takes a heading value and
 * drives in such a way so that no matter which way the robot is facing, if the
 * driver pushes the joystick forward, the robot drives forwards relative to the
 * driver.
 */
void fieldOrientedDrive(double forward, double strafe, double turn, double currentHeading);


#endif