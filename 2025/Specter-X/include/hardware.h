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
const int BACK_RIGHT_PORT = PORT15;

// The motor ports for the front flex-wheels in the intake.
const int LEFT_FLEXWHEEL_PORT = PORT3;
const int RIGHT_FLEXWHEEL_PORT = PORT4;
const int LOWER_INTAKE_PORT = PORT5;
const int UPPER_INTAKE_LEFT = PORT7;
const int UPPER_INTAKE_RIGHT = PORT8;
const int INTAKE_EXIT_PORT = PORT11;

// The port for all the sensors.
const int INERTIAL_SENSOR_PORT = PORT19;

// The motors for the drivetrain.
extern motor FrontLeft;
extern motor FrontRight;
extern motor BackLeft;
extern motor BackRight;

// A motor group with all the drive motors.
extern motor_group AllDriveMotors;

// The motors for the front flex-wheels in the intake.
extern motor FrontLeftFlexwheel;
extern motor FrontRightFlexwheel;
extern motor LowerIntake;
extern motor UpperIntakeLeft;
extern motor UpperIntakeRight;
extern motor IntakeExit;

// A motor group with all of the intake system motors.
extern motor_group IntakeMotors;

// The sensors.
extern inertial Inertial;
extern optical Optical;

// The solenoids for the pneumatics systems.
extern pneumatics LiftPneumatics;

// This is the deadzone value. If the joystick's position along an axis is less
// than or equal to this value, then a value of zero is returned for that axis
// instead. 
// This is done because the joysticks are imperfect and don't always
// return to position zero, so this establishes a range of position values that
// act as position zero.
const double DEADZONE = 3;

// Ratio for converting from degrees to radians.
const double DEG_TO_RAD = M_PI/180;

// The possible states for the drivetrain's alignment system.
enum AlignmentState {NORTH,
                     SOUTH,
                     CUSTOM,
                     MAINTAIN_CURRENT,
                     NEUTRAL};

// Some values needed for the aligner() function's PID
extern double error;
extern double previousError;
extern double integral;

/**
 * Controls the drivetrain using the holonomic drive formula and the values for
 * forwards/backwards velocity, strafing velocity, and angular velocity that are
 * passed in. 
 * 
 * However, this also has a mode where it uses the current heading of the bot
 * and drives in such a way so that no matter which way the robot is facing, if
 * the driver pushes the joystick forward, the robot drives forwards relative to
 * the driver.
 * 
 */
void drive(double forward, double strafe, double turn, bool robotOrientedDrive);


/**
 * Uses PID and an inertial sensor reading to return a turn value. This turn
 * value is then passed into the drive function to help automatically align the
 * bot with a desired heading.
 * 
 * @param targetHeading
 * The heading you want the bot to align to, in degrees.
 * 
 * @return
 * Returns a value of type double, that you then pass into the drive function as
 * the "turn" parameter.
 *  
 */
double aligner(double targetHeading);

void intake(double intakeVelocity);

double distanceToTime(double distanceInInches);


#endif