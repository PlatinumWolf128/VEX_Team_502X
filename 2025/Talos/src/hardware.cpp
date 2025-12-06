#include "vex.h"
#include "hardware.h"
#include <algorithm>

using namespace vex;
using std::max;
using std::min;

brain Brain;
controller Controller;

motor LeftFront(LEFT_FRONT_PORT, true);
motor LeftMiddle(LEFT_MIDDLE_PORT, true);
motor LeftBack(LEFT_BACK_PORT, true);

motor RightFront(RIGHT_FRONT_PORT);
motor RightMiddle(RIGHT_MIDDLE_PORT);
motor RightBack(RIGHT_BACK_PORT);

motor IntakeFrontBottom(INTAKE_FRONT_BOTTOM_PORT);
motor IntakeFrontTop(INTAKE_FRONT_TOP_PORT);
motor IntakeBackBottom(INTAKE_BACK_BOTTOM_PORT);
motor IntakeBackTop(INTAKE_BACK_TOP_PORT);

motor_group Left(LeftFront, LeftMiddle, LeftBack);
motor_group Right(RightFront, RightMiddle, RightBack);

pneumatics BottomRampPneumatics(Brain.ThreeWirePort.C);
pneumatics TopRampPneumatics(Brain.ThreeWirePort.D);
pneumatics Extender(Brain.ThreeWirePort.E);

optical OpticalSensor(OPTICAL_SENSOR_PORT, false);

void robotDrive(double frontBackSpeed, double turnSpeed) {

    // The arcade-drive formula:
    double leftSideSpeed = (frontBackSpeed + turnSpeed) * 0.98;
    double rightSideSpeed = (frontBackSpeed - turnSpeed);
    // leftSideSpeed is being multiplied by 0.98 in order to stop the robot from
    // pulling to the right when it drives forwards.

    // Caps the velocity value for either side to keep it between -100 and +100.
    // For example, if leftSideSpeed is somehow set to 300, then the min()
    // function returns 100, causing the max() function to also return 100 and
    // capping leftSideSpeed at 100. If leftSideSpeed is 35, however, the
    // min() function returns 35 and so does the max() function, so
    // leftSideSpeed falls within this constraint and doesn't change. 
    leftSideSpeed = max(-100.0, min(leftSideSpeed, 100.0));
    rightSideSpeed = max(-100.0, min(rightSideSpeed, 100.0));

    if (leftSideSpeed == 0) {
        Left.stop(brake);
    } 
    
    if (rightSideSpeed == 0) {
        Right.stop(brake);
    } 

    Left.spin(fwd, leftSideSpeed, pct);
    Right.spin(fwd, rightSideSpeed, pct);

}

void intakeMechanism(IntakeState intakeState) {
   
    // If we're using pneumatics, this remembers whether or not the piston has
    // been extended to change the orientation of the ramp
    static bool extended = false;
    
    switch (intakeState) {
        
        case INTAKE_TO_TOP:
            // The rollers pull the block into the upper hopper
            IntakeBackTop.spin(reverse);
            IntakeBackBottom.spin(reverse);
            IntakeFrontTop.spin(fwd);               
            if (extended == false) {
                // Extends the back of the ramp to allow for intaking
                TopRampPneumatics.set(true);
                extended = true;
            }
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Intaking");
            break;

        case INTAKE_TO_BOTTOM:
            // The bottom rollers push the block into the lower hopper.
            IntakeBackBottom.spin(fwd);
            break;
            
        case OUTTAKE_TO_TOP:
            // Every roller works to push the block upwards and to the front
            IntakeFrontTop.spin(fwd);
            IntakeBackBottom.spin(reverse);
            IntakeBackTop.spin(reverse);
            //IntakeFrontBottom.spin(fwd);
            if (extended) {
                // In theory retracts the back of the ramp to allow for
                // outtaking
                TopRampPneumatics.set(false);
                extended = false;
            }
            // To anchor the bot better and cancel out recoil when scoring, the
            // drivetrain holds in place and resists all movement.
            Left.stop(hold);
            Right.stop(hold);
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Outtaking to top"); 
            break;

        case OUTTAKE_TO_MIDDLE:
            // The front rollers work to push blocks out through the center of
            // the intake system, while the bottom back roller helps move it up.
            IntakeFrontTop.spin(reverse);
            IntakeBackBottom.spin(reverse);
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Outtaking to middle");
            break;

        case OUTTAKE_TO_BOTTOM:
            // The front-middle and back rollers work to pull blocks out of the hopper
            // and push them out of the intake through the bottom.
            IntakeFrontBottom.spin(reverse);
            IntakeBackBottom.spin(reverse);
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Outtaking to bottom");
            break;

        case NEUTRAL:
            // Nothing happens and the motors stop moving
            IntakeFrontTop.stop(brake);
            IntakeBackBottom.stop(brake);
            IntakeBackTop.stop(brake);
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Neutral");
            break;

        default:
            // In theory you should never reach this stage
            Controller.Screen.clearLine(1);
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("You shouldn't be here");
            break;
    }

}

int colorDetector() {
    
    if (OpticalSensor.color() == vex::color::red) {
        return RED_DETECTED;
    } else if (OpticalSensor.color() == vex::color::blue) {
        return BLUE_DETECTED;
    } else {
        return NOTHING_DETECTED;
    }
    
}