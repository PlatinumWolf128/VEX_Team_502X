#include "vex.h"
#include "hardware.h"
#include <algorithm>

using namespace vex;
using std::max;
using std::min;

brain Brain;
controller Controller;

motor LeftFront(LEFT_FRONT_PORT);
motor LeftMiddle(LEFT_MIDDLE_PORT);
motor LeftBack(LEFT_BACK_PORT);

motor RightFront(RIGHT_FRONT_PORT, true);
motor RightMiddle(RIGHT_MIDDLE_PORT, true);
motor RightBack(RIGHT_BACK_PORT, true);

motor IntakeFrontMiddle(INTAKE_FRONT_MIDDLE_PORT);
motor IntakeFrontTop(INTAKE_FRONT_TOP_PORT);
motor IntakeBack(INTAKE_BACK_PORT);

motor_group Left(LeftFront, LeftMiddle, LeftBack);
motor_group Right(RightFront, RightMiddle, RightBack);

pneumatics Pneumatics(Brain.ThreeWirePort.A);

IntakeState intakeState = NEUTRAL;

void robotDrive(double frontBackSpeed, double turnSpeed) {

    // The arcade-drive formula
    double leftSideSpeed = (frontBackSpeed + turnSpeed);
    double rightSideSpeed = (frontBackSpeed - turnSpeed);

    // Caps the velocity value for either side to keep it between -100 and +100.
    // For example, if leftSideSpeed is somehow set to 300, then the min()
    // function returns 100, causing the max() function to also return 100 and
    // capping leftSideSpeed at 100. If leftSideSpeed is 35, however, the
    // min() function returns 35 and so does the max() function, so
    // leftSideSpeed falls within this constraint and doesn't change. 
    leftSideSpeed = max(-100.0, min(leftSideSpeed, 100.0));
    rightSideSpeed = max(-100.0, min(rightSideSpeed, 100.0));

    Left.spin(fwd, leftSideSpeed, pct);
    Right.spin(fwd, rightSideSpeed, pct);

}

void intakeMechanism(IntakeState intakeState) {
   
    // Disable if we decide to use pneumatics
    bool noNeedForPneumatics = true;

    // If we're using pneumatics, this remembers whether or not the piston has
    // been extended to change the orientation of the ramp
    static bool extended = false;
        
    switch (intakeState) {
        
        case INTAKE:
            if (noNeedForPneumatics) {
                // The back roller pulls the block into the hopper
                IntakeBack.spin(fwd);
                if (extended == false) {
                    // In theory extends the back of the ramp to allow for
                    // intaking
                    Pneumatics.set(true);
                }
            } else {
                // The back roller pushes the block upwards
                IntakeBack.spin(reverse);
            }
            // The middle roller pulls the block in towards the hopper
            IntakeFrontMiddle.spin(fwd);
            break;

        case OUTTAKE_TO_TOP:
            // Every roller works to push the block upwards and to the front
            IntakeFrontMiddle.spin(fwd);
            IntakeBack.spin(reverse);
            IntakeFrontTop.spin(fwd);
            if (noNeedForPneumatics == false && extended) {
                // In theory retracts the back of the ramp to allow for
                // outtaking
                Pneumatics.set(false);
            } 
            break;

        case OUTTAKE_TO_BOTTOM:
            // The middle and back rollers work to pull blocks out of the hopper
            // and push them out of the intake through the bottom
            IntakeFrontMiddle.spin(reverse);
            IntakeBack.spin(reverse);
            break;

        case NEUTRAL:
            // Nothing happens and the motors stop moving
            IntakeFrontMiddle.stop(brake);
            IntakeFrontTop.stop(brake);
            IntakeBack.stop(brake);

        default:
            // In theory you should never reach this stage
            Controller.Screen.setCursor(0, 0);
            Controller.Screen.print("You shouldn't be here.");
            break;

    }

}