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

motor_group Left(LeftFront, LeftMiddle, LeftBack);
motor_group Right(RightFront, RightMiddle, RightBack);

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
