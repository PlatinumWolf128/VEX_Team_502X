#include "vex.h"
#include "hardware.h"
#include <algorithm>

using namespace vex;
using std::max;
using std::min;

brain Brain;
controller Controller;

motor LeftFront(LEFT_FRONT_PORT, true);
motor LeftMiddle(LEFT_MIDDLE_PORT);
motor LeftBack(LEFT_BACK_PORT);

motor RightFront(RIGHT_FRONT_PORT, true);
motor RightMiddle(RIGHT_MIDDLE_PORT);
motor RightBack(RIGHT_BACK_PORT);
