#ifndef HARDWARE_H
#define HARDWARE_H

#include "vex.h"

extern vex::brain Brain;
extern vex::controller Controller;

const int LEFT_FRONT_PORT = 1;
const int LEFT_MIDDLE_PORT = 2;
const int LEFT_BACK_PORT = 3;

const int RIGHT_FRONT_PORT = 4;
const int RIGHT_MIDDLE_PORT = 5;
const int RIGHT_BACK_PORT = 6;

extern vex::motor LeftFront;
extern vex::motor LeftMiddle;
extern vex::motor LeftBack;

extern vex::motor RightFront;
extern vex::motor RightMiddle;
extern vex::motor RightBack;

#endif