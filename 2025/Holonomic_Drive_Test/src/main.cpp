/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2886932                                                   */
/*    Created:      12/9/2025, 2:38:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain MyMind;
controller Controller;

motor FrontLeft(PORT12);
motor FrontRight(PORT13, true);
motor BackLeft(PORT14);
motor BackRight(PORT15, true);

void drive(int forward, int strafe, int turn) {

    // Theoretical holonomic drive formula
    FrontLeft.spin(fwd, (forward + strafe + turn), velocityUnits::pct);
    FrontRight.spin(fwd, (forward - strafe - turn), velocityUnits::pct);
    BackLeft.spin(fwd, (forward - strafe + turn), velocityUnits::pct);
    BackRight.spin(fwd, (forward + strafe - turn), velocityUnits::pct);

}

int main() {
   
    while(1) {
        
        int forward = Controller.Axis3.position();
        int strafe = Controller.Axis4.position();
        int turn = Controller.Axis1.position();

        drive(forward, strafe, turn);

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

