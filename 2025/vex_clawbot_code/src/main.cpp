/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       sanjaynataraj                                             */
/*    Created:      9/9/2025, 6:44:18 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <algorithm>

using namespace vex;
using std::max;
using std::min;

brain Brain;
controller Controller;

motor leftDriveMotor(2, true);
motor rightDriveMotor(10);
motor clawMotor(11);
motor armMotor(20);


void arcadeDrive(double frontBackSpeed, double turnSpeed);
void tankDrive(double leftSideSpeed, double rightSideSpeed);
void moveArm(double armVelocity, bool joystickAndNotButton);

int main() {
    
    bool arcadeDriveEnabled = true;
    bool twoStickArcadeDrive = false;
    
    while(1) {
        
        clawMotor.setVelocity(50, pct);
        armMotor.setVelocity(50, pct);

        leftDriveMotor.setBrake(hold);
        rightDriveMotor.setBrake(hold);
        clawMotor.setBrake(brake);
        armMotor.setBrake(hold);

        double controllerAxis1 = Controller.Axis1.position();
        double controllerAxis2 = Controller.Axis2.position();
        double controllerAxis3 = Controller.Axis3.position();
        double controllerAxis4 = Controller.Axis4.position();        

        if (arcadeDriveEnabled) {
            if (twoStickArcadeDrive) {
                arcadeDrive(controllerAxis3, controllerAxis1);
            } else {
                arcadeDrive(controllerAxis3, controllerAxis4);
            }
            moveArm(controllerAxis2, !twoStickArcadeDrive);
        } else {
            tankDrive(controllerAxis3, controllerAxis2);
            moveArm(controllerAxis2, false);
        }

        if (Controller.ButtonL1.pressing()) {
            clawMotor.spin(fwd);
        } else if (Controller.ButtonR1.pressing()) {
            clawMotor.spin(reverse);
        } else {
            clawMotor.stop();
        }

        if (Controller.ButtonX.pressing()) {
            twoStickArcadeDrive = !twoStickArcadeDrive;
        } else if (Controller.ButtonA.pressing()) {
            arcadeDriveEnabled = !arcadeDriveEnabled;
        }



        // Gonna leave this here because VEX added it and I'm worried removing
        // it will break something...
        this_thread::sleep_for(10);
    }

}

void arcadeDrive(double frontBackSpeed, double turnSpeed) {

    // This is the arcade drive formula:
    double leftMotorSpeed = (frontBackSpeed + turnSpeed);
    double rightMotorSpeed = (frontBackSpeed - turnSpeed);
    leftMotorSpeed = max(-100.0, min(leftMotorSpeed, 100.0));
    rightMotorSpeed = max(-100.0, min(rightMotorSpeed, 100.0));

    leftDriveMotor.spin(fwd, leftMotorSpeed, velocityUnits::pct);
    rightDriveMotor.spin(fwd, rightMotorSpeed, velocityUnits::pct);

    if(frontBackSpeed == 0 && turnSpeed == 0) {
        leftDriveMotor.stop();
        rightDriveMotor.stop();
    }

}

void tankDrive(double leftSideSpeed, double rightSideSpeed) {

    leftDriveMotor.spin(fwd, leftSideSpeed, velocityUnits::pct);
    rightDriveMotor.spin(fwd, rightSideSpeed, velocityUnits::pct);

}

void moveArm(double armVelocity, bool joystickAndNotButton) {
    
    if (joystickAndNotButton) {
        armMotor.spin(fwd, armVelocity, velocityUnits::pct);
    } else {
        if (Controller.ButtonL2.pressing()) {
            armMotor.spin(fwd);
        } else if (Controller.ButtonR2.pressing()) {
            armMotor.spin(reverse);
        } else {
            armMotor.stop();
        }
    }
}