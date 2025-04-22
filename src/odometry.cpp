/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odometry.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
#include "common.h"
using namespace vex;

// define used instances of motors and sensors as extern here because they are defined in robotconfig files
// Motor Groups
extern motor_group LeftMotors;
extern motor_group RightMotors;

//Sensors
extern inertial InertialA;

//Globals
double XPosition = 0.0;
double YPosition = 0.0;
double heading = 0.0;
float lastEncoder = 0.0;

// Constants
const float WHEEL_DIAMETER = 2.75; // in inches
const float TICKS_PER_REV = 360.0;

//Run as a thread
void updateOdometry(){
    while(true) {
        // Get current heading in radians
        heading = InertialA.heading();
        float theta = heading * M_PI / 180.0;

        // Get encoder position
        float currentEncoder = LeftMotors.position(degrees);
        float deltaTicks = currentEncoder - lastEncoder;
        lastEncoder = currentEncoder;

        // Convert ticks to distance
        float distance = (deltaTicks / TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER);

        // Update x, y using simple forward kinematics
        XPosition += distance * cos(theta);
        YPosition += distance * sin(theta);

        wait(50, msec);  // 20 Hz update rate
    }
}

float getXposition(){
    return XPosition;
}

float getYposition(){
    return YPosition;
}







