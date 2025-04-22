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
#include "simplemotion.h"
#include "proportionalmotion.h"
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
        float currentEncoder = (LeftMotors.position(degrees) + RightMotors.position(degrees)) /2;
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


// Returns the straight-line distance to the target point
float getDistanceToTarget(float targetX, float targetY) {
    float deltaX = targetX - getXposition();
    float deltaY = targetY - getYposition();
    return sqrt(deltaX * deltaX + deltaY * deltaY);
  }
  
// Returns the heading you need to face the target point (0–360°)
float getHeadingToTarget(float targetX, float targetY) {
    float deltaX = targetX - getXposition();
    float deltaY = targetY - getYposition();
  
    float angleToTargetDeg = atan2(deltaY, deltaX) * 180.0 / M_PI;
  
    if (angleToTargetDeg < 0) angleToTargetDeg += 360.0;
  
    return angleToTargetDeg;
}

void driveDirectToPoint(float targetX, float targetY){
    double dist = getDistanceToTarget(targetX, targetY);
    double degs = wrapAngle(getHeadingToTarget(targetX, targetY));

    if(degs>180){
        turnLeftToHeading(degs);
    }else{
        turnRightToHeading(degs);
    }
    driveForwardStraightPD(dist, 50);
}

