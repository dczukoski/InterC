/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       common.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
using namespace vex;

// define used instances of motors and sensors as extern here because they are defined in robotconfig files
// Motor Groups
extern motor_group LeftMotors;
extern motor_group RightMotors;

//Sensors
extern inertial InertialA;

void intialize(){
    InertialA.calibrate();
    wait(3,sec);
}

// Define constants 
const double WHEEL_DIAMETER = 2.5; 

//Convert Inches to Motor Encoder Degrees
double inchesToDegrees(double inches) { 
    return (inches / (M_PI * WHEEL_DIAMETER)) * 360.0; 
} 

// Function to wrap an angle within [0, 360]
double wrapAngle(double angle) {
    return fmod(angle + 360, 360);
}

// Function to compute clockwise distance from current to target angle
double clockwiseDistance(double currentAngle, double targetAngle) {
    double distance = targetAngle - currentAngle;
    if (distance < 0) {
        distance += 360.0; // Wrap around if negative
    }
    return distance;
}

// Function to compute counterclockwise distance from current to target angle
double counterclockwiseDistance(double currentAngle, double targetAngle) {
    double distance = currentAngle - targetAngle;
    if (distance < 0) {
        distance += 360.0; // Wrap around if negative
    }
    return distance;
}

//Debugging functions are used to display sensor values to the screen
void brainDisplay(){
    while(true){
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(1,30,"LeftM %f",LeftMotors.position(degrees));
        Brain.Screen.printAt(1,50,"RightM %f",RightMotors.position(degrees));
        Brain.Screen.printAt(1,70,"Rotation %f",InertialA.rotation(degrees));
        Brain.Screen.printAt(1,90,"Heading %f",InertialA.heading(degrees));
        wait(.25,sec);
    }
}
