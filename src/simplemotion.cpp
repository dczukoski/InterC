/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       simplemotion.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
using namespace vex;

// define used instances of motors and sensors as extern here because they are defined in robotconfig files

// Six Motor Blue Gear Drive
extern motor LeftFrontMotor;
extern motor RightFrontMotor;
extern motor LeftBackMotor;
extern motor RightBackMotor;
extern motor LeftStackMotor;
extern motor RightStackMotor;

// Motor Groups
extern motor_group LeftMotors;
extern motor_group RightMotors;

//Sensors
extern inertial InertialA;

//Drive Forward Simple
void driveForwardSimple(double distance, double speed) {   
   LeftMotors.resetPosition();
   while(LeftMotors.position(degrees)<distance){
        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

//Drive Reverse Simple
void driveReverseSimple(double distance, double speed) {   
    LeftMotors.resetPosition();
    while(LeftMotors.position(degrees) > -distance){
         LeftMotors.spin(reverse, speed, pct);
         RightMotors.spin(reverse, speed, pct);
     }
     LeftMotors.stop(brake);
     RightMotors.stop(brake);
 }
 
//Simple Turn Right
void turnRightSimple(double target, double speed) {   
    InertialA.resetRotation();
    wait(.25, sec); //Sometimes Intertial/Gyro Sensors need some time to settle
    while(InertialA.rotation(degrees) < target) {
        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(reverse, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

//Simple Turn Left
void turnLeftSimple(double target, double speed) {   
    InertialA.resetRotation();
    wait(.25, sec); //Sometimes Intertial or Gyro Sensors need some time to settle
    while(InertialA.rotation(degrees) > -target) {
        LeftMotors.spin(reverse, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}