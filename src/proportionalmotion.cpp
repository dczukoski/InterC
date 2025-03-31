/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       proportionalmotion.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
#include "helperfunctions.h"
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

void turnRightProportional(double target) {   
    //Propotional Turn Right
    InertialA.resetRotation();
    while(InertialA.rotation(degrees) < target) {
        double proportion = target - InertialA.rotation(degrees); 
        double kp = .35;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(reverse, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void turnLeftProportional(double target) {   
    //Propotional Turn Left
    InertialA.resetRotation();
    while(InertialA.rotation(degrees) > -target) {
        double proportion = target + InertialA.rotation(degrees); 
        double kp = .35;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotors.spin(reverse, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveForwardProportional(double distance) {   //inches
    //Drive Forward Proportional
    LeftFrontMotor.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftFrontMotor.position(degrees) < target) {
        double proportion = target - LeftFrontMotor.position(degrees); 
        double kp = .05;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveReverseProportional(double distance) {   //inches
    //Drive Forward Proportional
    LeftFrontMotor.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftFrontMotor.position(degrees) > -target) {
        double proportion = target + LeftFrontMotor.position(degrees); 
        double kp = .05;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotors.spin(reverse, speed, pct);
        RightMotors.spin(reverse, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveForwardStraight(double distance, double speed) {    //inches
    InertialA.resetRotation();
    double targetRotation = InertialA.rotation(degrees); //save heading

    LeftMotors.resetPosition();
    double targetDistance = inchesToDegrees(distance); 

    while(LeftMotors.position(degrees) < targetDistance) {
        double error = targetRotation - InertialA.rotation(degrees);
        double kp = .2;

        double leftSpeed = speed - (error * kp);
        double rightSpeed = speed + (error * kp);

        LeftMotors.spin(fwd, leftSpeed, pct);
        RightMotors.spin(fwd, rightSpeed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveReverseStraight(double distance, double speed) {    //inches
    InertialA.resetRotation();
    double targetRotation = InertialA.rotation(degrees); //save heading

    LeftMotors.resetPosition();
    double targetDistance = inchesToDegrees(distance); 

    while(LeftMotors.position(degrees) > -targetDistance) {
        double error = targetRotation - InertialA.rotation(degrees);
        double kp = .2;

        double leftSpeed = speed + (error * kp);
        double rightSpeed = speed - (error * kp);

        LeftMotors.spin(reverse, leftSpeed, pct);
        RightMotors.spin(reverse, rightSpeed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}