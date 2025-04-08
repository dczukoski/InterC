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
#include "common.h"
using namespace vex;

// define used instances of motors and sensors as extern here because they are defined in robotconfig files
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
    double kp = .05;
    double min_speed = .25;
    LeftMotors.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftMotors.position(degrees) < target) {
        double proportion = target - LeftMotors.position(degrees); 
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveReverseProportional(double distance) {   //inches
    //Drive Forward Proportional
    double kp = .05;
    double min_speed = .25;
    LeftMotors.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftMotors.position(degrees) > -target) {
        double proportion = target + LeftMotors.position(degrees); 
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
        double kp = 1;
        double leftSpeed = speed + (error * kp);
        double rightSpeed = speed - (error * kp);

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
        double kp = 1;

        double leftSpeed = speed - (error * kp);
        double rightSpeed = speed + (error * kp);

        LeftMotors.spin(reverse, leftSpeed, pct);
        RightMotors.spin(reverse, rightSpeed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void turnRightToHeading(double targetHeading){
    double kp = .3;
    targetHeading = wrapAngle(targetHeading);

    double currentHeading = wrapAngle(InertialA.heading(degrees));
    double error = clockwiseDistance(currentHeading, targetHeading);
    double speed = error * kp;

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(InertialA.heading(degrees));
        error = clockwiseDistance(currentHeading, targetHeading);
        speed = error * kp;
        LeftMotors.spin(forward, speed, pct);
        RightMotors.spin(reverse, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);   
}

void turnLeftToHeading(double targetHeading){
    double kp = .3;
    targetHeading = wrapAngle(targetHeading);

    double currentHeading = wrapAngle(InertialA.heading(degrees));
    double error = counterclockwiseDistance(currentHeading, targetHeading);
    double speed = error * kp;

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(InertialA.heading(degrees));
        error = counterclockwiseDistance(currentHeading, targetHeading);
        speed = error * kp;
        LeftMotors.spin(reverse, speed, pct);
        RightMotors.spin(forward, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);   
}

void driveForwardPD(double distance, double speed) {   //inches
    //Drive Forward Proportional
    double kp = .5;
    double kd = .05;
    double min_speed = .25;
    
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    double derivative;
    double error = target - LeftMotors.position(degrees);
    double previousError = error;

    LeftMotors.resetPosition();

    while(fabs(error) > 2.0) { //
        previousError = error;
        error = target - LeftMotors.position(degrees); 
        speed = error*kp + derivative*kd + min_speed; //one way to break out of the loop
        derivative = error - previousError;
        LeftMotors.spin(fwd, speed, pct);
        RightMotors.spin(fwd, speed, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void driveForwardStraightPD(double distance, double speed) {   //inches
    //Drive Forward Proportional
    LeftMotors.resetPosition();
    double target_d = inchesToDegrees(distance); //Convert Inches to Motor Encoder Degrees

    double kp_d = .5;
    double kd_d = .05;
    double min_speed = .25;

    double derivative_d;
    double error_d = target_d - LeftMotors.position(degrees);
    double previousError_d = error_d;

    InertialA.resetRotation();
    double target_h = InertialA.rotation(degrees);    
    double speed_correction = 0.0;

    double kp_h = .5;
    double kd_h = .05;

    double derivative_h;
    double error_h = target_h - InertialA.rotation(degrees);
    double previousError_h = error_h; 

    while(fabs(error_d) > 2.0) { //
        //distance pd calcs
        previousError_d = error_d;
        error_d = target_d - LeftMotors.position(degrees); 
        speed = error_d*kp_d + derivative_d*kd_d + min_speed; //one way to break out of the loop
        derivative_d = error_d - previousError_d;
        
        //correction pd calcs
        previousError_h = error_h;
        error_h = target_h - InertialA.rotation(degrees); 
        speed_correction = error_h*kp_h + derivative_h*kd_h;
        derivative_h = error_h - previousError_h;
        
        LeftMotors.spin(fwd, speed + speed_correction, pct);
        RightMotors.spin(fwd, speed - speed_correction, pct);
    }
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

