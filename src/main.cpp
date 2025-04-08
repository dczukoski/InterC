/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;

// include other needed cpp files
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"

int main() {
    intialize();
    thread a(brainDisplay);

    /*
    //Heading Tests    
    turnLeftToHeading(270);
    wait(1, sec);
    turnLeftToHeading(90);
    wait(1,sec);
    turnLeftToHeading(270);

    turnRightToHeading(90);
    wait(1, sec);
    turnRightToHeading(270);
    wait(1,sec);
    turnRightToHeading(90);
    */

    /*
    //Simple Movement Tests
    driveForwardSimple(1000, 50);
    wait(1, sec);
    driveReverseSimple(1000, 50);
    wait(1, sec);
    turnRightSimple(90, 25);
    wait(1, sec);
    turnLeftSimple(90, 25);
    */

    /*
    //Proportinal Movement Tests
    turnRightProportional(90);
    wait(1, sec);
    turnLeftProportional(90);
    wait(1, sec);
    driveForwardProportional(36);
    wait(1, sec);
    driveReverseProportional(36);
    wait(1, sec);
    */

    
    //Drive Straight Tests
    //driveReverseStraight(48, 50);
    //wait(1,sec);
    //driveForwardStraight(48, 50);
    //wait(1, sec);
    
    driveForwardPD(56,50);
    wait(1, sec);
}
