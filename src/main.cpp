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
#include "intialize.h"
#include "simplemotion.h"
#include "proportionalmotion.h"

int main() {
    intialize();


    /*
    //Simple Movement Testbed
    driveForwardSimple(1000, 50);
    wait(1, sec);
    driveReverseSimple(1000, 50);
    wait(1, sec);
    turnRightSimple(90, 25);
    wait(1, sec);
    turnLeftSimple(90, 25);
    */

    /*
    //Proportinal Movement Testbed
    turnRightProportional(90);
    wait(1, sec);
    turnLeftProportional(90);
    wait(1, sec);
    driveForwardProportional(24);
    wait(1, sec);
    driveReverseProportional(24);
    wait(1, sec);
    */

    driveReverseStraight(48, 50);
    wait(1,sec);
    driveForwardStraight(48, 50);
    wait(1, sec);

    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
