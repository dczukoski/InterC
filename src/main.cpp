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
#include "intialize.h"
#include "simplemotion.h"

int main() {
    intialize();

    driveForwardSimple(1000, 100);
    wait(3, sec);
    driveReverseSimple(1000, 100);
    wait(3, sec);
    turnRightSimple(90, 25);
    wait(3, sec);
    turnLeftSimple(90, 25);
    wait(3, sec);

    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
