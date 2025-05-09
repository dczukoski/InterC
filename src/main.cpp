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
#include "odometry.h"

int main() {
    intialize();
    thread a(brainDisplay);
    thread b(controllerDisplay);
    thread c(updateOdometry);

    //initializeOdometry(18,18);

    driveToPoint(20,20);
    wait(1, sec);
    driveToPoint(0,40);
    wait(1, sec);
    driveToPoint(0,0);
}
