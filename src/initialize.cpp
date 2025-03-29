/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       initilize.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
using namespace vex;

//Sensors
extern inertial InertialA;

void intialize(){
    InertialA.calibrate();
    wait(3,sec);
}
