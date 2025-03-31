/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       helperfunctions.cpp                                                  */
/*    Author:       davin                                                     */
/*    Created:      3/27/2025, 7:08:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

// Define constants 
const double WHEEL_DIAMETER = 2.5; 

//Convert Inches to Motor Encoder Degrees
double inchesToDegrees(double inches) { 
    return (inches / (M_PI * WHEEL_DIAMETER)) * 360.0; 
} 
