#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:
motor leftFrontMotor=motor(PORT10, true);
motor leftBackTopMotor=motor(PORT8, false);
motor leftBackBottomMotor=motor(PORT9, true);
motor rightFrontMotor=motor(PORT20, false);
motor rightBackTopMotor=motor(PORT18, true);
motor rightBackBottomMotor=motor(PORT19, false);
//Left Motors:x

void vexcodeInit( void ) {
  // nothing to initialize
}