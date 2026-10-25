#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:

#if ROBOT_ID == ROBOT_SMALL
#warning "Building for the 18 in robot with the 24 in robot's drive config. Set its ports here and its constants in main.cpp."
#endif

// Drive motors. These assume BLUE (600 rpm) cartridges: with 3.25" wheels and a
// 0.6 external ratio, that is a 360 rpm drive. The gear setting matters because
// ZERO_TRACKER_ODOM measures distance with these motor encoders; the default
// (green, 18:1) would make every odom distance 3x off on blue motors.
// TODO: change ratio6_1 if your cartridges are not blue.
motor leftFrontMotor=motor(PORT10, ratio6_1, true);
motor leftBackTopMotor=motor(PORT8, ratio6_1, false);
motor leftBackBottomMotor=motor(PORT9, ratio6_1, true);
motor rightFrontMotor=motor(PORT20, ratio6_1, false);
motor rightBackTopMotor=motor(PORT18, ratio6_1, true);
motor rightBackBottomMotor=motor(PORT19, ratio6_1, false);

// GPS: gps(port, x offset, y offset, units, heading offset).
// Offsets are from the robot's turning center to the sensor, x to the right
// and y forward, so GPS readings describe the robot center, not the sensor.
// Heading offset is the angle the sensor faces relative to the robot's front
// (180 if the sensor faces backward). Measure these on the real robot.
// IMPORTANT: enter the same offsets in the Jetson web dashboard; the Jetson
// reads the GPS separately and does not learn the offsets from the Brain.
// TODO: measure and set.
#if ROBOT_ID == ROBOT_BIG
gps GPS = gps(GPS_PORT, 0.0, 0.0, mm, 0.0);
#else
gps GPS = gps(GPS_PORT, 0.0, 0.0, mm, 0.0);
#endif

void vexcodeInit( void ) {
  // nothing to initialize
}
