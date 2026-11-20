#pragma once
using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;
//Every device must be declared "extern" here and created once in robot-config.cpp.

//Drive motors:
extern motor leftFrontMotor;
extern motor leftBackTopMotor;
extern motor leftBackBottomMotor;
extern motor rightFrontMotor;
extern motor rightBackTopMotor;
extern motor rightBackBottomMotor;

/*---------------------------------------------------------------------------*/
/*  VEX AI hardware                                                          */
/*                                                                           */
/*  - The Jetson / Raspberry Pi talks to the Brain over the Brain's micro-USB */
/*    port, so it does not use a Smart Port. That also means printf() goes   */
/*    to the Jetson: do not use printf for debugging (use ai::log instead).  */
/*  - The GPS sensor plugs into a Smart Port AND into the Jetson (USB-C).    */
/*  - VEXlink needs a SECOND V5 radio on its own Smart Port.                 */
/*---------------------------------------------------------------------------*/

// Every Smart Port can only be used once. The drive uses 8, 9, 10, 18, 19, 20.
// INERTIAL_PORT is passed to the JAR chassis in main.cpp, so change it here.
// TODO: GPS and the VEXlink radio are placeholders; set them to the ports you
// actually wire. They can differ per robot.
#define INERTIAL_PORT    PORT11
#define GPS_PORT         PORT13
#define LINK_RADIO_PORT  PORT12

extern gps GPS;

void  vexcodeInit( void );
