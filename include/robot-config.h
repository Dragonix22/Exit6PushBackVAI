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

// TODO: set real ports. Must not clash with the drive (8,9,10,18,19,20) or
// the inertial sensor (1). These differ per robot if the wiring differs.
#define INERTIAL_PORT    PORT1
#define GPS_PORT         PORT11
#define LINK_RADIO_PORT  PORT12

extern gps GPS;

void  vexcodeInit( void );
