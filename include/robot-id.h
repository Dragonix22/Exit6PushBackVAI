#pragma once

/*---------------------------------------------------------------------------*/
/*  Which of our two VEX AI robots this build is for.                        */
/*                                                                           */
/*  Both robots run this same project. Before downloading to a robot, set    */
/*  ROBOT_ID below (or pass -DROBOT_ID=2 to the compiler). The big robot is  */
/*  the VEXlink "manager" and the small robot is the "worker"; VEXlink needs */
/*  exactly one of each.                                                     */
/*---------------------------------------------------------------------------*/

#define ROBOT_BIG   1   // 24" robot: main scorer (match loads, field pins)
#define ROBOT_SMALL 2   // 18" robot: toggles, midfield, opportunistic scoring

#ifndef ROBOT_ID
#define ROBOT_ID ROBOT_BIG
#endif

// VEXlink pairs radios by name. Must be identical on both robots, at least
// 12 characters, and unique at an event (include the team number).
// TODO: put your real team number in here.
#define TEAM_LINK_NAME "exit6_vai_override_2627"
