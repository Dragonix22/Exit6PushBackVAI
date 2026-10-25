/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-id.h"
#include "robot-config.h"
#include "JAR-Template/odom.h"
#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"
#include "autons.h"

// VEX AI: official Jetson comms (from VEX-Robotics/VAIC_26_27) + our AI code
#include "ai/ai_jetson.h"
#include "ai/ai.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)