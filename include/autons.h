#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();
void odom_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

void autonOne();

// VEX AI Isolation routines (run through ai::run_isolation)
void iso_score_preload();
void iso_midfield();
