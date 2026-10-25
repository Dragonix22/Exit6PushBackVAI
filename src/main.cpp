#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

motor_group(leftFrontMotor, leftBackTopMotor, leftBackBottomMotor),

//Right Motors:
motor_group(rightFrontMotor, rightBackTopMotor, rightBackBottomMotor),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT1,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
//TODO (Exit6): with ZERO_TRACKER_ODOM the "tracker" is the right drive side, so this should be
//POSITIVE and about half the track width (center of robot to center of the right wheels).
//-2 makes odom drift during turns. Measure and fix, then re-run odom_test.
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/*---------------------------------------------------------------------------*/
/*                       Isolation routine selector                          */
/*                                                                           */
/*  In VEX AI the first autonomous period is the 15 s Isolation Period. The  */
/*  routine picked here runs then; the Interaction Period is always the AI.  */
/*  The JAR test routines are kept so you can still tune the drive.          */
/*---------------------------------------------------------------------------*/

struct RoutineEntry {
  const char *name;
  void (*fn)(void);
};

static RoutineEntry routines[] = {
  {"ISO: score preload", iso_score_preload},
  {"ISO: midfield", iso_midfield},
  {"ISO: do nothing", 0},
  {"JAR: autonOne", autonOne},
  {"JAR: drive_test", drive_test},
  {"JAR: turn_test", turn_test},
  {"JAR: swing_test", swing_test},
  {"JAR: full_test", full_test},
  {"JAR: odom_test", odom_test},
  {"JAR: tank_odom_test", tank_odom_test},
};
static const int NUM_ROUTINES = sizeof(routines) / sizeof(routines[0]);

int current_auton_selection = (ROBOT_ID == ROBOT_BIG) ? 0 : 1;
bool auto_started = false;

static bool connected_to_competition() {
  return Competition.isFieldControl() || Competition.isCompetitionSwitch();
}

static const char *alliance_mode_name() {
  switch (ai::alliance_mode) {
    case ai::ALLIANCE_FORCE_RED:  return "RED (forced)";
    case ai::ALLIANCE_FORCE_BLUE: return "BLUE (forced)";
    default:                      return "AUTO from GPS";
  }
}

/**
 * Runs before the match. Calibrates sensors and starts the AI background task
 * (keep the robot still for ~3 s after starting the program), then shows a
 * status screen:
 *  - tap the top-left to change the Isolation routine
 *  - tap the top-right to change how the alliance is chosen
 *  - when NOT connected to a field, tap TEST MATCH to run a full practice
 *    match (15 s Isolation + 1:45 Interaction); controller B stops it.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  ai::init();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Exit6 VAI  %s robot  battery %d%%",
                         ROBOT_ID == ROBOT_BIG ? "24in" : "18in", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 40, "ISO: %s", routines[current_auton_selection].name);
    Brain.Screen.printAt(5, 60, "Alliance: %s", alliance_mode_name());
    ai::draw_status(90);

    bool comp = connected_to_competition();
    if (!comp) {
      Brain.Screen.drawRectangle(300, 195, 175, 40);
      Brain.Screen.printAt(310, 222, ai::test_match_running() ? "RUNNING (B=stop)" : "TEST MATCH");
    }

    if (Brain.Screen.pressing()) {
      int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
      while (Brain.Screen.pressing()) { wait(10, msec); }
      if (y < 75 && x < 240) {
        current_auton_selection = (current_auton_selection + 1) % NUM_ROUTINES;
      } else if (y < 75) {
        ai::alliance_mode = (ai::AllianceMode)((ai::alliance_mode + 1) % 3);
      } else if (!comp && y > 190 && x > 300 && !ai::test_match_running()) {
        ai::start_test_match(routines[current_auton_selection].fn);
      }
    }
    wait(100, msec);
  }
}

/**
 * Autonomous callback. VEX AI field control has historically run BOTH periods
 * as autonomous, so the first call is Isolation and the second is Interaction.
 *
 * The flag is cleared BEFORE running Isolation: the field kills this task when
 * Isolation ends, so code after the routine may never run. (VEX's ai_demo sets
 * its flag after the routine, which only works if the routine finishes early.)
 * If a match is replayed, restart the program so the flag resets.
 */

static bool first_auto = true;

void autonomous(void) {
  auto_started = true;
  if (first_auto) {
    first_auto = false;
    ai::run_isolation(routines[current_auton_selection].fn);
  } else {
    ai::run_interaction();
  }
}

/**
 * Driver-control callback. Section 7 treats the Interaction Period like the
 * Driver Controlled Period, but robots stay autonomous and no human may drive.
 * So when connected to a field (or competition switch) this runs the AI.
 * Only when NOT connected (bench/practice) does the controller drive the robot.
 */

void usercontrol(void) {
  if (connected_to_competition()) {
    auto_started = true;
    ai::run_interaction();
    return;
  }

  while (1) {
    if (ai::test_match_running()) {
      // Practice safety stop. Never used under field control.
      if (controller(primary).ButtonB.pressing()) ai::emergency_stop();
    } else {
      //Replace this line with chassis.control_tank(); for tank drive
      //or chassis.control_holonomic(); for holo drive.
      chassis.control_arcade();
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
