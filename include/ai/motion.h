#pragma once
/*---------------------------------------------------------------------------*/
/*  Interruptible motion built on JAR.                                       */
/*                                                                           */
/*  JAR's moves block until they settle. An autonomous robot needs to give   */
/*  up on a move when it is stuck, when the plan changes, or when the match  */
/*  clock says to go park. These wrappers run the normal JAR move, but the   */
/*  supervisor (run from the AI background task) can stop it early by        */
/*  setting chassis.motion_abort. Each wrapper reports why it returned.      */
/*                                                                           */
/*  Use these in Isolation routines and AI behaviors instead of calling      */
/*  chassis.* directly, so deadlines and stuck detection always work.       */
/*---------------------------------------------------------------------------*/

namespace ai {

enum MoveResult { MOVE_DONE = 0, MOVE_STUCK, MOVE_DEADLINE, MOVE_ABORTED };

const char *move_result_name(MoveResult r);

// All coordinates are field inches (see ai/field.h).
MoveResult go_to(float x, float y, float max_volt = 10.0f, float timeout_ms = 4000.0f);
MoveResult go_to_pose(float x, float y, float heading, float max_volt = 10.0f,
                      float timeout_ms = 4000.0f);
MoveResult turn_to(float heading, float timeout_ms = 1500.0f);
MoveResult turn_toward(float x, float y, float timeout_ms = 1500.0f);
MoveResult drive(float inches, float max_volt = 8.0f, float timeout_ms = 2000.0f);

// Drive to a point `standoff` inches short of (x, y), facing it. Used to line
// up on objects, goals and toggles.
MoveResult approach(float x, float y, float standoff, float max_volt = 10.0f);

// Abort the move in progress (any task may call this).
void request_abort();

// Any move still running at this system time (ms) is stopped. 0 = no deadline.
// The match code uses this so a behavior can't eat the time needed to park.
void set_deadline(uint32_t system_ms);
uint32_t deadline();

// Called at 50 Hz by the AI background task.
void supervisor_tick();

// Stuck-detection tuning
extern float stuck_speed_ips;   // below this...
extern uint32_t stuck_time_ms;  // ...for this long, while far from the target = stuck

}  // namespace ai
