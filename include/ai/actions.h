#pragma once
/*---------------------------------------------------------------------------*/
/*  Behaviors: how a planner Task is carried out with motion + mechanisms.   */
/*                                                                           */
/*  The mechanism layer (ai::mech) is a STUB until the robots are built.     */
/*  With mech::simulate = true, the robot drives the whole plan and pretends */
/*  the intake/scoring worked, which is enough to test navigation, the       */
/*  planner, VEXlink and logging on a real field today.                      */
/*---------------------------------------------------------------------------*/

namespace ai {

enum TaskResult { TASK_OK = 0, TASK_FAILED, TASK_STUCK, TASK_OUT_OF_TIME };

const char *task_result_name(TaskResult r);

TaskResult run_task(const Task &t);

// Back away and turn after getting stuck.
void escape();

namespace mech {

extern bool simulate;   // pretend mechanisms work (no hardware yet)

void init();
void stop();

bool holding();
int  held_class();      // field::ObjClass or NO_CLASS
void set_held(int cls); // after an intake, what vision said we picked up

// Start the intake, then call intake_finish to wait for an object to arrive.
void intake_begin();
bool intake_finish(uint32_t timeout_ms);

bool place();                          // put the held object on the goal in front
bool flip_toggle(field::Alliance want);// set the toggle in front to our color
void release_all();                    // open everything

}  // namespace mech
}  // namespace ai
