#pragma once
/*---------------------------------------------------------------------------*/
/*  Match flow and the AI background task.                                   */
/*                                                                           */
/*  main.cpp calls:                                                          */
/*    ai::init()              in pre_auton                                   */
/*    ai::run_isolation(r)    the first time autonomous() runs               */
/*    ai::run_interaction()   the second autonomous(), OR drivercontrol()    */
/*                            when connected to field control                */
/*                                                                           */
/*  The AI background task (started by init) runs at 50 Hz: pose fusion,     */
/*  motion supervisor, Jetson requests (~15 Hz), world model, VEXlink, and   */
/*  logging. It is the ONLY place that calls jetson_comms.request_map().     */
/*---------------------------------------------------------------------------*/

namespace ai {

extern ai::jetson jetson_comms;
extern TeamLink   team_link;

enum Period { PRE_MATCH, ISOLATION, INTERACTION, POST_MATCH };

Period period();
float  period_time_left_s();
uint32_t period_end_ms();

typedef void (*Routine)(void);

void init();
void run_isolation(Routine scripted);
void run_interaction();

bool jetson_ok();       // a Jetson packet arrived in the last 0.5 s
int  current_task();    // TaskType being executed (for the screen/link)

// Alliance: detected from the GPS at the start of Isolation unless forced.
enum AllianceMode { ALLIANCE_AUTO, ALLIANCE_FORCE_RED, ALLIANCE_FORCE_BLUE };
extern AllianceMode alliance_mode;
field::Alliance alliance();

// Practice without field control: 15 s Isolation + 105 s Interaction,
// with the same thread kills a real field does at the end of each period.
void start_test_match(Routine scripted);
bool test_match_running();
void emergency_stop();

// Brain screen status block, drawn from main.cpp's pre_auton loop.
void draw_status(int y);

}  // namespace ai
