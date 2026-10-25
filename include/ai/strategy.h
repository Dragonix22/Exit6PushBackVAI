#pragma once
/*---------------------------------------------------------------------------*/
/*  Planner: picks the next task by expected points per second.              */
/*                                                                           */
/*  Every candidate task gets                                                */
/*     value  = expected change in (our score - their score)                 */
/*     time_s = expected seconds to finish it                                */
/*  and the planner runs the one with the best value / time_s. Scoring the  */
/*  MARGIN matters in Override: a red/yellow pin placed where blue owns the  */
/*  toggle gives red 5 but blue 10, so it is a net loss.                     */
/*                                                                           */
/*  Start simple and tune from logs. The numbers below are guesses until     */
/*  you time the real robot.                                                 */
/*---------------------------------------------------------------------------*/

namespace ai {

enum TaskType { T_NONE = 0, T_GET_OBJECT, T_SCORE_HELD, T_LOAD, T_SET_TOGGLE,
                T_PARK, T_SEARCH };

const char *task_name(int t);

struct Task {
  TaskType type;
  int      target;   // track id / goal index / loader index / quadrant
  float    x, y;     // where it happens, field inches
  float    value;    // expected change in margin (points)
  float    time_s;   // expected duration
  float    utility;  // value / time_s
};

// What this robot is allowed to do (set per robot in strategy.cpp).
struct RobotRole {
  bool picks_field_objects;
  bool uses_loaders;
  bool flips_toggles;
  bool parks;
};

class Planner {
 public:
  Planner();

  Task choose(float time_left_s);

  // Expected margin from placing one object of class cls on goal g.
  float placement_margin(int cls, int goal) const;
  int   best_goal_for(int cls, float &margin) const;

  // Seconds needed to reach the Midfield and park, plus buffer.
  float park_time_s() const;
  void  park_point(float &x, float &y) const;

  RobotRole role;

  // ---- tuning (time these on the real robot) ----
  float drive_speed_ips;   // average including turns
  float pickup_s;
  float place_s;
  float load_wait_s;       // waiting for the human to drop a match load
  float toggle_s;
  float park_buffer_s;
  float p_win_unknown_toggle;  // chance we own a neutral/unknown toggle at the end
  float p_keep_our_toggle;     // chance a toggle set to our color stays ours
  float p_win_midfield;        // chance we own the Midfield goal's yellow halves

 private:
  float travel_s(float x, float y) const;
  float p_we_own_quadrant(int q) const;
};

extern Planner planner;

}  // namespace ai
