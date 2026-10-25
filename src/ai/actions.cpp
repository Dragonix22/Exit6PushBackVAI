#include "vex.h"

using namespace vex;

namespace ai {

using namespace field;

// Robot geometry for lining up. TODO: measure on the real robot.
static const float OBJECT_STANDOFF_IN = 16.0f;  // stop this far from an object, then creep in
static const float INTAKE_REACH_IN    = 7.0f;   // robot center to where the intake grabs
static const float GOAL_STANDOFF_IN   = 12.0f;  // robot center to goal center when placing
static const float TOGGLE_STANDOFF_IN = 13.0f;  // robot center to toggle when flipping
static const float LOADER_STANDOFF_IN = 12.0f;  // robot center to loader when collecting
static const uint32_t LOAD_WAIT_MS    = 4000;

const char *task_result_name(TaskResult r) {
  switch (r) {
    case TASK_OK:          return "OK";
    case TASK_FAILED:      return "FAILED";
    case TASK_STUCK:       return "STUCK";
    case TASK_OUT_OF_TIME: return "OUT_OF_TIME";
  }
  return "?";
}

static TaskResult from_move(MoveResult r) {
  switch (r) {
    case MOVE_DONE:     return TASK_OK;
    case MOVE_STUCK:    return TASK_STUCK;
    case MOVE_DEADLINE: return TASK_OUT_OF_TIME;
    default:            return TASK_FAILED;
  }
}

static float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

void escape() {
  log::event("ESCAPE");
  drive(-10.0f, 8.0f, 800);
  Pose p = pose.get();
  turn_to(wrap360(p.heading + 50.0f), 800);
}

/*------------------------------- behaviors ---------------------------------*/
static TaskResult do_get_object(const Task &t) {
  const Track *tr = world.track(t.target);
  if (!tr) return TASK_FAILED;
  int id = tr->id, cls = tr->cls;
  float tx = tr->x, ty = tr->y;
  world.note_claim(id);  // so the partner doesn't chase the same one

  MoveResult r = approach(tx, ty, OBJECT_STANDOFF_IN);
  if (r != MOVE_DONE) {
    world.note_release(id);
    return from_move(r);
  }

  // The estimate is better up close. Let a fresh frame arrive and re-read it.
  wait(150, msec);
  tr = world.track(id);
  if (!tr || tr->conf < 0.25f) {
    log::event("GET_OBJECT lost track %d", id);
    world.note_release(id);
    return TASK_FAILED;
  }
  tx = tr->x;
  ty = tr->y;
  r = turn_toward(tx, ty);
  if (r != MOVE_DONE) {
    world.note_release(id);
    return from_move(r);
  }

  Pose p = pose.get();
  float d = hypotf(tx - p.x, ty - p.y) - INTAKE_REACH_IN;
  mech::intake_begin();
  if (d > 0.5f) r = drive(d, 5.0f, 1500);  // creep in with the intake running
  if (mech::intake_finish(600)) {
    mech::set_held(cls);
    world.note_picked(id);
    return TASK_OK;
  }
  mech::stop();
  world.note_release(id);
  return r == MOVE_DONE ? TASK_FAILED : from_move(r);
}

static TaskResult do_score_held(const Task &t) {
  const Goal &g = GOALS[t.target];
  MoveResult r = approach(g.x, g.y, GOAL_STANDOFF_IN);
  if (r != MOVE_DONE) return from_move(r);
  int cls = mech::held_class();
  if (!mech::place()) return TASK_FAILED;
  world.note_scored(t.target, cls);
  drive(-8.0f, 6.0f, 1000);  // back off before turning so we don't clip the stack
  return TASK_OK;
}

static TaskResult do_load(const Task &t) {
  const Loader &l = LOADERS[t.target];
  // Square up to the wall in front of the loader's bottom opening.
  float into_field = l.x < 0 ? 1.0f : -1.0f;
  MoveResult r = go_to(l.x + into_field * LOADER_STANDOFF_IN, l.y);
  if (r != MOVE_DONE) return from_move(r);
  r = turn_to(l.x < 0 ? 270.0f : 90.0f);
  if (r != MOVE_DONE) return from_move(r);

  mech::intake_begin();
  bool got = mech::intake_finish(LOAD_WAIT_MS);
  if (!got) {
    mech::stop();
    return TASK_FAILED;
  }
  world.note_load_taken();
  // TODO: an optical sensor in the intake could tell a Cup from a Pin here.
  mech::set_held(match_load_pin(world.us));
  drive(-6.0f, 6.0f, 800);
  return TASK_OK;
}

static TaskResult do_set_toggle(const Task &t) {
  const Toggle &tg = TOGGLES[t.target];
  float r_face = tg.face_heading * (float)M_PI / 180.0f;
  // Stop square in front of the toggle.
  MoveResult r = go_to(tg.x - sinf(r_face) * TOGGLE_STANDOFF_IN,
                       tg.y - cosf(r_face) * TOGGLE_STANDOFF_IN);
  if (r != MOVE_DONE) return from_move(r);
  r = turn_to(tg.face_heading);
  if (r != MOVE_DONE) return from_move(r);
  if (!mech::flip_toggle(world.us)) return TASK_FAILED;
  world.note_toggle(t.target, world.us);
  // A toggle only counts if no robot is touching it at the end: always back off.
  drive(-10.0f, 8.0f, 1000);
  return TASK_OK;
}

static TaskResult do_park(const Task &t) {
  world.our_midfield_robots_planned = world.partner.parking ? 2 : 1;
  for (int attempt = 0; attempt < 3; attempt++) {
    MoveResult r = go_to(t.x, t.y, 10.0f, 4000);
    Pose p = pose.get();
    if (in_midfield(p.x, p.y, 3.0f) || r == MOVE_DEADLINE) break;
    escape();
  }
  chassis.drive_stop(hold);
  // Sit still until the field disables us. Don't touch anything.
  while (true) wait(50, msec);
  return TASK_OK;
}

static TaskResult do_search(const Task &t) {
  MoveResult r = go_to(t.x, t.y);
  if (r != MOVE_DONE) return from_move(r);
  // Face the middle of the field and sweep so the camera sees more of it.
  Pose p = pose.get();
  float center = wrap360(atan2f(-p.x, -p.y) * 180.0f / (float)M_PI);
  const float sweep[3] = {0.0f, -40.0f, 40.0f};
  for (int i = 0; i < 3; i++) {
    r = turn_to(wrap360(center + sweep[i]), 1000);
    if (r != MOVE_DONE) return from_move(r);
    wait(400, msec);
  }
  return TASK_OK;
}

TaskResult run_task(const Task &t) {
  switch (t.type) {
    case T_GET_OBJECT: return do_get_object(t);
    case T_SCORE_HELD: return do_score_held(t);
    case T_LOAD:       return do_load(t);
    case T_SET_TOGGLE: return do_set_toggle(t);
    case T_PARK:       return do_park(t);
    case T_SEARCH:     return do_search(t);
    default:           wait(100, msec); return TASK_FAILED;
  }
}

/*--------------------------- mechanism stubs -------------------------------*/
// TODO: replace these with the real intake / lift / claw code once the robots
// exist. Keep the same function names so the behaviors above don't change.
namespace mech {

bool simulate = true;

static bool s_holding = false;
static int  s_held = NO_CLASS;

void init() {
  // TODO: set brake modes, home the lift, close the claw, etc.
}

void stop() {
  // TODO: stop intake/lift motors.
}

bool holding() { return s_holding; }
int held_class() { return s_holding ? s_held : NO_CLASS; }

void set_held(int cls) {
  s_held = cls;
  s_holding = true;
}

void intake_begin() {
  // TODO: spin the intake.
}

bool intake_finish(uint32_t timeout_ms) {
  if (simulate) {
    wait(300, msec);
    s_holding = true;
    return true;
  }
  timer t;
  while (t.time() < timeout_ms) {
    // TODO: return true once a distance/optical sensor sees the object:
    //   if (IntakeDistance.objectDistance(mm) < 40) { stop(); s_holding = true; return true; }
    wait(10, msec);
  }
  stop();
  return false;
}

bool place() {
  if (simulate) {
    wait(500, msec);
    s_holding = false;
    s_held = NO_CLASS;
    return true;
  }
  // TODO: lift / release sequence. Return false if it didn't work.
  return false;
}

bool flip_toggle(field::Alliance want) {
  (void)want;
  if (simulate) {
    wait(400, msec);
    return true;
  }
  // TODO: toggle mechanism. Which way to push depends on its current color.
  return false;
}

void release_all() {
  s_holding = false;
  s_held = NO_CLASS;
  // TODO: open claw / reverse intake.
}

}  // namespace mech
}  // namespace ai
