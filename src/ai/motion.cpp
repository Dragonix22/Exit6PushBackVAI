#include "vex.h"

using namespace vex;

namespace ai {

float stuck_speed_ips = 2.0f;
uint32_t stuck_time_ms = 700;

enum MoveKind { KIND_LINEAR, KIND_TURN };

static volatile bool s_active = false;
static MoveKind      s_kind = KIND_LINEAR;
static float         s_tx = 0, s_ty = 0, s_th = 0;
static volatile int  s_reason = MOVE_DONE;
static uint32_t      s_deadline = 0;
static uint32_t      s_start_ms = 0;
static uint32_t      s_slow_since = 0;

static float angle_diff(float a, float b) {
  float d = a - b;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

const char *move_result_name(MoveResult r) {
  switch (r) {
    case MOVE_DONE:     return "DONE";
    case MOVE_STUCK:    return "STUCK";
    case MOVE_DEADLINE: return "DEADLINE";
    case MOVE_ABORTED:  return "ABORTED";
  }
  return "?";
}

void set_deadline(uint32_t system_ms) { s_deadline = system_ms; }
uint32_t deadline() { return s_deadline; }

static void abort_with(int reason) {
  s_reason = reason;
  chassis.motion_abort = true;
}

void request_abort() {
  if (s_active) abort_with(MOVE_ABORTED);
}

static bool begin(MoveKind kind, float tx, float ty, float th) {
  uint32_t now = now_ms();
  if (s_deadline != 0 && now >= s_deadline) return false;  // no time left
  s_kind = kind;
  s_tx = tx;
  s_ty = ty;
  s_th = th;
  s_reason = MOVE_DONE;
  s_start_ms = now;
  s_slow_since = 0;
  chassis.motion_abort = false;
  s_active = true;
  return true;
}

static MoveResult end() {
  s_active = false;
  MoveResult r = MOVE_DONE;
  if (chassis.motion_abort) r = (MoveResult)s_reason;
  chassis.motion_abort = false;
  // JAR leaves the last voltage applied (for chaining). The AI usually does
  // something else next, so stop cleanly.
  chassis.drive_stop(brake);
  if (r != MOVE_DONE) log::event("MOVE ended %s", move_result_name(r));
  return r;
}

void supervisor_tick() {
  if (!s_active) return;
  uint32_t now = now_ms();

  if (s_deadline != 0 && now >= s_deadline) {
    abort_with(MOVE_DEADLINE);
    return;
  }

  // Stuck: not moving, but still far from where we're going.
  if (now - s_start_ms < 400) return;  // give it time to accelerate
  Pose p = pose.get();
  bool far, slow;
  if (s_kind == KIND_LINEAR) {
    far = hypotf(s_tx - p.x, s_ty - p.y) > 4.0f;
    slow = pose.speed_ips() < stuck_speed_ips;
  } else {
    far = fabsf(angle_diff(s_th, p.heading)) > 5.0f;
    slow = pose.turn_rate_dps() < 10.0f;
  }
  if (far && slow) {
    if (s_slow_since == 0) s_slow_since = now;
    else if (now - s_slow_since > stuck_time_ms) abort_with(MOVE_STUCK);
  } else {
    s_slow_since = 0;
  }
}

/*-------------------------------- moves ------------------------------------*/
MoveResult go_to(float x, float y, float max_volt, float timeout_ms) {
  if (!begin(KIND_LINEAR, x, y, 0)) return MOVE_DEADLINE;
  chassis.drive_to_point(x, y, chassis.drive_min_voltage, max_volt,
                         chassis.heading_max_voltage, chassis.drive_settle_error,
                         chassis.drive_settle_time, timeout_ms);
  return end();
}

MoveResult go_to_pose(float x, float y, float heading, float max_volt, float timeout_ms) {
  if (!begin(KIND_LINEAR, x, y, heading)) return MOVE_DEADLINE;
  chassis.drive_to_pose(x, y, heading, chassis.boomerang_lead, chassis.boomerang_setback,
                        chassis.drive_min_voltage, max_volt, chassis.heading_max_voltage,
                        chassis.drive_settle_error, chassis.drive_settle_time, timeout_ms);
  return end();
}

MoveResult turn_to(float heading, float timeout_ms) {
  if (!begin(KIND_TURN, 0, 0, heading)) return MOVE_DEADLINE;
  chassis.turn_to_angle(heading, chassis.turn_max_voltage, chassis.turn_settle_error,
                        chassis.turn_settle_time, timeout_ms);
  return end();
}

MoveResult turn_toward(float x, float y, float timeout_ms) {
  Pose p = pose.get();
  float h = atan2f(x - p.x, y - p.y) * 180.0f / (float)M_PI;
  if (h < 0) h += 360.0f;
  if (!begin(KIND_TURN, 0, 0, h)) return MOVE_DEADLINE;
  chassis.turn_to_point(x, y, 0, chassis.turn_max_voltage, chassis.turn_settle_error,
                        chassis.turn_settle_time, timeout_ms);
  return end();
}

MoveResult drive(float inches, float max_volt, float timeout_ms) {
  Pose p = pose.get();
  float r = p.heading * (float)M_PI / 180.0f;
  if (!begin(KIND_LINEAR, p.x + inches * sinf(r), p.y + inches * cosf(r), p.heading))
    return MOVE_DEADLINE;
  chassis.drive_distance(inches, p.heading, max_volt, chassis.heading_max_voltage,
                         chassis.drive_settle_error, chassis.drive_settle_time, timeout_ms);
  return end();
}

MoveResult approach(float x, float y, float standoff, float max_volt) {
  Pose p = pose.get();
  float dx = x - p.x, dy = y - p.y;
  float d = hypotf(dx, dy);
  MoveResult r = MOVE_DONE;
  if (d > standoff + 2.0f) {
    r = turn_toward(x, y);
    if (r != MOVE_DONE) return r;
    float k = (d - standoff) / d;
    r = go_to(p.x + dx * k, p.y + dy * k, max_volt);
    if (r != MOVE_DONE) return r;
  }
  return turn_toward(x, y);  // final alignment
}

}  // namespace ai
