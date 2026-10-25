#include "vex.h"
#include <stdarg.h>

using namespace vex;

namespace ai {

using namespace field;

/*----------------------------- global objects ------------------------------*/
// VEX's Jetson comms class (from VAIC_26_27). Its constructor starts a
// high-priority thread that reads Jetson packets from USB serial.
ai::jetson jetson_comms;

// VEXlink: the big robot is the manager, the small robot the worker.
#if ROBOT_ID == ROBOT_BIG
TeamLink team_link(LINK_RADIO_PORT, TEAM_LINK_NAME, linkType::manager);
#else
TeamLink team_link(LINK_RADIO_PORT, TEAM_LINK_NAME, linkType::worker);
#endif

AllianceMode alliance_mode = ALLIANCE_AUTO;

/*-------------------------------- state ------------------------------------*/
static Period    s_period = PRE_MATCH;
static uint32_t  s_period_start = 0;
static float     s_period_len_s = 0;
static volatile int s_task = T_NONE;
static uint32_t  s_last_jetson_ms = 0;
static AI_RECORD s_rec;
static Alliance  s_alliance = RED;
static bool      s_started = false;

Period period() { return s_period; }
int current_task() { return s_task; }
Alliance alliance() { return s_alliance; }

uint32_t period_end_ms() {
  return s_period_start + (uint32_t)(s_period_len_s * 1000.0f);
}

float period_time_left_s() {
  if (s_period != ISOLATION && s_period != INTERACTION) return 0.0f;
  float left = s_period_len_s - (now_ms() - s_period_start) / 1000.0f;
  return left > 0.0f ? left : 0.0f;
}

bool jetson_ok() {
  return s_last_jetson_ms != 0 && now_ms() - s_last_jetson_ms < 500;
}

/*--------------------------- background task -------------------------------*/
static void publish_to_partner() {
  Pose p = pose.get();
  link_pkt::StatePayload st;
  memset(&st, 0, sizeof(st));
  st.x_mm = (int16_t)in_to_mm(p.x);
  st.y_mm = (int16_t)in_to_mm(p.y);
  st.heading_cdeg = (uint16_t)((int)(p.heading * 100.0f) % 36000);
  st.task = (uint8_t)s_task;
  st.flags = (pose.gps_good() ? link_pkt::F_GPS_GOOD : 0) |
             (jetson_ok() ? link_pkt::F_JETSON_OK : 0) |
             (mech::holding() ? link_pkt::F_HOLDING : 0) |
             (s_task == T_PARK ? link_pkt::F_PARKING : 0);
  st.held_class = mech::holding() ? (uint8_t)mech::held_class() : 0xFF;
  team_link.set_state(st);

  link_pkt::FieldPayload f;
  world.make_field_payload(f);
  team_link.set_field(f);
}

static int ai_task() {
  uint32_t last_jetson = 0, last_link = 0, last_log = 0;
  int32_t last_packets = -1;
  while (true) {
    uint32_t now = now_ms();

    pose.update();
    supervisor_tick();

    // Jetson: ~15 Hz, matching the AI system's update rate.
    if (now - last_jetson >= 66) {
      last_jetson = now;
      int32_t pk = jetson_comms.get_packets();
      if (pk != last_packets) {  // only process new packets
        last_packets = pk;
        jetson_comms.get_data(&s_rec);
        s_last_jetson_ms = now;
        world.ingest_jetson(s_rec, now);
      }
      jetson_comms.request_map();  // only this task may request
    }

    if (now - last_link >= 100) {
      last_link = now;
      world.ingest_partner();
      publish_to_partner();
    }

    world.decay(now);

    if (now - last_log >= 100) {
      last_log = now;
      Pose p = pose.get();
      log::pose_row(p.x, p.y, p.heading, pose.gps_quality(), pose.gps_error_in(), s_task);
    }

    this_thread::sleep_for(20);  // 50 Hz
  }
  return 0;
}

/*-------------------------------- init -------------------------------------*/
void init() {
  if (s_started) return;
  log::init();
  log::event("BOOT robot=%d link=%s", ROBOT_ID, TEAM_LINK_NAME);

  // Calibrate while the robot sits still before the match, not during
  // Isolation (calibrating there wastes 1-2 of the 15 seconds).
  chassis.Gyro.calibrate();
  GPS.calibrate();
  timer t;
  while ((chassis.Gyro.isCalibrating() || GPS.isCalibrating()) && t.time() < 3000) {
    wait(20, msec);
  }

  mech::init();
  pose.init();
  if (!pose.seed_from_gps(1500)) {
    // GPS can't see the strip yet. Start odom anyway; pose.update() seeds
    // from the GPS as soon as it gets a good reading.
    chassis.set_coordinates(0, 0, 0);
    log::event("POSE not seeded at init (GPS quality %d)", pose.gps_quality());
  }

  thread t1(ai_task);
  s_started = true;
}

/*------------------------------- periods -----------------------------------*/
void run_isolation(Routine scripted) {
  s_period = ISOLATION;
  s_period_start = now_ms();
  s_period_len_s = ISOLATION_S;
  odom_constants();

  if (!pose.seeded()) pose.seed_from_gps(250);
  Pose p = pose.get();
  if (alliance_mode == ALLIANCE_FORCE_RED) s_alliance = RED;
  else if (alliance_mode == ALLIANCE_FORCE_BLUE) s_alliance = BLUE;
  else s_alliance = side_of(p.x, p.y);
  world.us = s_alliance;  // keep objects already seen before the match

  log::event("ISOLATION start alliance=%s pose=(%.1f,%.1f,%.0f) seeded=%d",
             s_alliance == RED ? "RED" : "BLUE", p.x, p.y, p.heading, pose.seeded());

  // Stop moving just before the period ends.
  set_deadline(period_end_ms() - 100);
  s_task = T_NONE;
  if (scripted) scripted();
  chassis.drive_stop(brake);
  log::event("ISOLATION routine finished, %.1f s left", period_time_left_s());
}

void run_interaction() {
  s_period = INTERACTION;
  s_period_start = now_ms();
  s_period_len_s = INTERACTION_S;
  odom_constants();
  mech::stop();
  log::event("INTERACTION start jetson=%d link=%d", jetson_ok(), team_link.linked());

  while (true) {
    float left = period_time_left_s();
    if (left <= 0.2f) break;

    Task t = planner.choose(left);

    // Every task except parking must finish in time to go park.
    uint32_t end = period_end_ms();
    if (t.type != T_PARK && planner.role.parks) {
      set_deadline(end - (uint32_t)(planner.park_time_s() * 1000.0f));
    } else {
      set_deadline(end - 100);
    }

    s_task = t.type;
    log::event("TASK %s target=%d at (%.0f,%.0f) value=%.1f time=%.1f left=%.1f",
               task_name(t.type), t.target, t.x, t.y, t.value, t.time_s, left);
    TaskResult r = run_task(t);
    log::event("TASK %s -> %s", task_name(t.type), task_result_name(r));

    if (r == TASK_STUCK) escape();
    if (r != TASK_OK) wait(50, msec);  // don't spin if nothing works
  }

  s_task = T_NONE;
  chassis.drive_stop(brake);
  mech::stop();
  s_period = POST_MATCH;
}

/*------------------------------ test match ---------------------------------*/
static Routine        s_test_routine = 0;
static volatile bool  s_test_running = false;
static volatile bool  s_estop = false;
static vex::thread   *s_period_thread = 0;

static int iso_entry() {
  run_isolation(s_test_routine);
  return 0;
}

static int inter_entry() {
  run_interaction();
  return 0;
}

static void stop_period_thread() {
  if (s_period_thread) {
    s_period_thread->interrupt();  // same as the field ending the period
    delete s_period_thread;
    s_period_thread = 0;
  }
  chassis.drive_stop(brake);
  mech::stop();
  set_deadline(0);
}

static bool wait_or_estop(float seconds) {
  timer t;
  while (t.time() < seconds * 1000.0f) {
    if (s_estop) return false;
    wait(20, msec);
  }
  return true;
}

static int test_match_task() {
  log::event("TEST MATCH start");
  s_period_thread = new vex::thread(iso_entry);
  bool ok = wait_or_estop(ISOLATION_S);
  stop_period_thread();
  if (ok) {
    wait(500, msec);  // the field pauses between periods too
    s_period_thread = new vex::thread(inter_entry);
    wait_or_estop(INTERACTION_S);
    stop_period_thread();
  }
  s_period = POST_MATCH;
  s_task = T_NONE;
  log::event("TEST MATCH end%s", s_estop ? " (e-stop)" : "");
  s_test_running = false;
  return 0;
}

void start_test_match(Routine scripted) {
  if (s_test_running) return;
  s_test_routine = scripted;
  s_estop = false;
  s_test_running = true;
  thread t(test_match_task);
}

bool test_match_running() { return s_test_running; }

void emergency_stop() {
  s_estop = true;
  chassis.drive_stop(brake);
  mech::stop();
}

/*-------------------------------- screen -----------------------------------*/
void screen_print(int x, int y, const char *fmt, ...) {
  char raw[96], safe[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(raw, sizeof(raw), fmt, args);
  va_end(args);
  // printAt treats its text as a format string, so double any '%'.
  int j = 0;
  for (int i = 0; raw[i] != 0 && j < (int)sizeof(safe) - 2; i++) {
    if (raw[i] == '%') safe[j++] = '%';
    safe[j++] = raw[i];
  }
  safe[j] = 0;
  // Exactly three arguments, so only printAt(x, y, format, ...) can match.
  Brain.Screen.printAt(x, y, safe);
}

void draw_status(int y) {
  Pose p = pose.get();
  screen_print(5, y, "Jetson %s  pkts %ld  det %ld   ", jetson_ok() ? "OK" : "--",
               (long)jetson_comms.get_packets(), (long)s_rec.detectionCount);
  screen_print(5, y + 20, "Link %s  partner %s   ", team_link.linked() ? "UP" : "down",
               world.partner.valid ? "seen" : "--");
  screen_print(5, y + 40, "GPS q%d  err %.1fin  %s   ", pose.gps_quality(),
               pose.gps_error_in(), pose.seeded() ? "seeded" : "NOT SEEDED");
  screen_print(5, y + 60, "Pose %.1f, %.1f, %.0f  side %s   ", p.x, p.y, p.heading,
               side_of(p.x, p.y) == RED ? "RED" : "BLUE");
  screen_print(5, y + 80, "Task %s  objs %d  %s   ", task_name(s_task),
               world.count_pickable(), log::enabled() ? log::file_name() : "no SD");
}

}  // namespace ai
