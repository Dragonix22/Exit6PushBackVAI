// Host unit tests for the AI layer. They run the REAL JAR + AI code on a
// laptop against a fake vex:: API (tests/host/fake_vex), so you can check
// math and planner changes without a robot. Run tests/host/run_tests.sh.
//
// The build uses -Dprivate=public so tests can poke at internals.
#include "vex.h"
#include <stdio.h>
#include <math.h>

namespace vex {
uint32_t g_now_ms = 1000;
double g_motor_deg[22];
double g_motor_vel[22];
double g_gyro_rot = 0;
double g_gps_x_mm = 0, g_gps_y_mm = 0, g_gps_h = 0;
int g_gps_q = 100;
std::vector<uint8_t> g_link_tx;
}
using namespace vex;

competition Competition;
Drive chassis(ZERO_TRACKER_ODOM,
  motor_group(leftFrontMotor, leftBackTopMotor, leftBackBottomMotor),
  motor_group(rightFrontMotor, rightBackTopMotor, rightBackBottomMotor),
  PORT1, 3.25, 0.6, 360, PORT1, -PORT2, PORT3, -PORT4, 3, 2.75, 6.0, 1, -2.75, 5.5);

static int fails = 0, passes = 0;
#define CHECK(cond, ...) do { if (cond) { passes++; } else { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

// Drive the fake robot: set right-side encoder so odom moves forward `in` inches.
static double s_right_in = 0;
static void move_forward(double in) {
  s_right_in += in;
  // drive_in_to_deg_ratio = ratio/360*pi*D  => deg = in / ratio_in_per_deg
  double in_per_deg = 0.6 / 360.0 * M_PI * 3.25;
  g_motor_deg[PORT20] = s_right_in / in_per_deg;
  g_motor_deg[PORT10] = s_right_in / in_per_deg;
}
static void odom_step() {
  chassis.odom.update_position(chassis.get_ForwardTracker_position(),
                               chassis.get_SidewaysTracker_position(),
                               chassis.get_absolute_heading());
}
static void tick(int n = 1) {
  for (int i = 0; i < n; i++) { g_now_ms += 20; odom_step(); ai::pose.update(); }
}

static void test_link_roundtrip() {
  using namespace ai;
  ai::TeamLink a(PORT12, "exit6_vai_test_link", linkType::manager);
  ai::TeamLink b(PORT13, "exit6_vai_test_link", linkType::worker);

  link_pkt::StatePayload st;
  memset(&st, 0, sizeof(st));
  st.x_mm = -1234; st.y_mm = 567; st.heading_cdeg = 27050; st.task = T_LOAD;
  st.flags = link_pkt::F_GPS_GOOD | link_pkt::F_HOLDING; st.held_class = field::CUP;
  g_link_tx.clear();
  a.send_packet(link_pkt::STATE, &st, sizeof(st));
  CHECK(g_link_tx.size() == 16, "STATE packet should be 16 bytes, got %zu", g_link_tx.size());
  for (size_t i = 0; i < g_link_tx.size(); i++) b.process(g_link_tx[i]);
  link_pkt::StatePayload got;
  bool ok = b.partner_state(got);
  CHECK(ok, "partner state not received");
  CHECK(got.x_mm == -1234 && got.y_mm == 567 && got.heading_cdeg == 27050 && got.held_class == field::CUP,
        "state mismatch");

  // Corrupt a byte: must be rejected
  g_link_tx.clear();
  st.x_mm = 999;
  a.send_packet(link_pkt::STATE, &st, sizeof(st));
  g_link_tx[8] ^= 0x40;
  uint32_t errs = b.rx_errors;
  for (size_t i = 0; i < g_link_tx.size(); i++) b.process(g_link_tx[i]);
  b.partner_state(got);
  CHECK(b.rx_errors == errs + 1 && got.x_mm == -1234, "corrupt packet accepted");

  // Events: each sent 3x, interleaved; receiver must keep exactly one copy each
  link_pkt::EventPayload e1 = {7, link_pkt::EV_SCORED, 2, field::RED_YELLOW_PIN, 100, 200};
  link_pkt::EventPayload e2 = {8, link_pkt::EV_TOGGLE, 1, field::RED, 0, -1780};
  g_link_tx.clear();
  for (int r = 0; r < 3; r++) {
    a.send_packet(link_pkt::EVENT, &e1, sizeof(e1));
    a.send_packet(link_pkt::EVENT, &e2, sizeof(e2));
  }
  for (size_t i = 0; i < g_link_tx.size(); i++) b.process(g_link_tx[i]);
  link_pkt::EventPayload out;
  int n = 0, kinds = 0;
  while (b.pop_event(out)) { n++; kinds |= 1 << out.kind; }
  CHECK(n == 2, "expected 2 unique events, got %d", n);
  CHECK(kinds == ((1 << link_pkt::EV_SCORED) | (1 << link_pkt::EV_TOGGLE)), "wrong event kinds");

  // FIELD packet with 2 sightings (variable length)
  link_pkt::FieldPayload f;
  memset(&f, 0, sizeof(f));
  f.toggle_bits = 0x1B; f.loads_taken = 3; f.count = 2;
  f.s[0].cls = field::YELLOW_PIN; f.s[0].x_mm = 300; f.s[0].y_mm = -300;
  f.s[1].cls = field::CUP; f.s[1].x_mm = -10; f.s[1].y_mm = 20;
  g_link_tx.clear();
  a.send_packet(link_pkt::FIELD, &f, 3 + 2 * sizeof(link_pkt::Sighting));
  for (size_t i = 0; i < g_link_tx.size(); i++) b.process(g_link_tx[i]);
  link_pkt::FieldPayload fg;
  CHECK(b.partner_field(fg) && fg.count == 2 && fg.s[1].cls == field::CUP && fg.s[0].y_mm == -300,
        "field payload mismatch");

  // Event queue tx: 3 events pending, at most 2 go out per tick, none lost
  a.send_event(link_pkt::EV_CLAIM, 0, 0, 1, 1);
  a.send_event(link_pkt::EV_CLAIM, 0, 0, 2, 2);
  a.send_event(link_pkt::EV_CLAIM, 0, 0, 3, 3);
  int total_sends = 0;
  for (int i = 0; i < ai::TeamLink::EVENT_QUEUE; i++) total_sends += a.tx_event_sends_left_[i];
  CHECK(total_sends == 9, "each event should be queued 3x, got %d", total_sends);
}

static void test_pose_seed_and_fusion() {
  using namespace ai;
  // GPS: robot at (-1500 mm, -900 mm) facing 45 deg
  g_gps_x_mm = -1500; g_gps_y_mm = -900; g_gps_h = 45; g_gps_q = 100;
  pose.init();
  bool ok = pose.seed_from_gps(1000);
  Pose p = pose.get();
  CHECK(ok && pose.seeded(), "seed failed");
  CHECK(near(p.x, -1500 / 25.4f, 0.05f) && near(p.y, -900 / 25.4f, 0.05f) && near(p.heading, 45, 0.1f),
        "seeded pose wrong: %.2f %.2f %.1f", p.x, p.y, p.heading);
  CHECK(field::side_of(p.x, p.y) == field::RED, "should be on red side");

  // Odom drives 12 in forward at 45 deg, GPS agrees => tiny error
  for (int i = 0; i < 12; i++) { move_forward(1.0); tick(); }
  p = pose.get();
  float ex = -1500 / 25.4f + 12 * sinf(45 * M_PI / 180), ey = -900 / 25.4f + 12 * cosf(45 * M_PI / 180);
  CHECK(near(p.x, ex, 0.3f) && near(p.y, ey, 0.3f), "odom forward wrong: %.2f,%.2f vs %.2f,%.2f", p.x, p.y, ex, ey);

  // GPS says we're 3 in further right (+x) than odom: fusion should pull us over gradually
  g_gps_x_mm = (ex + 3.0f) * 25.4f; g_gps_y_mm = ey * 25.4f;
  tick(5);
  float after5 = pose.get().x;
  tick(200);
  float after200 = pose.get().x;
  CHECK(after5 > ex && after5 < ex + 1.5f, "fusion too fast/slow early: %.2f", after5);
  CHECK(near(after200, ex + 3.0f, 0.2f), "fusion didn't converge: %.2f vs %.2f", after200, ex + 3.0f);

  // A one-off 30 in GPS glitch is ignored...
  float x_before = pose.get().x;
  g_gps_x_mm += 30 * 25.4f;
  tick(3);
  g_gps_x_mm -= 30 * 25.4f;
  tick(1);
  CHECK(near(pose.get().x, x_before, 0.5f), "glitch moved pose: %.2f vs %.2f", pose.get().x, x_before);
  // ...but a persistent one (robot got shoved) snaps after ~0.5 s
  g_gps_x_mm += 20 * 25.4f;
  tick(30);
  CHECK(near(pose.get().x, x_before + 20, 0.5f), "persistent offset not snapped: %.2f", pose.get().x);
  g_gps_x_mm -= 20 * 25.4f;
  tick(30);  // snap back
}

static void test_reanchoring() {
  using namespace ai;
  // Put the robot at (10, 20) facing +X (heading 90) with GPS agreeing.
  g_gps_q = 0;  // no GPS corrections during this test
  pose.seed_manual(10, 20, 90);
  tick(30);
  world.reset(field::RED);
  world.camera_latency_ms = 0;

  // Jetson believes (wrongly) it's at the origin facing +Y. It projects an
  // object 20 in straight ahead at (0, 20 in). Re-anchoring must place it 20 in
  // ahead of OUR pose: (30, 20).
  AI_RECORD rec;
  memset(&rec, 0, sizeof(rec));
  rec.pos.framecnt = 5;
  rec.pos.x = 0; rec.pos.y = 0; rec.pos.az = 0;
  rec.detectionCount = 1;
  rec.detections[0].classID = field::RED_YELLOW_PIN;
  rec.detections[0].probability = 0.9f;
  rec.detections[0].depth = (20 - 6) / 39.3701f;
  rec.detections[0].screenLocation.x = 320;
  rec.detections[0].mapLocation.x = 0;
  rec.detections[0].mapLocation.y = 20 / 39.3701f;
  rec.detections[0].mapLocation.z = 3.0f / 39.3701f;
  world.ingest_jetson(rec, g_now_ms);
  const Track *t = world.nearest_pickable(field::NO_CLASS, 30, 20, 0.0f);
  CHECK(t && near(t->x, 30, 0.3f) && near(t->y, 20, 0.3f), "re-anchored track wrong: %.2f %.2f",
        t ? t->x : -1, t ? t->y : -1);

  // Jetson pose goes stale (frame counter frozen > 300 ms): fall back to depth.
  tick(20);  // 400 ms, robot still
  rec.detections[0].mapLocation.x = 5;  // garbage map position that must be ignored
  world.ingest_jetson(rec, g_now_ms);
  t = world.nearest_pickable(field::NO_CLASS, 30, 20, 0.0f);
  CHECK(t && near(t->x, 30, 0.5f) && near(t->y, 20, 0.5f), "depth fallback wrong: %.2f %.2f",
        t ? t->x : -1, t ? t->y : -1);
  CHECK(t && t->hits >= 2, "fallback detection should update the same track (hits=%d)", t ? t->hits : 0);

  // Object to the robot's right in the image (screen x > 320) => smaller y (robot faces +X).
  rec.detections[0].screenLocation.x = 320 + 100;
  world.reset(field::RED);
  world.ingest_jetson(rec, g_now_ms);
  t = world.nearest_pickable(field::NO_CLASS, 30, 17, 0.0f);
  CHECK(t && t->y < 20 - 2, "right-of-center object should be at lower y, got %.2f", t ? t->y : -1);

  // Elevated object next to a goal is "on goal", not pickable.
  world.reset(field::RED);
  pose.seed_manual(field::GOALS[2].x - 20, field::GOALS[2].y, 90);
  tick(30);
  rec.pos.framecnt++;
  rec.pos.x = (field::GOALS[2].x - 20) / 39.3701f; rec.pos.y = field::GOALS[2].y / 39.3701f; rec.pos.az = 90;
  rec.detections[0].mapLocation.x = field::GOALS[2].x / 39.3701f;
  rec.detections[0].mapLocation.y = field::GOALS[2].y / 39.3701f;
  rec.detections[0].mapLocation.z = 9.0f / 39.3701f;
  world.ingest_jetson(rec, g_now_ms);
  CHECK(world.count_pickable(0.0f) == 0, "stacked object counted as pickable");

  // Unseen-but-should-be-visible tracks lose confidence and disappear.
  world.reset(field::RED);
  pose.seed_manual(0, -40, 0);
  tick(30);
  rec.pos.framecnt++;
  rec.pos.x = 0; rec.pos.y = -40 / 39.3701f; rec.pos.az = 0;
  rec.detections[0].mapLocation.x = 0; rec.detections[0].mapLocation.y = -10 / 39.3701f;
  rec.detections[0].mapLocation.z = 3 / 39.3701f;
  world.ingest_jetson(rec, g_now_ms);
  CHECK(world.count_pickable(0.0f) == 1, "object not added");
  rec.detectionCount = 0;
  for (int i = 0; i < 12; i++) { tick(4); rec.pos.framecnt++; world.ingest_jetson(rec, g_now_ms); }
  CHECK(world.count_pickable(0.0f) == 0, "object that vanished in view was kept");
  g_gps_q = 100;
}

static void test_planner() {
  using namespace ai;
  using namespace ai::field;
  g_gps_q = 0;
  world.reset(RED);
  // Red/yellow pin: red half +5; yellow half on an unknown-toggle goal +10*(2*0.55-1) = +1
  float m = planner.placement_margin(RED_YELLOW_PIN, 2);
  CHECK(near(m, 6.0f, 0.01f), "red/yellow margin %.2f", m);
  CHECK(planner.placement_margin(BLUE_YELLOW_PIN, 2) < 0, "blue/yellow should be negative for red");
  CHECK(near(planner.placement_margin(RED_BLUE_PIN, 2), 0, 0.01f), "red/blue should net 0");
  CHECK(planner.placement_margin(RED_YELLOW_PIN, 3) < -100, "blue alliance goal must be off-limits to red");
  world.note_toggle(Q_RED2, BLUE);
  CHECK(planner.placement_margin(RED_YELLOW_PIN, 2) < 2.0f, "blue-owned quadrant should hurt yellow");
  world.reset(RED);

  // With a red/yellow pin nearby, planner should go get it.
  pose.seed_manual(-40, -40, 45);
  tick(5);
  AI_RECORD rec;
  memset(&rec, 0, sizeof(rec));
  rec.pos.framecnt = 99;
  rec.pos.x = -40 / 39.3701f; rec.pos.y = -40 / 39.3701f; rec.pos.az = 45;
  rec.detectionCount = 2;
  rec.detections[0].classID = RED_YELLOW_PIN; rec.detections[0].probability = 0.9f;
  rec.detections[0].mapLocation.x = -25 / 39.3701f; rec.detections[0].mapLocation.y = -25 / 39.3701f;
  rec.detections[1].classID = BLUE_YELLOW_PIN; rec.detections[1].probability = 0.9f;
  rec.detections[1].mapLocation.x = -35 / 39.3701f; rec.detections[1].mapLocation.y = -20 / 39.3701f;
  for (int i = 0; i < 3; i++) { world.ingest_jetson(rec, g_now_ms); tick(1); }
  RobotRole saved = planner.role;
  planner.role.flips_toggles = false;  // the 18" robot would rather flip toggles here
  Task t = planner.choose(90);
  CHECK(t.type == T_GET_OBJECT, "expected GET_OBJECT, got %s", task_name(t.type));
  const Track *tr = world.track(t.target);
  CHECK(tr && tr->cls == RED_YELLOW_PIN, "planner chose the wrong object");

  // Partner claims it: planner should not chase it.
  link_pkt::EventPayload claim = {1, link_pkt::EV_CLAIM, 0, 0, (int16_t)(-25 * 25.4f), (int16_t)(-25 * 25.4f)};
  world.apply_partner_event(claim);
  t = planner.choose(90);
  CHECK(t.type != T_GET_OBJECT || world.track(t.target)->cls != RED_YELLOW_PIN,
        "planner chased a partner-claimed object");

  planner.role = saved;

  // Toggle value grows late in the match, and a toggle bot goes for it.
  planner.role.flips_toggles = true;
  planner.role.picks_field_objects = false;
  t = planner.choose(30);
  CHECK(t.type == T_SET_TOGGLE, "toggle bot late in match: got %s", task_name(t.type));
  planner.role = saved;

  // Endgame: park.
  t = planner.choose(2.0f);
  CHECK(t.type == T_PARK, "expected PARK at 2 s left, got %s", task_name(t.type));
  CHECK(in_midfield(t.x, t.y), "park point not in midfield");

  // Holding a pin: only scoring tasks, never at the blue goal.
  mech::set_held(RED_YELLOW_PIN);
  t = planner.choose(60);
  CHECK(t.type == T_SCORE_HELD && t.target != 3, "holding: got %s target %d", task_name(t.type), t.target);
  mech::release_all();
  g_gps_q = 100;
}

static void test_deadline() {
  using namespace ai;
  set_deadline(g_now_ms);
  MoveResult r = go_to(10, 10);
  CHECK(r == MOVE_DEADLINE, "move should refuse to start past the deadline, got %s", move_result_name(r));
  set_deadline(0);
}

static void test_field_consistency() {
  using namespace ai::field;
  // Each quadrant has exactly one remaining goal, and goals lie in their quadrant.
  for (int g = 1; g < NUM_GOALS; g++) {
    CHECK(quadrant_of(GOALS[g].x, GOALS[g].y) == GOALS[g].quadrant, "goal %d in wrong quadrant", g);
  }
  CHECK(GOALS[1].color == RED && quadrant_color(GOALS[1].quadrant) == RED, "red goal not in red quadrant");
  CHECK(GOALS[3].color == BLUE && quadrant_color(GOALS[3].quadrant) == BLUE, "blue goal not in blue quadrant");
  for (int q = 0; q < NUM_QUADRANTS; q++) {
    CHECK(quadrant_of(TOGGLES[q].x * 0.95f, TOGGLES[q].y * 0.95f) == TOGGLES[q].quadrant, "toggle %d quadrant", q);
  }
  for (int l = 0; l < NUM_LOADERS; l++) {
    CHECK(side_of(LOADERS[l].x, LOADERS[l].y) == LOADERS[l].color, "loader %d on wrong side", l);
  }
  // AprilTags go counterclockwise from the red goal: angle should increase.
  float prev = -10.0f;
  for (int g = 1; g < NUM_GOALS; g++) {
    float a = atan2f(GOALS[g].y, GOALS[g].x);  // math angle, CCW positive
    if (a < atan2f(GOALS[1].y, GOALS[1].x) - 1e-3f) a += 2 * M_PI;
    CHECK(a > prev, "AprilTag %d not counterclockwise", g);
    prev = a;
  }
}

int main() {
  test_field_consistency();
  test_link_roundtrip();
  test_pose_seed_and_fusion();
  test_reanchoring();
  test_planner();
  test_deadline();
  printf("%d passed, %d failed\n", passes, fails);
  return fails ? 1 : 0;
}
