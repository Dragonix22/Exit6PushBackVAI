#include "vex.h"

using namespace vex;

namespace ai {

using namespace field;

WorldModel world;

static const float DEG = (float)M_PI / 180.0f;
static const uint32_t CLAIM_TIMEOUT_MS = 6000;

// RealSense D435 color stream at 640x480: ~69 deg horizontal FOV.
static const float CAM_FX_PX = 462.0f;
static const float CAM_CX_PX = 320.0f;
static const float CAM_HALF_FOV_DEG = 30.0f;  // a bit under 34.5 to stay conservative

WorldModel::WorldModel()
    : us(RED), our_midfield_robots_planned(0), loads_taken_total(0),
      gate_in(7.0f), decay_tau_s(10.0f), miss_penalty(0.08f),
      camera_latency_ms(80.0f), max_turn_for_vision_dps(90.0f), min_prob(0.45f),
      max_range_in(100.0f), cam_fwd_in(6.0f), cam_right_in(0.0f), next_id_(1),
      last_framecnt_(-1), last_framecnt_change_ms_(0), last_decay_ms_(0),
      my_loads_taken_(0) {
  reset(RED);
}

void WorldModel::reset(Alliance alliance) {
  us = alliance;
  for (int i = 0; i < MAX_TRACKS; i++) {
    memset(&tracks[i], 0, sizeof(Track));
    tracks[i].cls = NO_CLASS;
  }
  for (int q = 0; q < NUM_QUADRANTS; q++) {
    // TODO: Appendix A shows the toggles' starting orientation; set it here
    // (known = true) once you've confirmed it on a real field.
    toggles[q].color = NO_ALLIANCE;
    toggles[q].known = false;
    toggles[q].t_ms = 0;
  }
  for (int g = 0; g < NUM_GOALS; g++) {
    memset(&goals[g], 0, sizeof(GoalBelief));
  }
  // Section 7: each neutral goal starts with a yellow/yellow pin Placed on it.
  for (int g = 0; g < NUM_GOALS; g++) {
    if (GOALS[g].color == NO_ALLIANCE) goals[g].halves[HALF_YELLOW] = 2;
  }
  memset(&partner, 0, sizeof(partner));
  our_midfield_robots_planned = 0;
  loads_taken_total = 0;
  my_loads_taken_ = 0;
}

/*-------------------------------- tracks -----------------------------------*/
int WorldModel::find_near(int cls, float x, float y, float gate) const {
  int best = -1;
  float best_d = gate;
  for (int i = 0; i < MAX_TRACKS; i++) {
    const Track &t = tracks[i];
    if (!t.active) continue;
    if (cls != NO_CLASS && t.cls != cls) continue;
    float d = hypotf(t.x - x, t.y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

int WorldModel::add_or_update(int cls, float x, float y, float z, float conf_gain,
                              uint32_t now) {
  int i = find_near(cls, x, y, gate_in);
  if (i >= 0) {
    Track &t = tracks[i];
    const float a = 0.4f;  // smoothing: new measurement weight
    t.x += a * (x - t.x);
    t.y += a * (y - t.y);
    t.z += a * (z - t.z);
    t.conf = fminf(1.0f, t.conf + conf_gain);
    t.last_seen_ms = now;
    if (t.hits < 65535) t.hits++;
  } else {
    // New track: use a free slot, or replace the least confident one.
    i = 0;
    float worst = 2.0f;
    for (int k = 0; k < MAX_TRACKS; k++) {
      if (!tracks[k].active) { i = k; worst = -1.0f; break; }
      if (tracks[k].conf < worst) { worst = tracks[k].conf; i = k; }
    }
    Track &t = tracks[i];
    memset(&t, 0, sizeof(t));
    t.active = true;
    // Pick an id that no active track is using.
    for (int tries = 0; tries < 256; tries++) {
      uint8_t cand = next_id_++;
      if (cand == 0 || cand == 0xFF) continue;
      bool used = false;
      for (int k = 0; k < MAX_TRACKS; k++) {
        if (tracks[k].active && k != i && tracks[k].id == cand) used = true;
      }
      if (!used) { t.id = cand; break; }
    }
    t.cls = cls;
    t.x = x;
    t.y = y;
    t.z = z;
    t.conf = 0.25f + conf_gain;
    t.last_seen_ms = now;
    t.hits = 1;
  }

  // Elevated and next to a goal: it's sitting on a stack, don't try to grab it.
  Track &t = tracks[i];
  t.on_goal = false;
  for (int g = 0; g < NUM_GOALS; g++) {
    if (hypotf(t.x - GOALS[g].x, t.y - GOALS[g].y) < 7.0f && t.z > 4.5f) t.on_goal = true;
  }
  return i;
}

void WorldModel::penalize_unseen(const Pose &p, const bool *seen, uint32_t now) {
  // A track that should be in front of the camera but wasn't detected
  // probably got moved (or picked up by someone else).
  for (int i = 0; i < MAX_TRACKS; i++) {
    Track &t = tracks[i];
    if (!t.active || seen[i] || t.on_goal) continue;
    float dx = t.x - p.x, dy = t.y - p.y;
    float range = hypotf(dx, dy);
    if (range < 15.0f || range > 70.0f) continue;
    float bearing = atan2f(dx, dy) / DEG - p.heading;
    while (bearing > 180.0f) bearing -= 360.0f;
    while (bearing < -180.0f) bearing += 360.0f;
    if (fabsf(bearing) < CAM_HALF_FOV_DEG) {
      t.conf -= miss_penalty;
      if (t.conf < 0.1f) t.active = false;
    }
  }
  (void)now;
}

void WorldModel::ingest_jetson(const AI_RECORD &rec, uint32_t rx_ms) {
  // The Jetson only updates its pose when its GPS has a valid fix; when the
  // frame counter stops changing its pose is stale.
  if (rec.pos.framecnt != last_framecnt_) {
    last_framecnt_ = rec.pos.framecnt;
    last_framecnt_change_ms_ = rx_ms;
  }
  bool jetson_pose_fresh = rec.pos.framecnt > 0 && (rx_ms - last_framecnt_change_ms_) < 300;

  // Spinning fast: latency turns into big position errors. Skip the frame.
  if (pose.turn_rate_dps() > max_turn_for_vision_dps) return;

  // Our pose when the image was taken, not when the packet arrived.
  Pose p = pose.at_time(rx_ms - (uint32_t)camera_latency_ms);
  float sh = sinf(p.heading * DEG), ch = cosf(p.heading * DEG);

  float jx = m_to_in(rec.pos.x), jy = m_to_in(rec.pos.y);
  float jsh = sinf(rec.pos.az * DEG), jch = cosf(rec.pos.az * DEG);

  bool seen[MAX_TRACKS];
  memset(seen, 0, sizeof(seen));

  int n = rec.detectionCount;
  if (n > MAX_DETECTIONS) n = MAX_DETECTIONS;
  for (int i = 0; i < n; i++) {
    const DETECTION_OBJECT &d = rec.detections[i];
    if (d.classID < 0 || d.classID >= NUM_CLASSES) continue;
    if (d.probability < min_prob) continue;

    // Detection relative to the robot: forward / right, inches.
    float fwd, right, z;
    if (jetson_pose_fresh) {
      // The Jetson placed the object using ITS pose. Undo that to get the
      // camera-relative vector, then re-apply OUR fused pose below. This
      // keeps detections consistent with odom even when the Jetson's
      // (smoothed, lagging) GPS pose disagrees with ours.
      float dx = m_to_in(d.mapLocation.x) - jx;
      float dy = m_to_in(d.mapLocation.y) - jy;
      fwd = dx * jsh + dy * jch;
      right = dx * jch - dy * jsh;
      z = m_to_in(d.mapLocation.z);
    } else {
      // No usable Jetson pose: use depth + image column directly.
      if (d.depth <= 0.0f) continue;
      float depth_in = m_to_in(d.depth);
      fwd = depth_in + cam_fwd_in;
      right = depth_in * ((float)d.screenLocation.x - CAM_CX_PX) / CAM_FX_PX + cam_right_in;
      z = 0.0f;  // unknown
    }

    if (hypotf(fwd, right) > max_range_in) continue;

    float x = p.x + fwd * sh + right * ch;
    float y = p.y + fwd * ch - right * sh;
    const float lim = HALF_FIELD_IN + 2.0f;
    if (x < -lim || x > lim || y < -lim || y > lim) continue;

    int k = add_or_update(d.classID, x, y, z, 0.25f * d.probability, rx_ms);
    if (k >= 0) seen[k] = true;
  }

  penalize_unseen(p, seen, rx_ms);
}

void WorldModel::decay(uint32_t now) {
  if (last_decay_ms_ == 0) last_decay_ms_ = now;
  float dt = (now - last_decay_ms_) / 1000.0f;
  last_decay_ms_ = now;
  if (dt <= 0.0f) return;
  float k = expf(-dt / decay_tau_s);
  for (int i = 0; i < MAX_TRACKS; i++) {
    Track &t = tracks[i];
    if (!t.active) continue;
    t.conf *= k;
    if (t.conf < 0.1f) t.active = false;
    if (t.partner_claim_ms && now - t.partner_claim_ms > CLAIM_TIMEOUT_MS) t.partner_claim_ms = 0;
  }
}

/*-------------------------------- partner ----------------------------------*/
void WorldModel::apply_partner_event(const link_pkt::EventPayload &e) {
  uint32_t now = now_ms();
  float x = mm_to_in(e.x_mm), y = mm_to_in(e.y_mm);
  switch (e.kind) {
    case link_pkt::EV_CLAIM: {
      int i = find_near(NO_CLASS, x, y, 8.0f);
      if (i >= 0) tracks[i].partner_claim_ms = now;
      break;
    }
    case link_pkt::EV_RELEASE: {
      int i = find_near(NO_CLASS, x, y, 8.0f);
      if (i >= 0) tracks[i].partner_claim_ms = 0;
      break;
    }
    case link_pkt::EV_PICKED: {
      int i = find_near(NO_CLASS, x, y, 8.0f);
      if (i >= 0) tracks[i].active = false;
      break;
    }
    case link_pkt::EV_SCORED:
      if (e.a < NUM_GOALS) {
        if (e.b == CUP) {
          goals[e.a].cups++;
        } else if (is_pin(e.b)) {
          HalfColor h1, h2;
          pin_halves(e.b, h1, h2);
          goals[e.a].halves[h1]++;
          goals[e.a].halves[h2]++;
        }
      }
      break;
    case link_pkt::EV_TOGGLE:
      if (e.a < NUM_QUADRANTS && e.b <= NO_ALLIANCE) {
        toggles[e.a].color = (Alliance)e.b;
        toggles[e.a].known = true;
        toggles[e.a].t_ms = now;
      }
      break;
  }
}

void WorldModel::ingest_partner() {
  link_pkt::StatePayload st;
  partner.valid = team_link.partner_state(st);
  if (partner.valid) {
    partner.x = mm_to_in(st.x_mm);
    partner.y = mm_to_in(st.y_mm);
    partner.heading = st.heading_cdeg / 100.0f;
    partner.task = st.task;
    partner.holding = (st.flags & link_pkt::F_HOLDING) != 0;
    partner.parking = (st.flags & link_pkt::F_PARKING) != 0;
  }

  link_pkt::FieldPayload f;
  if (team_link.partner_field(f)) {
    uint32_t now = now_ms();
    loads_taken_total = my_loads_taken_ + f.loads_taken;
    for (int q = 0; q < NUM_QUADRANTS; q++) {
      int bits = (f.toggle_bits >> (2 * q)) & 0x3;
      if (bits != 0 && !toggles[q].known) {  // only fill in what we don't know
        toggles[q].color = bits == 1 ? RED : (bits == 2 ? BLUE : NO_ALLIANCE);
        toggles[q].known = true;
        toggles[q].t_ms = now;
      }
    }
    for (int i = 0; i < f.count; i++) {
      // Partner sightings count for less than our own.
      add_or_update(f.s[i].cls, mm_to_in(f.s[i].x_mm), mm_to_in(f.s[i].y_mm), 0.0f,
                    0.08f, now);
    }
  }

  link_pkt::EventPayload e;
  while (team_link.pop_event(e)) apply_partner_event(e);
}

void WorldModel::make_field_payload(link_pkt::FieldPayload &f) const {
  memset(&f, 0, sizeof(f));
  for (int q = 0; q < NUM_QUADRANTS; q++) {
    int bits = 0;
    if (toggles[q].known) {
      bits = toggles[q].color == RED ? 1 : (toggles[q].color == BLUE ? 2 : 3);
    }
    f.toggle_bits |= (uint8_t)(bits << (2 * q));
  }
  f.loads_taken = (uint8_t)my_loads_taken_;

  // Share the most confident, recently seen, pickable objects.
  uint32_t now = now_ms();
  bool used[MAX_TRACKS];
  memset(used, 0, sizeof(used));
  while (f.count < link_pkt::MAX_SIGHTINGS) {
    int best = -1;
    for (int i = 0; i < MAX_TRACKS; i++) {
      const Track &t = tracks[i];
      if (!t.active || used[i] || t.on_goal || now - t.last_seen_ms > 1000) continue;
      if (best < 0 || t.conf > tracks[best].conf) best = i;
    }
    if (best < 0) break;
    used[best] = true;
    link_pkt::Sighting &s = f.s[f.count++];
    s.cls = (uint8_t)tracks[best].cls;
    s.x_mm = (int16_t)in_to_mm(tracks[best].x);
    s.y_mm = (int16_t)in_to_mm(tracks[best].y);
  }
}

/*------------------------------ our actions --------------------------------*/
void WorldModel::note_picked(int track_id) {
  for (int i = 0; i < MAX_TRACKS; i++) {
    if (tracks[i].active && tracks[i].id == track_id) {
      team_link.send_event(link_pkt::EV_PICKED, 0, 0, tracks[i].x, tracks[i].y);
      tracks[i].active = false;
    }
  }
}

void WorldModel::note_scored(int goal, int cls) {
  if (goal < 0 || goal >= NUM_GOALS) return;
  if (cls == CUP) {
    goals[goal].cups++;
  } else if (is_pin(cls)) {
    HalfColor h1, h2;
    pin_halves(cls, h1, h2);
    goals[goal].halves[h1]++;
    goals[goal].halves[h2]++;
  }
  team_link.send_event(link_pkt::EV_SCORED, (uint8_t)goal, (uint8_t)cls,
                       GOALS[goal].x, GOALS[goal].y);
}

void WorldModel::note_toggle(int q, Alliance color) {
  if (q < 0 || q >= NUM_QUADRANTS) return;
  toggles[q].color = color;
  toggles[q].known = true;
  toggles[q].t_ms = now_ms();
  team_link.send_event(link_pkt::EV_TOGGLE, (uint8_t)q, (uint8_t)color,
                       TOGGLES[q].x, TOGGLES[q].y);
}

void WorldModel::note_claim(int track_id) {
  const Track *t = track(track_id);
  if (t) team_link.send_event(link_pkt::EV_CLAIM, 0, 0, t->x, t->y);
}

void WorldModel::note_release(int track_id) {
  const Track *t = track(track_id);
  if (t) team_link.send_event(link_pkt::EV_RELEASE, 0, 0, t->x, t->y);
}

void WorldModel::note_load_taken() {
  my_loads_taken_++;
  loads_taken_total++;
}

/*-------------------------------- queries ----------------------------------*/
const Track *WorldModel::track(int id) const {
  for (int i = 0; i < MAX_TRACKS; i++) {
    if (tracks[i].active && tracks[i].id == id) return &tracks[i];
  }
  return 0;
}

bool WorldModel::partner_claimed(const Track &t) const {
  return t.partner_claim_ms != 0 && now_ms() - t.partner_claim_ms < CLAIM_TIMEOUT_MS;
}

const Track *WorldModel::nearest_pickable(int cls, float x, float y, float min_conf) const {
  const Track *best = 0;
  float best_d = 1e9f;
  for (int i = 0; i < MAX_TRACKS; i++) {
    const Track &t = tracks[i];
    if (!t.active || t.on_goal || t.conf < min_conf) continue;
    if (cls != NO_CLASS && t.cls != cls) continue;
    if (partner_claimed(t)) continue;
    float d = hypotf(t.x - x, t.y - y);
    if (d < best_d) {
      best_d = d;
      best = &t;
    }
  }
  return best;
}

int WorldModel::count_pickable(float min_conf) const {
  int n = 0;
  for (int i = 0; i < MAX_TRACKS; i++) {
    const Track &t = tracks[i];
    if (t.active && !t.on_goal && t.conf >= min_conf) n++;
  }
  return n;
}

Alliance WorldModel::predicted_owner(int q) const {
  if (q < 0 || q >= NUM_QUADRANTS || !toggles[q].known) return NO_ALLIANCE;
  return toggles[q].color;
}

int WorldModel::yellow_halves_in_quadrant(int q) const {
  int n = 0;
  for (int g = 0; g < NUM_GOALS; g++) {
    if (GOALS[g].quadrant == q) n += goals[g].halves[HALF_YELLOW];
  }
  return n;
}

void WorldModel::estimate_score(int &ours, int &theirs) const {
  int pts[2] = {0, 0};  // [RED, BLUE]
  for (int g = 0; g < NUM_GOALS; g++) {
    pts[RED] += goals[g].halves[HALF_RED] * PTS_ALLIANCE_HALF;
    pts[BLUE] += goals[g].halves[HALF_BLUE] * PTS_ALLIANCE_HALF;
    Alliance owner = NO_ALLIANCE;
    if (g == MIDFIELD_GOAL) {
      // Owned by whoever ends with more robots in the Midfield. We only know
      // our plan; assume we win it if we plan to park at least one robot.
      if (our_midfield_robots_planned > 0) owner = us;
    } else {
      owner = predicted_owner(GOALS[g].quadrant);
    }
    if (owner == RED || owner == BLUE) {
      pts[owner] += goals[g].halves[HALF_YELLOW] * PTS_OWNED_YELLOW;
    }
  }
  pts[us] += our_midfield_robots_planned * PTS_MIDFIELD_ROBOT;
  ours = pts[us];
  theirs = pts[opponent(us)];
}

}  // namespace ai
