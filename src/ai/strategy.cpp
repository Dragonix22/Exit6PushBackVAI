#include "vex.h"

using namespace vex;

namespace ai {

using namespace field;

Planner planner;

const char *task_name(int t) {
  switch (t) {
    case T_NONE:       return "NONE";
    case T_GET_OBJECT: return "GET_OBJECT";
    case T_SCORE_HELD: return "SCORE_HELD";
    case T_LOAD:       return "LOAD";
    case T_SET_TOGGLE: return "SET_TOGGLE";
    case T_PARK:       return "PARK";
    case T_SEARCH:     return "SEARCH";
  }
  return "?";
}

Planner::Planner()
    : drive_speed_ips(30.0f), pickup_s(1.2f), place_s(1.5f), load_wait_s(2.5f),
      toggle_s(1.5f), park_buffer_s(2.0f), p_win_unknown_toggle(0.55f),
      p_keep_our_toggle(0.8f), p_win_midfield(0.6f) {
#if ROBOT_ID == ROBOT_BIG
  role.picks_field_objects = true;
  role.uses_loaders = true;
  role.flips_toggles = false;
  role.parks = true;
#else
  role.picks_field_objects = true;
  role.uses_loaders = false;
  role.flips_toggles = true;
  role.parks = true;
#endif
}

float Planner::travel_s(float x, float y) const {
  Pose p = pose.get();
  return hypotf(x - p.x, y - p.y) / drive_speed_ips + 0.6f;  // + turn overhead
}

float Planner::p_we_own_quadrant(int q) const {
  const ToggleBelief &t = world.toggles[q];
  if (!t.known || t.color == NO_ALLIANCE) return p_win_unknown_toggle;
  return t.color == world.us ? p_keep_our_toggle : 0.3f;
}

float Planner::placement_margin(int cls, int g) const {
  if (g < 0 || g >= NUM_GOALS || !goal_usable_by(g, world.us)) return -1000.0f;
  if (!is_pin(cls)) return 0.0f;  // Cups score nothing by themselves. TODO: model covering halves
  HalfColor h[2];
  pin_halves(cls, h[0], h[1]);
  float m = 0.0f;
  for (int i = 0; i < 2; i++) {
    if (h[i] == HALF_RED) {
      m += world.us == RED ? PTS_ALLIANCE_HALF : -PTS_ALLIANCE_HALF;
    } else if (h[i] == HALF_BLUE) {
      m += world.us == BLUE ? PTS_ALLIANCE_HALF : -PTS_ALLIANCE_HALF;
    } else {
      float p = (g == MIDFIELD_GOAL) ? p_win_midfield : p_we_own_quadrant(GOALS[g].quadrant);
      m += PTS_OWNED_YELLOW * (2.0f * p - 1.0f);  // we get it with p, they with (1-p)
    }
  }
  return m;
}

int Planner::best_goal_for(int cls, float &margin) const {
  int best = -1;
  margin = -1e9f;
  for (int g = 0; g < NUM_GOALS; g++) {
    if (!goal_usable_by(g, world.us)) continue;
    float m = placement_margin(cls, g);
    if (m > margin) {
      margin = m;
      best = g;
    }
  }
  return best;
}

void Planner::park_point(float &x, float &y) const {
  // Four spots inside the Midfield, clear of the center goal:
  //   ours   = corner on our side, theirs = corner on their side,
  //   diag1/diag2 = the two corners on the dividing diagonal.
  // Each robot has its own preference order so they don't pick the same spot,
  // and skips a spot the partner is already parking at.
  const float k = MIDFIELD_HALF_IN - 10.0f;
  const float s = world.us == RED ? -1.0f : 1.0f;
  float spots[4][2] = {{s * k, s * k}, {-k, k}, {k, -k}, {-s * k, -s * k}};
#if ROBOT_ID == ROBOT_BIG
  int order[4] = {0, 1, 2, 3};
#else
  // Small robot: nearer diagonal corner first.
  Pose p = pose.get();
  bool d1_nearer = hypotf(spots[1][0] - p.x, spots[1][1] - p.y) <
                   hypotf(spots[2][0] - p.x, spots[2][1] - p.y);
  int order[4] = {d1_nearer ? 1 : 2, d1_nearer ? 2 : 1, 0, 3};
#endif
  for (int i = 0; i < 4; i++) {
    float cx = spots[order[i]][0], cy = spots[order[i]][1];
    bool partner_there = world.partner.valid && world.partner.parking &&
                         hypotf(cx - world.partner.x, cy - world.partner.y) < 20.0f;
    if (!partner_there) {
      x = cx;
      y = cy;
      return;
    }
  }
  x = spots[order[0]][0];
  y = spots[order[0]][1];
}

float Planner::park_time_s() const {
  float x, y;
  park_point(x, y);
  return travel_s(x, y) + park_buffer_s;
}

Task Planner::choose(float time_left_s) {
  Task best;
  memset(&best, 0, sizeof(best));
  best.type = T_NONE;
  best.utility = -1e9f;

  // Endgame: go park while there's still time to get there.
  float park_t = role.parks ? park_time_s() : 0.0f;
  if (role.parks && time_left_s <= park_t) {
    best.type = T_PARK;
    park_point(best.x, best.y);
    best.value = PTS_MIDFIELD_ROBOT;
    best.time_s = park_t;
    best.utility = 1e9f;
    return best;
  }
  const float avail = time_left_s - park_t;  // time before we must leave to park

  // C++11 lambda: keep the best candidate that fits in the time we have.
  auto consider = [&](Task c) {
    if (c.time_s > avail) return;
    c.utility = c.value / (c.time_s > 0.5f ? c.time_s : 0.5f);
    if (c.utility > best.utility) best = c;
  };

  if (mech::holding()) {
    // Holding something: only placing it makes sense.
    int cls = mech::held_class();
    for (int g = 0; g < NUM_GOALS; g++) {
      if (!goal_usable_by(g, world.us)) continue;
      Task c;
      memset(&c, 0, sizeof(c));
      c.type = T_SCORE_HELD;
      c.target = g;
      c.x = GOALS[g].x;
      c.y = GOALS[g].y;
      c.value = placement_margin(cls, g);
      c.time_s = travel_s(c.x, c.y) + place_s;
      consider(c);
    }
    return best;
  }

  // Pick up a field object, then score it at its best goal.
  if (role.picks_field_objects) {
    for (int i = 0; i < WorldModel::MAX_TRACKS; i++) {
      const Track &t = world.tracks[i];
      if (!t.active || t.on_goal || t.conf < 0.4f || world.partner_claimed(t)) continue;
      float gm;
      int g = best_goal_for(t.cls, gm);
      if (g < 0 || gm <= 0.0f) continue;
      Task c;
      memset(&c, 0, sizeof(c));
      c.type = T_GET_OBJECT;
      c.target = t.id;
      c.x = t.x;
      c.y = t.y;
      c.value = gm * t.conf;
      c.time_s = travel_s(t.x, t.y) + pickup_s +
                 hypotf(GOALS[g].x - t.x, GOALS[g].y - t.y) / drive_speed_ips + place_s;
      consider(c);
    }
  }

  // Match loads (Interaction Period only; humans drop them into our loaders).
  if (role.uses_loaders && period() == INTERACTION &&
      world.loads_taken_total < MATCH_LOAD_PINS + MATCH_LOAD_CUPS) {
    float gm;
    int g = best_goal_for(match_load_pin(world.us), gm);
    for (int l = 0; l < NUM_LOADERS && g >= 0; l++) {
      if (LOADERS[l].color != world.us) continue;
      Task c;
      memset(&c, 0, sizeof(c));
      c.type = T_LOAD;
      c.target = l;
      c.x = LOADERS[l].x;
      c.y = LOADERS[l].y;
      c.value = gm * 0.9f;  // we can't control what the human loads
      c.time_s = travel_s(c.x, c.y) + load_wait_s +
                 hypotf(GOALS[g].x - c.x, GOALS[g].y - c.y) / drive_speed_ips + place_s;
      consider(c);
    }
  }

  // Toggles: worth more later in the match (less time for them to flip back).
  if (role.flips_toggles) {
    float keep = 1.0f - (time_left_s > 105.0f ? 105.0f : time_left_s) / 150.0f;
    for (int q = 0; q < NUM_QUADRANTS; q++) {
      const ToggleBelief &tb = world.toggles[q];
      if (tb.known && tb.color == world.us) continue;
      if (world.partner.valid && world.partner.task == T_SET_TOGGLE &&
          hypotf(world.partner.x - TOGGLES[q].x, world.partner.y - TOGGLES[q].y) < 30.0f)
        continue;  // partner is on it
      int n = world.yellow_halves_in_quadrant(q);
      bool theirs = tb.known && tb.color == opponent(world.us);
      Task c;
      memset(&c, 0, sizeof(c));
      c.type = T_SET_TOGGLE;
      c.target = q;
      c.x = TOGGLES[q].x;
      c.y = TOGGLES[q].y;
      c.value = PTS_OWNED_YELLOW * n * (theirs ? 2.0f : 1.0f) * keep;
      c.time_s = travel_s(c.x, c.y) + toggle_s;
      consider(c);
    }
  }

  // Nothing worth doing: look around for objects.
  if (best.type == T_NONE || best.utility < 0.2f) {
    Task c;
    memset(&c, 0, sizeof(c));
    c.type = T_SEARCH;
    // Vantage point on our half, looking at the field.
    c.x = world.us == RED ? -30.0f : 30.0f;
    c.y = world.us == RED ? -30.0f : 30.0f;
    c.value = 0.5f;
    c.time_s = travel_s(c.x, c.y) + 2.5f;
    c.utility = c.value / c.time_s;
    if (c.time_s <= avail || best.type == T_NONE) best = c;
  }
  return best;
}

}  // namespace ai
