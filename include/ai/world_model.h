#pragma once
/*---------------------------------------------------------------------------*/
/*  World model: what this robot believes about the field right now.         */
/*                                                                           */
/*  It is a belief, not ground truth. Everything carries a confidence or a   */
/*  timestamp and fades when it isn't re-observed:                           */
/*   - object tracks from Jetson detections (and partner sightings)          */
/*   - toggle colors (from our actions, the partner's, or future vision)     */
/*   - goal contents, counted from scoring events (ours + partner's)         */
/*   - the partner robot's position (VEXlink)                                */
/*                                                                           */
/*  Updated only by the AI background task; read by the planner.             */
/*---------------------------------------------------------------------------*/

namespace ai {

struct Track {
  bool     active;
  uint8_t  id;            // stable while the track lives
  int      cls;           // field::ObjClass
  float    x, y, z;       // field inches; z = height of detection above tiles
  float    conf;          // 0..1, rises when seen, decays when not
  uint32_t last_seen_ms;
  uint16_t hits;
  bool     on_goal;       // elevated next to a goal: part of a stack, not pickable
  uint32_t partner_claim_ms; // when the partner claimed it (0 = not claimed)
};

struct ToggleBelief {
  field::Alliance color;  // RED, BLUE or NO_ALLIANCE (neutral/yellow)
  bool     known;
  uint32_t t_ms;          // when we last learned it
};

struct GoalBelief {
  int halves[3];          // visible pin halves by field::HalfColor (from scoring events)
  int cups;
};

struct PartnerInfo {
  bool  valid;
  float x, y, heading;
  int   task;
  bool  holding, parking;
};

class WorldModel {
 public:
  static const int MAX_TRACKS = 40;

  WorldModel();
  void reset(field::Alliance us);

  // ---- inputs (AI background task only) ----
  void ingest_jetson(const AI_RECORD &rec, uint32_t rx_ms);
  void ingest_partner();         // reads TeamLink
  void decay(uint32_t now_ms);

  // ---- things our own robot did ----
  void note_picked(int track_id);
  void note_scored(int goal, int cls);
  void note_toggle(int quadrant, field::Alliance color);
  void note_claim(int track_id);  // tells the partner too
  void note_release(int track_id);
  void note_load_taken();

  // What we tell the partner about the field (toggles + best sightings).
  void make_field_payload(link_pkt::FieldPayload &f) const;

  // ---- queries ----
  const Track *track(int id) const;
  // cls = NO_CLASS matches any class. Skips stacked and partner-claimed objects.
  const Track *nearest_pickable(int cls, float x, float y, float min_conf = 0.4f) const;
  bool partner_claimed(const Track &t) const;
  int  count_pickable(float min_conf = 0.4f) const;
  field::Alliance predicted_owner(int quadrant) const;  // toggle color, or NO_ALLIANCE if unknown
  int  yellow_halves_in_quadrant(int quadrant) const;
  void estimate_score(int &ours, int &theirs) const;

  // ---- state ----
  field::Alliance us;
  Track        tracks[MAX_TRACKS];
  ToggleBelief toggles[field::NUM_QUADRANTS];
  GoalBelief   goals[field::NUM_GOALS];
  PartnerInfo  partner;
  int          our_midfield_robots_planned;  // used for midfield yellow ownership guess
  int          loads_taken_total;            // ours + partner's match loads used

  // ---- tuning ----
  float gate_in;            // association distance for "same object"
  float decay_tau_s;        // confidence half-life-ish when not observed
  float miss_penalty;       // confidence lost when it should have been seen but wasn't
  float camera_latency_ms;  // image capture -> packet arrival
  float max_turn_for_vision_dps;  // ignore frames while spinning fast
  float min_prob;           // ignore weaker detections
  float max_range_in;       // ignore far detections (depth gets noisy)
  // Camera position on the robot (inches, from the turning center). Only used
  // when the Jetson's GPS pose is stale. Keep in sync with the web dashboard.
  float cam_fwd_in, cam_right_in;

 private:
  int  add_or_update(int cls, float x, float y, float z, float conf_gain, uint32_t now);
  void penalize_unseen(const Pose &p, const bool *seen, uint32_t now);
  int  find_near(int cls, float x, float y, float gate) const;
  void apply_partner_event(const link_pkt::EventPayload &e);

  uint8_t  next_id_;
  int32_t  last_framecnt_;
  uint32_t last_framecnt_change_ms_;
  uint32_t last_decay_ms_;
  int      my_loads_taken_;
};

extern WorldModel world;

}  // namespace ai
