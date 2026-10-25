#pragma once
/*---------------------------------------------------------------------------*/
/*  Pose estimation: JAR odometry corrected by the GPS sensor.               */
/*                                                                           */
/*  JAR's odom stays the "real" pose (so chassis.drive_to_point keeps        */
/*  working); this class nudges it toward the GPS when the GPS is            */
/*  trustworthy. It is a complementary filter, not a Kalman filter: simple   */
/*  to tune and hard to break.                                               */
/*                                                                           */
/*  - Small disagreements are removed gradually (gain per update).           */
/*  - Big jumps are ignored as GPS glitches, unless they persist (then odom  */
/*    was knocked off by a collision and we snap to GPS).                    */
/*  - GPS is ignored while moving fast or turning, because it lags.          */
/*  - Heading is corrected only while sitting still.                         */
/*---------------------------------------------------------------------------*/

namespace ai {

struct Pose {
  float x, y;      // field inches
  float heading;   // degrees, clockwise from +Y, 0..360
};

class PoseEstimator {
 public:
  PoseEstimator();

  // Call once in pre_auton, after the IMU and GPS have calibrated.
  void init();

  // Wait (up to timeout_ms) for a good GPS fix, then put JAR in the field
  // frame. Returns false if the GPS never gave a good reading.
  bool seed_from_gps(uint32_t timeout_ms);

  // Fallback when the GPS can't see the field strip at the start.
  void seed_manual(float x, float y, float heading);

  bool seeded() const { return seeded_; }

  // Call at ~50 Hz from the AI background task.
  void update();

  Pose get();                         // current fused pose
  Pose at_time(uint32_t t_ms);        // pose at an earlier time (for camera latency)
  float speed_ips() const { return speed_ips_; }
  float turn_rate_dps() const { return turn_dps_; }

  bool  gps_good() const { return gps_good_; }
  int   gps_quality() const { return gps_quality_; }
  float gps_error_in() const { return gps_err_in_; }  // last |GPS - odom|

  // ---- tuning ----
  float gain;             // fraction of GPS-odom error removed per update
  int   min_quality;      // GPS quality (0-100) needed to use a reading
  float max_jump_in;      // bigger disagreement = probably a glitch...
  int   persist_updates;  // ...unless it lasts this many updates; then snap
  float max_speed_ips;    // ignore GPS above this speed
  float max_turn_dps;     // ignore GPS above this turn rate
  bool  correct_heading;  // fix IMU drift from GPS while stationary

 private:
  bool  read_gps(float &x, float &y, float &h);

  bool     seeded_;
  bool     gps_good_;
  int      gps_quality_;
  float    gps_err_in_;
  int      outlier_count_;
  int      still_count_;
  float    speed_ips_, turn_dps_;
  Pose     last_;
  uint32_t last_ms_;

  static const int HIST = 40;   // 40 x 20 ms = 0.8 s of history
  Pose     hist_[HIST];
  uint32_t hist_ms_[HIST];
  int      hist_head_;
};

extern PoseEstimator pose;

}  // namespace ai
