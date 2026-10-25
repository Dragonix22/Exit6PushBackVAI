#include "vex.h"

using namespace vex;

namespace ai {

PoseEstimator pose;

static float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

static float angle_diff(float a, float b) {  // a - b in [-180, 180)
  float d = a - b;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

PoseEstimator::PoseEstimator()
    : gain(0.04f), min_quality(90), max_jump_in(10.0f), persist_updates(25),
      max_speed_ips(30.0f), max_turn_dps(60.0f), correct_heading(true),
      seeded_(false), gps_good_(false), gps_quality_(0), gps_err_in_(0),
      outlier_count_(0), still_count_(0), speed_ips_(0), turn_dps_(0),
      last_ms_(0), hist_head_(0) {
  last_.x = last_.y = last_.heading = 0;
  for (int i = 0; i < HIST; i++) {
    hist_[i] = last_;
    hist_ms_[i] = 0;
  }
}

void PoseEstimator::init() {
  last_ = get();
  last_ms_ = now_ms();
}

Pose PoseEstimator::get() {
  Pose p;
  p.x = chassis.get_X_position();
  p.y = chassis.get_Y_position();
  p.heading = chassis.get_absolute_heading();
  return p;
}

bool PoseEstimator::read_gps(float &x, float &y, float &h) {
  gps_quality_ = GPS.quality();
  if (gps_quality_ < min_quality) return false;
  x = field::mm_to_in(GPS.xPosition(mm));
  y = field::mm_to_in(GPS.yPosition(mm));
  h = GPS.heading();
  // Reject readings that put us outside the field.
  const float lim = field::HALF_FIELD_IN + 4.0f;
  if (x < -lim || x > lim || y < -lim || y > lim) return false;
  return true;
}

bool PoseEstimator::seed_from_gps(uint32_t timeout_ms) {
  timer t;
  int n = 0;
  float sx = 0, sy = 0, ss = 0, sc = 0;
  while (t.time() < timeout_ms) {
    float x, y, h;
    if (read_gps(x, y, h)) {
      sx += x;
      sy += y;
      ss += sinf(h * (float)M_PI / 180.0f);  // average headings on the circle
      sc += cosf(h * (float)M_PI / 180.0f);
      if (++n >= 5) {
        float hd = wrap360(atan2f(ss, sc) * 180.0f / (float)M_PI);
        seed_manual(sx / n, sy / n, hd);
        return true;
      }
    }
    wait(20, msec);
  }
  return false;
}

void PoseEstimator::seed_manual(float x, float y, float heading) {
  chassis.set_coordinates(x, y, heading);
  seeded_ = true;
  last_ = get();
  last_ms_ = now_ms();
  outlier_count_ = 0;
}

void PoseEstimator::update() {
  uint32_t now = now_ms();
  Pose p = get();

  // Speed and turn rate from odom (smoothed).
  float dt = (now - last_ms_) / 1000.0f;
  if (dt > 0.001f) {
    float v = hypotf(p.x - last_.x, p.y - last_.y) / dt;
    float w = fabsf(angle_diff(p.heading, last_.heading)) / dt;
    speed_ips_ = 0.7f * speed_ips_ + 0.3f * v;
    turn_dps_ = 0.7f * turn_dps_ + 0.3f * w;
  }
  last_ = p;
  last_ms_ = now;

  hist_[hist_head_] = p;
  hist_ms_[hist_head_] = now;
  hist_head_ = (hist_head_ + 1) % HIST;

  float gx, gy, gh;
  gps_good_ = read_gps(gx, gy, gh);
  if (!gps_good_) {
    outlier_count_ = 0;
    return;
  }

  // Never seeded (e.g. GPS couldn't see the strip pre-match): seed now.
  if (!seeded_) {
    seed_manual(gx, gy, gh);
    log::event("POSE seeded late from GPS (%.1f, %.1f, %.0f)", gx, gy, gh);
    return;
  }

  float ex = gx - p.x, ey = gy - p.y;
  gps_err_in_ = hypotf(ex, ey);
  bool fast = speed_ips_ > max_speed_ips || turn_dps_ > max_turn_dps;

  if (gps_err_in_ > max_jump_in) {
    // Either a GPS glitch or odom got knocked off. If it persists, trust GPS.
    if (++outlier_count_ >= persist_updates && !fast) {
      log::event("POSE snap to GPS, error was %.1f in", gps_err_in_);
      chassis.correct_position(gx, gy);
      outlier_count_ = 0;
    }
    return;
  }
  outlier_count_ = 0;

  if (!fast) {
    chassis.correct_position(p.x + gain * ex, p.y + gain * ey);
  }

  // IMU heading drifts slowly; correct it from GPS only while sitting still.
  if (correct_heading) {
    if (speed_ips_ < 1.0f && turn_dps_ < 2.0f) still_count_++;
    else still_count_ = 0;
    if (still_count_ > 25) {  // ~0.5 s still
      float dh = angle_diff(gh, p.heading);
      if (fabsf(dh) > 1.5f && fabsf(dh) < 20.0f) {
        chassis.set_heading(wrap360(p.heading + 0.5f * dh));
      }
      still_count_ = 0;
    }
  }
}

Pose PoseEstimator::at_time(uint32_t t_ms) {
  // Newest entry at or before t_ms; oldest available if t_ms is too old.
  Pose best = last_;
  uint32_t best_ms = 0;
  bool found = false;
  for (int i = 0; i < HIST; i++) {
    if (hist_ms_[i] == 0) continue;
    if (hist_ms_[i] <= t_ms && (!found || hist_ms_[i] > best_ms)) {
      best = hist_[i];
      best_ms = hist_ms_[i];
      found = true;
    }
  }
  if (!found) {  // older than our history: use the oldest sample
    uint32_t oldest = 0xFFFFFFFF;
    for (int i = 0; i < HIST; i++) {
      if (hist_ms_[i] != 0 && hist_ms_[i] < oldest) {
        oldest = hist_ms_[i];
        best = hist_[i];
      }
    }
  }
  return best;
}

}  // namespace ai
