// Host-side FAKE of the vex:: API so the real JAR + AI code can run in unit
// tests. Devices read from global variables the test sets. Threads never run.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>

namespace vex {
enum directionType { fwd, reverse, undefined_dir };
enum voltageUnits { volt, voltageUnits_mV };
enum rotationUnits { deg, rev, raw };
enum velocityUnits { pct, rpm, dps };
enum percentUnits { percent };
enum brakeType { coast, brake, hold };
enum distanceUnits { mm, inches, cm };
enum turnType { left, right };
enum gearSetting { ratio36_1, ratio18_1, ratio6_1 };
enum timeUnits { sec, msec };
enum fontType { mono12, mono15, mono20, mono30, prop20 };
enum controllerType { primary, partner };
enum class linkType { manager, worker, raw };
enum currentUnits { amp };
const int32_t PORT1 = 0, PORT2 = 1, PORT3 = 2, PORT4 = 3, PORT5 = 4, PORT6 = 5,
  PORT7 = 6, PORT8 = 7, PORT9 = 8, PORT10 = 9, PORT11 = 10, PORT12 = 11,
  PORT13 = 12, PORT14 = 13, PORT15 = 14, PORT16 = 15, PORT17 = 16, PORT18 = 17,
  PORT19 = 18, PORT20 = 19, PORT21 = 20, PORT22 = 21;

class color { public: color() {} color(int) {} color(int, int, int) {} };
static const color black, white, red, green, blue, yellow, orange, purple, cyan, transparent;

// ---- fake world, set by tests ----
extern uint32_t g_now_ms;
extern double g_motor_deg[22];
extern double g_motor_vel[22];
extern double g_gyro_rot;
extern double g_gps_x_mm, g_gps_y_mm, g_gps_h;
extern int g_gps_q;
extern std::vector<uint8_t> g_link_tx;

inline void wait(double t, timeUnits u) { g_now_ms += (uint32_t)(u == sec ? t * 1000 : t); }

class timer {
  uint32_t start_;
 public:
  timer() : start_(g_now_ms) {}
  uint32_t time() { return g_now_ms - start_; }
  double time(timeUnits u) { return u == sec ? time() / 1000.0 : time(); }
  void clear() { start_ = g_now_ms; }
  static uint32_t system() { return g_now_ms; }
};
class mutex { public: void lock() {} void unlock() {} bool try_lock() { return true; } };
class thread {
 public:
  static const int32_t threadPriorityHigh = 15;
  thread() {}
  thread(int (*)(void)) {}
  thread(void (*)(void)) {}
  thread(int (*)(void *), void *) {}
  void setPriority(int32_t) {}
  void interrupt() {}
};
namespace this_thread { inline void sleep_for(uint32_t t) { g_now_ms += t; } inline void yield() {} }
class task { public: task() {} task(int (*)(void)) {} void stop() {} static void sleep(uint32_t t) { g_now_ms += t; } };
class device { public: device() {} device(int32_t) {} bool installed() { return true; } };
class triport { public: class port { public: port() {} }; triport(int32_t) {} port Port[8]; };

class motor : public device {
 public:
  int port;
  motor(int32_t p) : port(p) {}
  motor(int32_t p, bool) : port(p) {}
  motor(int32_t p, gearSetting) : port(p) {}
  motor(int32_t p, gearSetting, bool) : port(p) {}
  void spin(directionType, double, voltageUnits) {}
  void spin(directionType, double, velocityUnits) {}
  void stop() {}
  void stop(brakeType) {}
  double position(rotationUnits) { return g_motor_deg[port]; }
  double velocity(velocityUnits) { return g_motor_vel[port]; }
};
class motor_group {
  int port_;
 public:
  motor_group() : port_(0) {}
  template <typename... Args> motor_group(motor &m1, Args &...) : port_(m1.port) {}
  void spin(directionType, double, voltageUnits) {}
  void stop() {}
  void stop(brakeType) {}
  double position(rotationUnits) { return g_motor_deg[port_]; }
  double velocity(velocityUnits) { return g_motor_vel[port_]; }
};
class inertial : public device {
  double off_;
 public:
  inertial(int32_t) : off_(0) {}
  void calibrate() {}
  bool isCalibrating() { return false; }
  double rotation() { return g_gyro_rot + off_; }
  void setRotation(double v, rotationUnits) { off_ = v - g_gyro_rot; }
};
class rotation : public device { public: rotation(int32_t, bool = false) {} double position(rotationUnits) { return 0; } };
class encoder { public: encoder(triport::port &) {} double position(rotationUnits) { return 0; } };
class gps : public device {
 public:
  gps(int32_t, double, double, distanceUnits, double, turnType = right) {}
  void calibrate() {}
  bool isCalibrating() { return false; }
  double xPosition(distanceUnits = mm) { return g_gps_x_mm; }
  double yPosition(distanceUnits = mm) { return g_gps_y_mm; }
  double heading() { return g_gps_h; }
  int32_t quality() { return g_gps_q; }
};
class serial_link : public device {
 public:
  serial_link(int32_t, const char *, linkType, bool = false) {}
  int32_t send(uint8_t *b, int32_t n) { g_link_tx.insert(g_link_tx.end(), b, b + n); return n; }
  int32_t receive(uint8_t *, int32_t, int32_t = 500) { return 0; }
  bool isLinked() { return true; }
 protected:
  uint32_t crc32(uint8_t *p, uint32_t n, uint32_t acc) {
    for (uint32_t j = 0; j < n; j++) {
      acc ^= (uint32_t)p[j] << 24;
      for (int k = 0; k < 8; k++) acc = (acc & 0x80000000u) ? (acc << 1) ^ 0x04C11DB7u : (acc << 1);
    }
    return acc;
  }
};
class vision { public: class signature {}; class code {}; };
class brain {
 public:
  class lcd { public:
    void clearScreen() {} void printAt(int32_t, int32_t, const char *, ...) {}
    void setFont(fontType) {} void drawRectangle(int, int, int, int) {}
    bool pressing() { return false; } int32_t xPosition() { return 0; } int32_t yPosition() { return 0; } };
  class battery { public: uint32_t capacity() { return 100; } };
  class sdcard { public:
    bool isInserted() { return false; }
    int32_t savefile(const char *, uint8_t *, int32_t) { return 0; }
    int32_t appendfile(const char *, uint8_t *, int32_t) { return 0; }
    int32_t loadfile(const char *, uint8_t *, int32_t) { return 0; } };
  lcd Screen; battery Battery; sdcard SDcard;
};
class controller {
 public:
  class axis { public: int32_t value() { return 0; } };
  class button { public: bool pressing() { return false; } };
  controller() {} controller(controllerType) {}
  axis Axis1, Axis2, Axis3, Axis4; button ButtonB;
};
class competition { public:
  void autonomous(void (*)(void)) {} void drivercontrol(void (*)(void)) {}
  bool isFieldControl() { return false; } bool isCompetitionSwitch() { return false; } };
}  // namespace vex
