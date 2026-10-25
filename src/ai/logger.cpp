#include "vex.h"
#include <stdarg.h>

using namespace vex;

// NOTE on threads: VEXcode tasks are cooperative, so code that doesn't call
// wait()/sleep/blocking I/O can't be interrupted. The logger relies on that
// instead of a mutex: when the field ends a period it kills the competition
// task, and a task killed while holding a mutex would freeze everything that
// logs afterwards. append() never blocks; the flush thread does the slow SD
// write from its own copy of the buffer.

namespace ai {
namespace log {

static bool     s_enabled = false;
static char     s_name[24] = "";
static uint8_t  s_buf[4096];
static uint8_t  s_out[4096];
static int      s_len = 0;
static uint32_t s_dropped = 0;

bool enabled() { return s_enabled; }
const char *file_name() { return s_name; }

static int flush_task() {
  while (true) {
    this_thread::sleep_for(500);
    flush();
  }
  return 0;
}

void init() {
  if (s_enabled || !Brain.SDcard.isInserted()) return;
  // Keep a counter on the card so every power-up gets its own file.
  uint8_t num[8] = {0};
  int n = 0;
  if (Brain.SDcard.loadfile("ai_lognum.txt", num, sizeof(num) - 1) > 0) {
    n = atoi((const char *)num) + 1;
  }
  if (n > 999) n = 0;
  int len = snprintf((char *)num, sizeof(num), "%d", n);
  Brain.SDcard.savefile("ai_lognum.txt", num, len);

  snprintf(s_name, sizeof(s_name), "ai_%03d.csv", n);
  const char *header = "time_ms,type,data\n";
  Brain.SDcard.savefile(s_name, (uint8_t *)header, strlen(header));
  s_enabled = true;
  thread t(flush_task);
}

static void append(const char *line, int len) {
  if (!s_enabled || len <= 0) return;
  if (s_len + len > (int)sizeof(s_buf)) {
    s_dropped++;  // buffer full until the next flush; don't block the caller
    return;
  }
  memcpy(s_buf + s_len, line, len);
  s_len += len;
}

void event(const char *fmt, ...) {
  if (!s_enabled) return;
  char line[160];
  int n = snprintf(line, sizeof(line), "%lu,EV,", (unsigned long)now_ms());
  va_list args;
  va_start(args, fmt);
  n += vsnprintf(line + n, sizeof(line) - n - 1, fmt, args);
  va_end(args);
  if (n > (int)sizeof(line) - 2) n = sizeof(line) - 2;
  line[n++] = '\n';
  append(line, n);
}

void pose_row(float x, float y, float h, int gps_q, float gps_err, int task) {
  if (!s_enabled) return;
  char line[96];
  int n = snprintf(line, sizeof(line), "%lu,POSE,%.1f,%.1f,%.1f,%d,%.1f,%d\n",
                   (unsigned long)now_ms(), x, y, h, gps_q, gps_err, task);
  if (n > 0 && n < (int)sizeof(line)) append(line, n);
}

void flush() {
  if (!s_enabled || s_len == 0) return;
  // Copy-and-reset doesn't yield, so no other task can append mid-copy.
  int n = s_len;
  memcpy(s_out, s_buf, n);
  s_len = 0;
  Brain.SDcard.appendfile(s_name, s_out, n);
}

}  // namespace log
}  // namespace ai
