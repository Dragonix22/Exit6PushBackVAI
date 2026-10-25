#pragma once
// Milliseconds since the Brain started. Wrapped so all AI code uses one clock.
namespace ai {
inline uint32_t now_ms() {
  static vex::timer t;
  return t.system();
}
}  // namespace ai
