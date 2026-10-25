#pragma once
/*---------------------------------------------------------------------------*/
/*  Field model for the VEX AI Override field (Section 7 layout).            */
/*                                                                           */
/*  FRAME: field-centered, inches. +Y and heading 0 point the same way as    */
/*  the GPS field code, and heading increases clockwise. This is the same    */
/*  convention JAR uses, so after ai::pose seeds JAR from the GPS,           */
/*  chassis.drive_to_point(x, y) takes these field coordinates directly.     */
/*                                                                           */
/*  WHERE THE NUMBERS COME FROM: goal, toggle and loader positions are from  */
/*  the VEXcode VR Override playground (GPS frame, mm), with the four goals  */
/*  that Section 7 removes taken out. VERIFY them on a real field with the   */
/*  GPS before trusting them, and check Appendix A of the manual. Values     */
/*  marked "TODO verify" are placeholders.                                   */
/*---------------------------------------------------------------------------*/

namespace ai {
namespace field {

inline float mm_to_in(float mm) { return mm / 25.4f; }
inline float m_to_in(float m)   { return m * 39.3701f; }
inline float in_to_mm(float in) { return in * 25.4f; }

const float HALF_FIELD_IN = 70.2f;  // center to inside of perimeter. TODO verify

/*------------------------------- Alliances ---------------------------------*/
enum Alliance { RED = 0, BLUE = 1, NO_ALLIANCE = 2 };  // NO_ALLIANCE = neutral/yellow

inline Alliance opponent(Alliance a) {
  return a == RED ? BLUE : (a == BLUE ? RED : NO_ALLIANCE);
}

// Red's half is the left + bottom quadrants (x + y < 0 in the GPS frame).
// Robots start on their own side, so this identifies our alliance from the
// first GPS fix. Check it once on a real field.
inline Alliance side_of(float x, float y) { return (x + y < 0.0f) ? RED : BLUE; }

/*------------------------------- Quadrants ---------------------------------*/
// The four triangles between the field diagonals. Numbering follows the
// manual (Red 1, Red 2, Blue 1, Blue 2 go counterclockwise from the red goal).
enum Quadrant { Q_RED1 = 0,   // left   (red goal, AprilTag 1, red loaders' wall)
                Q_RED2 = 1,   // bottom (neutral goal, AprilTag 2)
                Q_BLUE1 = 2,  // right  (blue goal, AprilTag 3)
                Q_BLUE2 = 3,  // top    (neutral goal, AprilTag 4)
                NUM_QUADRANTS = 4,
                Q_NONE = -1 };

inline Alliance quadrant_color(int q) { return q <= Q_RED2 ? RED : BLUE; }

inline Quadrant quadrant_of(float x, float y) {
  float ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
  if (ax >= ay) return x < 0 ? Q_RED1 : Q_BLUE1;
  return y < 0 ? Q_RED2 : Q_BLUE2;
}

/*-------------------------------- Midfield ---------------------------------*/
const float MIDFIELD_HALF_IN = 24.0f;  // half-width of the Midfield square. TODO verify

inline bool in_midfield(float x, float y, float margin_in = 0.0f) {
  float h = MIDFIELD_HALF_IN - margin_in;
  return x > -h && x < h && y > -h && y < h;
}

/*--------------------------------- Goals -----------------------------------*/
struct Goal {
  int      april_tag;   // Section 7 numbering
  float    x, y;        // inches
  int      quadrant;    // Q_NONE for the Midfield goal
  Alliance color;       // RED/BLUE alliance goal, or NO_ALLIANCE for neutral
  bool     tall;
};

const int NUM_GOALS = 5;
const int MIDFIELD_GOAL = 0;

const Goal GOALS[NUM_GOALS] = {
  // tag   x (in)    y (in)   quadrant  color        tall
  {  0,    0.000f,   0.000f, Q_NONE,  NO_ALLIANCE, true  },  // Midfield goal
  {  1,  -47.244f, -23.622f, Q_RED1,  RED,         false },  // (-1200,-600) mm
  {  2,   23.622f, -47.244f, Q_RED2,  NO_ALLIANCE, false },  // ( 600,-1200) mm
  {  3,   47.244f,  23.622f, Q_BLUE1, BLUE,        false },  // ( 1200, 600) mm
  {  4,  -23.622f,  47.244f, Q_BLUE2, NO_ALLIANCE, false },  // (-600, 1200) mm
};

// Robots may not interact with the opposing alliance's goal (SG9).
inline bool goal_usable_by(int g, Alliance us) {
  return GOALS[g].color == NO_ALLIANCE || GOALS[g].color == us;
}

/*-------------------------------- Toggles ----------------------------------*/
struct Toggle {
  int   quadrant;
  float x, y;          // center of the toggle on the wall, inches
  float face_heading;  // heading that faces the wall at this toggle
};

const Toggle TOGGLES[NUM_QUADRANTS] = {
  { Q_RED1,  -70.079f,   0.000f, 270.0f },  // left wall   (-1780, 0) mm
  { Q_RED2,    0.000f, -70.079f, 180.0f },  // bottom wall (0, -1780) mm
  { Q_BLUE1,  70.079f,   0.000f,  90.0f },  // right wall  (1780, 0) mm
  { Q_BLUE2,   0.000f,  70.079f,   0.0f },  // top wall    (0, 1780) mm
};

/*-------------------------------- Loaders ----------------------------------*/
struct Loader { Alliance color; float x, y; };

const int NUM_LOADERS = 4;
const Loader LOADERS[NUM_LOADERS] = {
  { RED,  -68.504f,  58.661f },  // (-1740,  1490) mm
  { RED,  -68.504f, -58.661f },  // (-1740, -1490) mm
  { BLUE,  68.504f,  58.661f },  // assumed mirror of red. TODO verify
  { BLUE,  68.504f, -58.661f },  // assumed mirror of red. TODO verify
};

// Per alliance in VEX AI: 10 Cups + 10 alliance/yellow Pins (no yellow/yellow).
const int MATCH_LOAD_PINS = 10;
const int MATCH_LOAD_CUPS = 10;

/*------------------------------ Game objects -------------------------------*/
// Must match labels.txt on the Jetson (VAIC_26_27 model), in this order.
enum ObjClass { RED_BLUE_PIN = 0, BLUE_YELLOW_PIN = 1, RED_YELLOW_PIN = 2,
                YELLOW_PIN = 3, CUP = 4, NUM_CLASSES = 5, NO_CLASS = -1 };

inline bool is_pin(int c) { return c >= RED_BLUE_PIN && c <= YELLOW_PIN; }

enum HalfColor { HALF_RED = 0, HALF_BLUE = 1, HALF_YELLOW = 2 };

inline void pin_halves(int c, HalfColor &a, HalfColor &b) {
  switch (c) {
    case RED_BLUE_PIN:    a = HALF_RED;    b = HALF_BLUE;   break;
    case BLUE_YELLOW_PIN: a = HALF_BLUE;   b = HALF_YELLOW; break;
    case RED_YELLOW_PIN:  a = HALF_RED;    b = HALF_YELLOW; break;
    default:              a = HALF_YELLOW; b = HALF_YELLOW; break;
  }
}

// Our alliance's match-load pin (red/yellow for red, blue/yellow for blue).
inline ObjClass match_load_pin(Alliance us) {
  return us == RED ? RED_YELLOW_PIN : BLUE_YELLOW_PIN;
}

/*-------------------------------- Scoring ----------------------------------*/
// TODO verify every value against the Section 2 scoring table.
const int PTS_ALLIANCE_HALF   = 5;   // each visible red/blue half of a Placed Pin
const int PTS_OWNED_YELLOW    = 10;  // each visible yellow half, to the owner
const int PTS_MIDFIELD_ROBOT  = 8;   // each robot ending in the Midfield
const int PTS_ISOLATION_BONUS = 12;

/*------------------------------ Match timing -------------------------------*/
const float ISOLATION_S   = 15.0f;
const float INTERACTION_S = 105.0f;

}  // namespace field
}  // namespace ai
