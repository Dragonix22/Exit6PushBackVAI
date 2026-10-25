#pragma once
/*---------------------------------------------------------------------------*/
/*  Robot-to-robot link over VEXlink.                                        */
/*                                                                           */
/*  Based on ai_robot_link from VEX-Robotics/VAIC_26_27 (MIT license, James  */
/*  Pearman / Innovation First): same packet framing and CRC, but with our   */
/*  own packet types instead of the demo's single location packet.          */
/*                                                                           */
/*  BANDWIDTH: VEXlink carries roughly 520 bytes/second. Budget:            */
/*    STATE  16 B x 10 Hz = 160 B/s   (where I am, what I'm doing)          */
/*    FIELD <=39 B x 3.3 Hz = 130 B/s (toggles + a few object sightings)    */
/*    EVENT  14 B, rare, each sent 3x for reliability (claims, scores)      */
/*  Keep new packets small, or the link will silently drop data.            */
/*---------------------------------------------------------------------------*/

namespace ai {

namespace link_pkt {

enum Type { STATE = 1, FIELD = 2, EVENT = 3 };

// Bits in StatePayload::flags
enum StateFlags { F_GPS_GOOD = 0x01, F_JETSON_OK = 0x02, F_HOLDING = 0x04,
                  F_PARKING = 0x08 };

typedef struct __attribute__((__packed__)) _StatePayload {
  int16_t  x_mm, y_mm;     // fused pose, field frame
  uint16_t heading_cdeg;   // centidegrees, 0..35999
  uint8_t  task;           // ai::TaskType being executed
  uint8_t  flags;          // StateFlags
  uint8_t  held_class;     // field::ObjClass carried, 0xFF if none
  uint8_t  spare;
} StatePayload;            // 10 bytes

typedef struct __attribute__((__packed__)) _Sighting {
  uint8_t  cls;            // field::ObjClass
  int16_t  x_mm, y_mm;
} Sighting;                // 5 bytes

const int MAX_SIGHTINGS = 6;

typedef struct __attribute__((__packed__)) _FieldPayload {
  uint8_t  toggle_bits;    // 2 bits per quadrant: 0 unknown, 1 red, 2 blue, 3 neutral
  uint8_t  loads_taken;    // match loads this robot has pulled from loaders
  uint8_t  count;          // sightings that follow
  Sighting s[MAX_SIGHTINGS];
} FieldPayload;            // 3 + 5*count bytes (up to 33)

enum EventKind { EV_CLAIM = 1,       // "I'm going for the object near (x,y)"
                 EV_RELEASE = 2,     // claim dropped
                 EV_SCORED = 3,      // a = goal index, b = ObjClass placed
                 EV_TOGGLE = 4,      // a = quadrant, b = field::Alliance it is now set to
                 EV_PICKED = 5 };    // object near (x,y) was picked up (remove it)

typedef struct __attribute__((__packed__)) _EventPayload {
  uint8_t  seq;            // increments per event; receiver drops duplicates
  uint8_t  kind;
  uint8_t  a, b;
  int16_t  x_mm, y_mm;
} EventPayload;            // 8 bytes

}  // namespace link_pkt

class TeamLink : public vex::serial_link {
 public:
  TeamLink(int32_t port, const char *name, vex::linkType type);

  bool linked();

  // ---- outgoing ----
  void set_state(const link_pkt::StatePayload &s);
  void set_field(const link_pkt::FieldPayload &f);
  void send_event(uint8_t kind, uint8_t a, uint8_t b, float x_in, float y_in);

  // ---- incoming ----
  // Returns false if no partner state has arrived in the last max_age_ms.
  bool partner_state(link_pkt::StatePayload &out, uint32_t max_age_ms = 500);
  bool partner_field(link_pkt::FieldPayload &out, uint32_t max_age_ms = 1500);
  bool pop_event(link_pkt::EventPayload &out);

  // ---- stats ----
  uint32_t rx_packets, rx_errors, rx_timeouts, tx_packets, tx_errors;

 private:
  typedef struct __attribute__((__packed__)) _Header {
    uint8_t  sync[2];
    uint8_t  length;
    uint8_t  type;
    uint16_t crc;
  } Header;

  enum class RxState { Sync1, Sync2, Length, Type, Crc, Payload };

  bool send_packet(uint8_t type, const void *payload, uint8_t len);
  void process(uint8_t byte);
  void handle_packet(uint8_t type, const uint8_t *data, uint8_t len);

  // rx
  RxState  rx_state;
  vex::timer rx_timer;
  uint8_t  rx_len, rx_type, rx_index;
  uint16_t rx_crc;
  uint32_t rx_calc;
  uint8_t  rx_buf[64];

  link_pkt::StatePayload partner_state_;
  uint32_t partner_state_ms_;
  link_pkt::FieldPayload partner_field_;
  uint32_t partner_field_ms_;
  static const int EVENT_QUEUE = 16;
  link_pkt::EventPayload rx_events_[EVENT_QUEUE];
  int rx_ev_head_, rx_ev_tail_;
  static const int SEEN_SEQS = 12;
  int seen_seq_[SEEN_SEQS];
  int seen_next_;

  // tx
  link_pkt::StatePayload my_state_;
  link_pkt::FieldPayload my_field_;
  link_pkt::EventPayload tx_events_[EVENT_QUEUE];
  uint8_t tx_event_sends_left_[EVENT_QUEUE];
  uint8_t tx_seq_;

  vex::mutex lock_;

  static int rx_task(void *arg);
  static int tx_task(void *arg);
};

}  // namespace ai
