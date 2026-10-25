/*---------------------------------------------------------------------------*/
/*  Robot-to-robot link over VEXlink. Framing and CRC follow ai_robot_link   */
/*  from VEX-Robotics/VAIC_26_27 (MIT license, Innovation First).            */
/*---------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

namespace ai {

using namespace link_pkt;

// Both robots must agree on these sizes byte-for-byte.
static_assert(sizeof(StatePayload) == 10, "StatePayload size changed");
static_assert(sizeof(Sighting) == 5, "Sighting size changed");
static_assert(sizeof(FieldPayload) == 3 + 5 * MAX_SIGHTINGS, "FieldPayload size changed");
static_assert(sizeof(EventPayload) == 8, "EventPayload size changed");

static const uint8_t SYNC1 = 0xA5;
static const uint8_t SYNC2 = 0x5A;

TeamLink::TeamLink(int32_t port, const char *name, linkType type)
    : serial_link(port, name, type),
      rx_packets(0), rx_errors(0), rx_timeouts(0), tx_packets(0), tx_errors(0),
      rx_state(RxState::Sync1), rx_len(0), rx_type(0), rx_index(0), rx_crc(0),
      rx_calc(0), partner_state_ms_(0), partner_field_ms_(0), rx_ev_head_(0),
      rx_ev_tail_(0), seen_next_(0), tx_seq_(0) {
  for (int i = 0; i < SEEN_SEQS; i++) seen_seq_[i] = -1;
  memset(&partner_state_, 0, sizeof(partner_state_));
  memset(&partner_field_, 0, sizeof(partner_field_));
  memset(&my_state_, 0, sizeof(my_state_));
  memset(&my_field_, 0, sizeof(my_field_));
  my_state_.held_class = 0xFF;
  memset(tx_event_sends_left_, 0, sizeof(tx_event_sends_left_));

  thread t1(tx_task, static_cast<void *>(this));
  thread t2(rx_task, static_cast<void *>(this));
}

bool TeamLink::linked() { return isLinked(); }

/*------------------------------- outgoing ----------------------------------*/
void TeamLink::set_state(const StatePayload &s) {
  lock_.lock();
  my_state_ = s;
  lock_.unlock();
}

void TeamLink::set_field(const FieldPayload &f) {
  lock_.lock();
  my_field_ = f;
  if (my_field_.count > MAX_SIGHTINGS) my_field_.count = MAX_SIGHTINGS;
  lock_.unlock();
}

void TeamLink::send_event(uint8_t kind, uint8_t a, uint8_t b, float x_in, float y_in) {
  lock_.lock();
  // Find a free slot (or overwrite the oldest-sent one).
  int slot = 0;
  for (int i = 0; i < EVENT_QUEUE; i++) {
    if (tx_event_sends_left_[i] == 0) { slot = i; break; }
  }
  EventPayload &e = tx_events_[slot];
  e.seq = ++tx_seq_;
  e.kind = kind;
  e.a = a;
  e.b = b;
  e.x_mm = (int16_t)field::in_to_mm(x_in);
  e.y_mm = (int16_t)field::in_to_mm(y_in);
  tx_event_sends_left_[slot] = 3;  // VEXlink can drop packets; repeat
  lock_.unlock();
}

bool TeamLink::send_packet(uint8_t type, const void *payload, uint8_t len) {
  uint8_t buf[sizeof(Header) + 64];
  if (len > 64) return false;
  Header *h = reinterpret_cast<Header *>(buf);
  h->sync[0] = SYNC1;
  h->sync[1] = SYNC2;
  h->length = len;
  h->type = type;
  memcpy(buf + sizeof(Header), payload, len);
  h->crc = (uint16_t)crc32(buf + sizeof(Header), len, 0);
  if (send(buf, sizeof(Header) + len) > 0) {
    tx_packets++;
    return true;
  }
  tx_errors++;
  return false;
}

int TeamLink::tx_task(void *arg) {
  TeamLink *self = static_cast<TeamLink *>(arg);
  while (!self->isLinked()) this_thread::sleep_for(50);

  uint32_t tick = 0;
  while (true) {
    // Copy what to send while holding the lock, send without it.
    StatePayload st;
    FieldPayload fd;
    EventPayload evs[2];
    int nev = 0;

    self->lock_.lock();
    st = self->my_state_;
    fd = self->my_field_;
    // At most 2 events per tick to stay inside the bandwidth budget.
    for (int i = 0; i < EVENT_QUEUE && nev < 2; i++) {
      if (self->tx_event_sends_left_[i] > 0) {
        evs[nev++] = self->tx_events_[i];
        self->tx_event_sends_left_[i]--;
      }
    }
    self->lock_.unlock();

    self->send_packet(STATE, &st, sizeof(st));
    if (tick % 3 == 0) {
      uint8_t len = (uint8_t)(3 + fd.count * sizeof(Sighting));
      self->send_packet(FIELD, &fd, len);
    }
    for (int i = 0; i < nev; i++) {
      self->send_packet(EVENT, &evs[i], sizeof(EventPayload));
    }
    tick++;
    this_thread::sleep_for(100);  // 10 Hz
  }
  return 0;
}

/*------------------------------- incoming ----------------------------------*/
bool TeamLink::partner_state(StatePayload &out, uint32_t max_age_ms) {
  lock_.lock();
  out = partner_state_;
  uint32_t t = partner_state_ms_;
  lock_.unlock();
  return t != 0 && now_ms() - t <= max_age_ms;
}

bool TeamLink::partner_field(FieldPayload &out, uint32_t max_age_ms) {
  lock_.lock();
  out = partner_field_;
  uint32_t t = partner_field_ms_;
  lock_.unlock();
  return t != 0 && now_ms() - t <= max_age_ms;
}

bool TeamLink::pop_event(EventPayload &out) {
  bool got = false;
  lock_.lock();
  if (rx_ev_tail_ != rx_ev_head_) {
    out = rx_events_[rx_ev_tail_];
    rx_ev_tail_ = (rx_ev_tail_ + 1) % EVENT_QUEUE;
    got = true;
  }
  lock_.unlock();
  return got;
}

void TeamLink::handle_packet(uint8_t type, const uint8_t *data, uint8_t len) {
  lock_.lock();
  if (type == STATE && len == sizeof(StatePayload)) {
    // If the partner went quiet for a while it may have restarted its program,
    // which restarts its event numbering; forget old event numbers.
    if (now_ms() - partner_state_ms_ > 2000) {
      for (int i = 0; i < SEEN_SEQS; i++) seen_seq_[i] = -1;
    }
    memcpy(&partner_state_, data, len);
    partner_state_ms_ = now_ms();
  } else if (type == FIELD && len >= 3 && len <= sizeof(FieldPayload)) {
    memset(&partner_field_, 0, sizeof(partner_field_));
    memcpy(&partner_field_, data, len);
    if (partner_field_.count > MAX_SIGHTINGS) partner_field_.count = MAX_SIGHTINGS;
    partner_field_ms_ = now_ms();
  } else if (type == EVENT && len == sizeof(EventPayload)) {
    EventPayload e;
    memcpy(&e, data, len);
    // Each event is sent 3 times; drop copies we've already seen.
    bool seen = false;
    for (int i = 0; i < SEEN_SEQS; i++) {
      if (seen_seq_[i] == (int)e.seq) seen = true;
    }
    if (!seen) {
      seen_seq_[seen_next_] = e.seq;
      seen_next_ = (seen_next_ + 1) % SEEN_SEQS;
      int next = (rx_ev_head_ + 1) % EVENT_QUEUE;
      if (next != rx_ev_tail_) {
        rx_events_[rx_ev_head_] = e;
        rx_ev_head_ = next;
      }
    }
  }
  lock_.unlock();
}

void TeamLink::process(uint8_t byte) {
  // 250 ms inter-byte timeout resets the decoder
  if (rx_state != RxState::Sync1 && rx_timer.time() > 250) {
    rx_timeouts++;
    rx_state = RxState::Sync1;
  }
  rx_timer.clear();

  switch (rx_state) {
    case RxState::Sync1:
      if (byte == SYNC1) rx_state = RxState::Sync2;
      break;
    case RxState::Sync2:
      rx_state = (byte == SYNC2) ? RxState::Length : RxState::Sync1;
      break;
    case RxState::Length:
      rx_len = byte;
      rx_state = (rx_len <= sizeof(rx_buf)) ? RxState::Type : RxState::Sync1;
      break;
    case RxState::Type:
      rx_type = byte;
      rx_index = 0;
      rx_crc = 0;
      rx_state = RxState::Crc;
      break;
    case RxState::Crc:
      rx_crc = (uint16_t)((rx_crc >> 8) | ((uint16_t)byte << 8));  // little endian
      if (++rx_index == 2) {
        rx_index = 0;
        rx_calc = 0;
        rx_state = RxState::Payload;
        if (rx_len == 0) {  // empty payload
          rx_state = RxState::Sync1;
        }
      }
      break;
    case RxState::Payload:
      rx_buf[rx_index++] = byte;
      rx_calc = crc32(&byte, 1, rx_calc);
      if (rx_index == rx_len) {
        if ((uint16_t)rx_calc == rx_crc) {
          rx_packets++;
          handle_packet(rx_type, rx_buf, rx_len);
        } else {
          rx_errors++;
        }
        rx_state = RxState::Sync1;
      }
      break;
  }
}

int TeamLink::rx_task(void *arg) {
  TeamLink *self = static_cast<TeamLink *>(arg);
  uint8_t buffer[128];
  while (true) {
    int32_t n = self->receive(buffer, sizeof(buffer), 10);
    for (int32_t i = 0; i < n; i++) self->process(buffer[i]);
    this_thread::sleep_for(5);
  }
  return 0;
}

}  // namespace ai
