#include "marantec_opener.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace marantec_opener {

static const char *const TAG = "marantec_opener.cover";
static const uint8_t RX_MSG_LEN = 5;
static const uint8_t TX_MSG_LEN = 5;
static const uint8_t GAP_BETWEEN_MSG_MS = 90;
static const uint8_t COMMAND_WAKEUP[] = {0x00};
static const uint8_t COMMAND_CLOSE[] = {0x6A, 0x01, 0x01, 0x08, 0xD2};
static const uint8_t COMMAND_OPEN[] = {0x6A, 0x01, 0x01, 0x10, 0xA1};
static const uint8_t COMMAND_IDLEA[] = {0x6A, 0x01, 0x01, 0x00, 0x3E};

using namespace esphome::cover;

bool MarantecOpener::read_next_msg(uint8_t *data) {
  uint8_t pb;

  while (this->available() > 0 && this->peek_byte(&pb) && pb != 0xEA) {
    this->read_byte(&pb);
    // ESP_LOGD(TAG, "read wrong byte %x on uart",pb);
  }
  // ESP_LOGD(TAG, "read new msg on uart");
  // incomplete messages have now been ignored, ready to read a full msg
  if (this->available() >= RX_MSG_LEN) {
    for (int i = 0; i < RX_MSG_LEN; i++) {
      // ESP_LOGD(TAG, "byte %i",i);
      this->read_byte(data + i);
    }

    this->last_rx_msg_ = millis();
    // this->process_rx_(data);

    return true;
  }
  return false;
}

// TODO merge with read_next_msg, optional timeout for repeated tries
bool MarantecOpener::read_next_msg(uint8_t *data, uint16_t timeout_ms) {
  uint32_t since = millis();

  while (millis() - since < timeout_ms) {
    if (read_next_msg(data)) {
      return true;
    }
  }

  return false;
}

void MarantecOpener::setup() {
  // delay(5000);
  // ESP_LOGE(TAG, "staofsetup");
  auto restore = this->restore_state_();

  if (restore.has_value()) {
    restore->apply(this);
    this->publish_state(false);
  } else {
    // if no other information, assume half open
    this->position = 0.5f;
  }

  // get actual value
  uint8_t data_msg[RX_MSG_LEN];

  bool r = read_next_msg(data_msg, 2 * GAP_BETWEEN_MSG_MS);
  // ESP_LOGD(TAG, "midofsetup");
  // if opener is not awake, trigger wakeup and read again
  if (!r) {
    // ESP_LOGD(TAG, "opener is not awake, trigger wakeup and read again");
    this->wakeup_bus_();
    r = read_next_msg(data_msg, 3 * GAP_BETWEEN_MSG_MS);
  }

  if (r) {
    process_rx_(data_msg);
  }

  this->last_recompute_time_ = this->start_dir_time_ = millis();

  this->set_interval(300, [this]() { this->update_(); });
  // ESP_LOGD(TAG, "endofsetup");
}

CoverTraits MarantecOpener::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  traits.set_is_assumed_state(false);
  return traits;
}

void MarantecOpener::dump_config() {
  LOG_COVER("", "Marantec Cover", this);
  this->check_uart_settings(1000, 1, uart::UART_CONFIG_PARITY_NONE, 8);
  ESP_LOGCONFIG(TAG,
                "  Open Duration: %.1fs\n"
                "  Close Duration: %.1fs",
                this->open_duration_ / 1e3f, this->close_duration_ / 1e3f);
  auto restore = this->restore_state_();
  if (restore.has_value())
    ESP_LOGCONFIG(TAG, "  Saved position %d%%", (int) (restore->position * 100.f));
}

void MarantecOpener::endstop_reached_(CoverOperation operation) {
  const uint32_t now = millis();

  this->set_current_operation_(COVER_OPERATION_IDLE);
  auto new_position = (operation == COVER_OPERATION_OPENING) ? COVER_OPEN : COVER_CLOSED;
  if (new_position != this->position || this->current_operation != COVER_OPERATION_IDLE) {
    this->position = new_position;
    this->current_operation = COVER_OPERATION_IDLE;
    float dur = (float) (now - this->start_dir_time_) / 1e3f;
    ESP_LOGD(TAG, "'%s' - %s endstop reached. Took %.1fs.", this->name_.c_str(),
             operation == COVER_OPERATION_OPENING ? "Open" : "Close", dur);
    this->publish_state();
  }
}

void MarantecOpener::set_current_operation_(cover::CoverOperation operation) {
  if (this->current_operation != operation) {
  cover:
    CoverOperation prev_op = this->current_operation;
    this->current_operation = operation;
    if (operation != COVER_OPERATION_IDLE) {
      this->last_recompute_time_ = millis();
      this->start_dir_time_ = millis();
    }
    if (prev_op != this->current_operation) {
      this->previous_operation = prev_op;
    }
  }
}

void MarantecOpener::process_rx_(uint8_t *msg) {
  // TODO: Check if change happened in operation and/or next direction
  ESP_LOGD(TAG, "Process RX data %02X %02X %02X %02X %02X", msg[0], msg[1], msg[2], msg[3], msg[4]);

  if (msg[0] == 0xEA && msg[1] == 0x01 && msg[2] == 0x0A) {
    if (msg[3] == 0x04 && msg[4] == 0xBE) {
      // idle at closed endstop
      this->endstop_reached_(COVER_OPERATION_CLOSING);
    } else if (msg[3] == 0x07 && msg[4] == 0x3B) {
      // idle at opened endstop
      this->endstop_reached_(COVER_OPERATION_OPENING);
    } else if (msg[3] == 0x00 && msg[4] == 0xC8) {
      // idle at stop midway
      this->set_current_operation_(COVER_OPERATION_IDLE);
    } else if (msg[3] == 0xC0 && msg[4] == 0x99) {
      this->set_current_operation_(COVER_OPERATION_OPENING);
    } else if (msg[3] == 0x80 && msg[4] == 0x6B) {
      this->set_current_operation_(COVER_OPERATION_CLOSING);
    }

  } else if (msg[0] == 0xEA && msg[1] == 0x01 && msg[2] == 0x0B) {
    // special state between moving and stopped - maybe sth. like the motor coasting to stop?
    // ignore for now
  }
}

void MarantecOpener::update_() {
  /// ESP_LOGD(TAG, "update-call");
  if (this->current_operation != COVER_OPERATION_IDLE) {
    this->recompute_position_();

    // check if we reached the target position
    if (this->is_at_target_()) {
      this->start_direction_(COVER_OPERATION_IDLE);
    }
  }
}

void MarantecOpener::wakeup_bus_() {
  if (millis() - this->last_rx_msg_ > 10 * GAP_BETWEEN_MSG_MS &&
      millis() - this->last_wakeup_call_ > 3 * GAP_BETWEEN_MSG_MS) {
    // if bus / motor is idle send wakeup
    ESP_LOGD(TAG, "Wakeup bus");
    this->write_array(COMMAND_WAKEUP, 1);
    this->last_wakeup_call_ = millis();
    // this->flush();
    delay(10);
  }
}

void MarantecOpener::loop() {
  uint8_t data[RX_MSG_LEN];

  if (read_next_msg(data)) {
    // ESP_LOGD(TAG, "read package succ");
    this->process_rx_(data);  // TODO: does this take too long?
    // ESP_LOGD(TAG, "time since last send: %u",millis() - this->last_rx_msg_);
  }

  if (this->enqueued_command_ != COVER_OPERATION_IDLE) {
    // ESP_LOGD(TAG,"command needs to be send");
    this->wakeup_bus_();

    // send enqueued command
    if (millis() - this->last_rx_msg_ < 30) {
      ESP_LOGD(TAG, "in correct time slot for sending command, do so");
      // enque command only if we are just in a silent slot, max 40ms after end of last msg OR when bus is idle/silent
      // for
      while (millis() - this->last_rx_msg_ <= 16) {
        delay(1);
      }
      uint32_t startt = millis();
      if (this->enqueued_command_ == COVER_OPERATION_OPENING) {
        this->write_array(COMMAND_OPEN, TX_MSG_LEN);
        // this->flush();
        //  keepalive_count=100;
      } else if (this->enqueued_command_ == COVER_OPERATION_CLOSING) {
        this->write_array(COMMAND_CLOSE, TX_MSG_LEN);
        // this->flush();
      }

      ESP_LOGD(TAG, "Wrote command %s to serial..duration: %u ms, offset after receive %u",
               this->enqueued_command_ == COVER_OPERATION_OPENING   ? "OPEN"
               : this->enqueued_command_ == COVER_OPERATION_CLOSING ? "CLOSE"
                                                                    : "STOP",
               millis() - startt, startt - this->last_rx_msg_);

      this->enqueued_command_ = COVER_OPERATION_IDLE;
    }
  }
}

void MarantecOpener::control(const CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(COVER_OPERATION_IDLE);
  } else if (call.get_toggle().has_value()) {
    // toggle action logic: OPEN - STOP - CLOSE

    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->start_direction_(COVER_OPERATION_IDLE);
    } else {
      // motor was idle look back to last state
      if (this->previous_operation == COVER_OPERATION_OPENING) {
        this->start_direction_(COVER_OPERATION_CLOSING);
      } else if (this->previous_operation == COVER_OPERATION_CLOSING) {
        this->start_direction_(COVER_OPERATION_OPENING);
      }
    }

  } else if (call.get_position().has_value()) {
    // go to position action
    auto pos = *call.get_position();
    // are we at the target?
    if (pos == this->position) {
      this->start_direction_(COVER_OPERATION_IDLE);
    } else {
      this->target_position_ = pos;
      this->start_direction_(pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING);
    }
  }
}

/**
 * Check if the cover has reached or passed the target position. This is used only
 * for partial open/close requests - endstops are used for full open/close.
 * @return True if the cover has reached or passed its target position. For full open/close target always return false.
 */
bool MarantecOpener::is_at_target_() const {
  // equality of floats is fraught with peril - this is reliable since the values are 0.0 or 1.0 which are
  // exactly representable.
  if (this->target_position_ == COVER_OPEN || this->target_position_ == COVER_CLOSED)
    return false;
  // aiming for an intermediate position - exact comparison here will not work and we need to allow for overshoot
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
    case COVER_OPERATION_IDLE:
      return this->current_operation == COVER_OPERATION_IDLE;
    default:
      return true;
  }
}

void MarantecOpener::start_direction_(CoverOperation dir) {
  ESP_LOGD(TAG, "'%s' - Direction '%s' requested.", this->name_.c_str(),
           dir == COVER_OPERATION_OPENING   ? "OPEN"
           : dir == COVER_OPERATION_CLOSING ? "CLOSE"
                                            : "STOP");

  if (this->current_operation == dir) {
    ESP_LOGD(TAG, "No change in direction, dont do anything");
  } else {
    // if the cover is moving, both open and close commands are interpreted as a
    // stop (use close here).
    this->enqueued_command_ = (dir != COVER_OPERATION_IDLE) ? dir : COVER_OPERATION_CLOSING;
  }
}

void MarantecOpener::recompute_position_() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  const uint32_t now = millis();
  if (now > this->last_recompute_time_) {
    auto diff = (unsigned) (now - last_recompute_time_);
    float delta;
    switch (this->current_operation) {
      case COVER_OPERATION_OPENING:
        delta = (float) diff / (float) this->open_duration_;
        break;
      case COVER_OPERATION_CLOSING:
        delta = -(float) diff / (float) this->close_duration_;
        break;
      default:
        return;
    }

    // make sure our guesstimate never reaches full open or close.
    auto new_position = clamp(delta + this->position, COVER_CLOSED + 0.01f, COVER_OPEN - 0.01f);
    ESP_LOGD(TAG, "Recompute %ums, dir=%u, delta=%f, pos=%f", diff, this->current_operation, delta, new_position);
    this->last_recompute_time_ = now;
    if (this->position != new_position) {
      this->position = new_position;
      this->publish_state();
    }
  }
}

}  // namespace marantec_opener
}  // namespace esphome
