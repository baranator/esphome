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
static const uint8_t *COMMAND_STOP = COMMAND_OPEN;
static const uint8_t COMMAND_IDLEA[] = {0x6A, 0x01, 0x01, 0x00, 0x3E};

using namespace esphome::cover;

bool from_endstop = false;
float from_which_endstop = COVER_CLOSED;
unsigned long from_endstop_ts = 0;
bool coasting = false;
bool external_cmd = true;
bool twostepaction = false;

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
    delay(10);
  }

  return false;
}

void MarantecOpener::setup() {
  // delay(5000);
  ESP_LOGD(TAG, "start of setup");
  // get actual value
  uint8_t data_msg[RX_MSG_LEN];

  bool r = read_next_msg(data_msg, 2 * GAP_BETWEEN_MSG_MS);

  // ESP_LOGD(TAG, "midofsetup");
  // if opener is not awake, trigger wakeup and read again
  int failcount = 0;
  if (!r) {
    ESP_LOGD(TAG, "STARTUP: opener is not awake, trigger wakeup and read again");
    while (!this->wakeup_bus_() && failcount < 5) {
      failcount++;
      ESP_LOGD(TAG, "STARTUP: wakeup triggered, waiting for successful read (try #%i)", failcount);
      // only continue if the bus is awake/active, otherwise  start over
      delay(5);
      r = read_next_msg(data_msg, 2000);
    }
  }

  if (r) {
    process_rx_(data_msg);
  } else {
    ESP_LOGE(TAG, "STARTUP: initial state could not be retrieved. Check pin-config & connection. Using default values");
    this->position = COVER_CLOSED;
    this->current_operation = COVER_OPERATION_IDLE;
  }
  this->publish_state();

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

const char *print_operation(CoverOperation o) {
  switch (o) {
    case COVER_OPERATION_IDLE:
      return "IDLE";
    case COVER_OPERATION_CLOSING:
      return "CLOSE";
    case COVER_OPERATION_OPENING:
      return "OPEN";
    default:
      return "*unknown*";
  }
}

const char *print_command(const uint8_t *cmd) {
  if (cmd == COMMAND_CLOSE) {
    return "CLOSE/STOP";
  } else if (cmd == COMMAND_OPEN) {
    return "OPEN";
  } else {
    return "*unknown*";
  }
}

void MarantecOpener::dump_config() {
  LOG_COVER("", "Marantec Cover", this);
  this->check_uart_settings(1000, 1, uart::UART_CONFIG_PARITY_NONE, 8);
}

void MarantecOpener::set_current_operation_(cover::CoverOperation new_operation) {
  if (this->current_operation == new_operation) {
    return;
  }

  ESP_LOGD(TAG, "operation changed: %s@%.1f %% to %s ", print_operation(this->current_operation), this->position,
           print_operation(new_operation));

  if (external_cmd) {
    // for external triggered up/down set the target to the endposition
    // for an external triggered IDLE there is no difference in handling from the internal command

    if (new_operation == COVER_OPERATION_OPENING) {
      this->target_position_ = COVER_OPEN;
    } else if (new_operation == COVER_OPERATION_CLOSING) {
      this->target_position_ = COVER_CLOSED;
    }
    ESP_LOGD(TAG, "external command %s is being processed.", print_operation(new_operation));
  }

  // TODO; right here? restructure statemachining in this method
  if (twostepaction) {
    ESP_LOGD(TAG, "processing twostep-action in phase %s", print_operation(this->current_operation));
    if (new_operation == COVER_OPERATION_IDLE) {
      // if we are in step two of a twostep-action
      twostepaction = false;
      cover::CoverOperation from = this->previous_operation;
      ESP_LOGD(TAG, "akt: %.1f  target: %.1f", this->position, this->target_position_);
      if (this->target_position_ > this->position) {
        this->enqueue_command_(COMMAND_OPEN);
      } else if (this->target_position_ < this->position) {
        this->enqueue_command_(COMMAND_CLOSE);
      }
      // twostepaction has finished, reset external_cmd-flag
      external_cmd = true;
    } else {
      // what to do in step one of twostep-action: do not reset external_cmd-flag
    }
  } else {
    external_cmd = true;
  }

  if (new_operation != COVER_OPERATION_IDLE) {
    this->last_recompute_time_ = millis();
    this->start_dir_time_ = millis();

    // calibrate durations
    if (new_operation == COVER_OPERATION_OPENING && this->position < 0.05) {
      from_endstop = true;
      from_which_endstop = COVER_CLOSED;
      from_endstop_ts = millis();
    } else if (new_operation == COVER_OPERATION_CLOSING && this->position > 0.95) {
      from_endstop = true;
      from_which_endstop = COVER_OPEN;
      from_endstop_ts = millis();
    } else {
      // shouldnt be needed
      from_endstop = false;
    }

  } else {
    if (from_endstop) {
      if (this->position == COVER_CLOSED && from_which_endstop == COVER_OPEN) {
        // TODO: IDLE at top or bottom, measure time & reset
        this->close_duration_ = millis() - from_endstop_ts;
        ESP_LOGD(TAG, "calculated new close-duration: %u", this->close_duration_);
      } else if (this->position == COVER_OPEN && from_which_endstop == COVER_CLOSED) {
        this->open_duration_ = millis() - from_endstop_ts;
        ESP_LOGD(TAG, "calculated new open-duration: %u", this->open_duration_);
      } else {
        from_endstop = false;
      }
    }
  }

  this->previous_operation = this->current_operation;
  this->current_operation = new_operation;
  this->publish_state();
}

void MarantecOpener::process_rx_(uint8_t *msg) {
  // TODO: Check if change happened in operation and/or next direction
  ESP_LOGD(TAG, "Process RX data %02X %02X %02X %02X %02X", msg[0], msg[1], msg[2], msg[3], msg[4]);

  if (msg[0] == 0xEA && msg[1] == 0x01 && msg[2] == 0x0A) {
    coasting = false;
    if (msg[3] == 0x04 && msg[4] == 0xBE) {
      // idle at closed endstop
      this->position = COVER_CLOSED;
      this->set_current_operation_(COVER_OPERATION_IDLE);
    } else if (msg[3] == 0x07 && msg[4] == 0x3B) {
      // idle at opened endstop
      this->position = COVER_OPEN;
      this->set_current_operation_(COVER_OPERATION_IDLE);
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
    coasting = true;
  }
}

void MarantecOpener::update_() {
  /// ESP_LOGD(TAG, "update-call");
  if (this->current_operation != COVER_OPERATION_IDLE) {
    this->recompute_position_();

    // check if we reached the target position
    if (this->is_at_target_() && !coasting) {
      this->enqueue_command_(COMMAND_STOP);
    }
  }
}

/**
 * Wakeup the bus.
 * @return True if the bus is already awake, false if it is not and a wakeup-call was inited now or recently
 */
bool MarantecOpener::wakeup_bus_() {
  if (millis() - this->last_rx_msg_ > 10 * GAP_BETWEEN_MSG_MS) {
    // bus is not awake
    if (millis() - this->last_wakeup_call_ > 3 * GAP_BETWEEN_MSG_MS) {
      // if bus / motor is idle & last wakeup was some time ago send wakeup
      ESP_LOGD(TAG, "Wakeup bus");
      this->write_array(COMMAND_WAKEUP, 1);
      this->last_wakeup_call_ = millis();
      this->flush();
      delay(5);
    }
    return false;
  }
  return true;
}

void MarantecOpener::loop() {
  uint8_t data[RX_MSG_LEN];

  // always check for status updates; if a message is received, one can conclude that the bus is awake
  if (read_next_msg(data)) {
    // ESP_LOGD(TAG, "read package succ");
    this->process_rx_(data);  // TODO: does this take too long?
  }

  if (this->enqueued_command_ == NULL) {
    return;
  }

  ESP_LOGD(TAG, "NEW COMMAND ABOUT TO BE WRITTEN!");

  // if a command is pending, ensure the bus being awake: wakeup, wait & start over if asleep
  if (!this->wakeup_bus_()) {
    ESP_LOGD(TAG, " bus was not awake");
    delay(5);
    return;
  }

  delay(5);
  ESP_LOGD(TAG, " bus is awake & ready now");

  // send enqueued command
  if (millis() - this->last_rx_msg_ >= 30) {
    // ESP_LOGD(TAG, "Waiting for correct time slot for sending command.");
    return;
  }
  ESP_LOGD(TAG, "NEW CMD: in correct time slot for sending command, do so");
  // enque command only if we are just in a silent slot, max 40ms after end of last msg OR when bus is idle/silent
  // for
  while (millis() - this->last_rx_msg_ <= 16) {
    delay(1);
  }
  uint32_t startt = millis();
  this->write_array(this->enqueued_command_, TX_MSG_LEN);
  this->flush();

  ESP_LOGD(TAG, "Wrote command %s to serial..duration: %lu ms, offset after receive %lu",
           print_command(this->enqueued_command_), millis() - startt, startt - this->last_rx_msg_);

  this->enqueued_command_ = NULL;
}

void MarantecOpener::control(const CoverCall &call) {
  if (call.get_stop()) {
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->enqueue_command_(COMMAND_STOP);
      external_cmd = false;
    }

  } else if (call.get_toggle().has_value()) {
    // toggle action logic: OPEN - STOP - CLOSE

    if (this->current_operation != COVER_OPERATION_IDLE) {
      twostepaction = true;
      this->enqueue_command_(COMMAND_STOP);
    } else {
      // motor was idle look back to last state
      if (this->previous_operation == COVER_OPERATION_OPENING) {
        this->enqueue_command_(COMMAND_CLOSE);
      } else if (this->previous_operation == COVER_OPERATION_CLOSING) {
        this->enqueue_command_(COMMAND_OPEN);
      }
    }
    external_cmd = false;

  } else if (call.get_position().has_value()) {
    // TODO: check if cover is moving/not idle, in that case trigger a two step action
    //  go to position action
    auto pos = *call.get_position();
    ESP_LOGD(TAG, "internal position request: %.1f -> %.1f ", this->position, pos);
    // are we at the target?
    if (abs(pos - this->position) >= 0.1) {
      this->target_position_ = pos;
      if (this->current_operation == COVER_OPERATION_IDLE) {
        if (pos < this->position) {
          this->enqueue_command_(COMMAND_CLOSE);
        } else {
          this->enqueue_command_(COMMAND_OPEN);
        }
      } else {
        twostepaction = true;
        this->enqueue_command_(COMMAND_STOP);
      }
      external_cmd = false;
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

void MarantecOpener::enqueue_command_(const uint8_t *cmd) {
  this->enqueued_command_ = cmd;
  ESP_LOGD(TAG, "command %s was enqueued ", print_command(this->enqueued_command_));
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
