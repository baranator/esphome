#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"

namespace esphome {
namespace marantec_opener {

class MarantecOpener : public cover::Cover, public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // void set_open_duration(uint32_t duration) { this->open_duration_ = duration; }
  // void set_close_duration(uint32_t duration) { this->close_duration_ = duration; }

  cover::CoverTraits get_traits() override;

 protected:
  void update_();
  void control(const cover::CoverCall &call) override;
  bool is_at_target_() const;
  void enqueue_command_(cover::CoverOperation dir);
  void endstop_reached_(cover::CoverOperation operation);
  void recompute_position_();
  void set_current_operation_(cover::CoverOperation operation, float pos);
  void process_rx_(uint8_t *data);

  unsigned open_duration_{20000};
  unsigned close_duration_{20000};
  unsigned toggles_needed_{0};
  // cover::CoverOperation next_direction_{cover::COVER_OPERATION_IDLE};
  cover::CoverOperation last_command_{cover::COVER_OPERATION_IDLE};

  uint32_t last_recompute_time_{0};
  uint32_t start_dir_time_{0};
  float target_position_{0};
  bool query_seen_{};
  uint8_t counter_{};

  uint32_t last_rx_msg_{0};
  uint32_t last_wakeup_call_{0};
  bool read_next_msg(uint8_t *data);
  bool read_next_msg(uint8_t *data, uint16_t timeout_ms);
  cover::CoverOperation enqueued_command_{cover::COVER_OPERATION_IDLE};
  cover::CoverOperation previous_operation{cover::COVER_OPERATION_IDLE};

  void wakeup_bus_();
};

}  // namespace marantec_opener
}  // namespace esphome
