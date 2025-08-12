#include "kc868_ha_component.h"
#include "esphome/core/log.h"
#include <cstring> 
#include <cstdio>  

namespace esphome {
namespace kc868_ha {

static const char *const TAG = "kc868_ha";

char *KC868HaComponent::format_uart_data_(const uint8_t *uart_data, int length) {
  static char str[256] = {0};
  char tmp[10];
  str[0] = '\0';
  for (int i = 0; i < length; i++) {
    sprintf(tmp, "%02X:", uart_data[i]);
    strncat(str, tmp, sizeof(str) - strlen(str) - 1);
  }
  if (length > 0) {
    str[strlen(str) - 1] = '\0';
  }
  return str;
}

uint16_t KC868HaComponent::crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 8; j > 0; j--) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void KC868HaComponent::send_command(const uint8_t* data, size_t len) {
    this->write_array(data, len);
    ESP_LOGD(TAG, "KC868-HA Sent: %s", this->format_uart_data_(data, len));
}

void KC868HaComponent::setup() { ESP_LOGD(TAG, "KC868HaComponent::setup"); }

void KC868HaComponent::loop() {
  // 1. Read all available bytes into the buffer
  uint8_t byte;
  while (this->available()) {
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
  }

  // 2. Try to find a valid 21-byte frame in the buffer
  while (this->rx_buffer_.size() >= 21) {
    uint8_t* frame_start = this->rx_buffer_.data();
    uint8_t crc_data[19];
    memcpy(crc_data, frame_start, 19);

    uint16_t calculated_crc = this->crc16(crc_data, 19);
    uint8_t calc_lo = static_cast<uint8_t>(calculated_crc & 0x00FF);
    uint8_t calc_hi = static_cast<uint8_t>((calculated_crc >> 8) & 0xFF);

    ESP_LOGD(TAG, "CRC Check: Recv=[%02X %02X], Calc=[%02X %02X]", frame_start[19], frame_start[20], calc_lo, calc_hi);

    // 3. Check for CRC match. Your logs confirm the order is Low Byte then High Byte.
    if (frame_start[19] == calc_lo && frame_start[20] == calc_hi) {
      // CRC MATCH! This is a valid frame.
      ESP_LOGD(TAG, "KC868-HA Received: %s", this->format_uart_data_(frame_start, 21));
      this->handle_frame_(frame_start, 21);
      
      // Remove the processed frame from the buffer
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + 21);
      
      continue;
    }

    // 4. CRC MISMATCH. Discard the first byte and slide the window forward.
    this->rx_buffer_.erase(this->rx_buffer_.begin());
  }
}

void KC868HaComponent::handle_frame_(const uint8_t *frame, size_t len) {
  if (len < 21) return;

  for (auto *sensor : this->binary_sensors_) {
    if (sensor->get_target_relay_controller_addr() == frame[0] &&
        sensor->get_switch_adapter_addr() == frame[3]) {
      for (int i = 7; i <= 17; i += 2) {
        if (frame[i] == (sensor->get_bind_output() + 100)) {
          sensor->publish_state(frame[i + 1] == 1);
        }
      }
    }
  }
}

void KC868HaComponent::dump_config() { ESP_LOGCONFIG(TAG, "KC868HaComponent::dump_config"); }

const std::vector<KC868HaSwitch *>& KC868HaSwitch::get_all_switches() {
    return this->parent_->get_switches();
}

void KC868HaBinarySensor::setup() { this->publish_initial_state(false); }
void KC868HaBinarySensor::dump_config() { LOG_BINARY_SENSOR("", "KC868-HA Binary Sensor", this); }

void KC868HaSwitch::setup() {
  auto restored = this->get_initial_state_with_restore_mode();
  if (restored.has_value()) {
      this->write_state(*restored);
  }
}

void KC868HaSwitch::dump_config() { LOG_SWITCH("", "KC868-HA Switch", this); }

void KC868HaSwitch::write_state(bool state) {
  uint8_t data_payload[21] = {this->get_target_relay_controller_addr(), 0x03, 0x12, 0x55, 0xBB,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  int channel = this->get_bind_output();
  int byte_index = (channel - 1) / 8;
  int bit_position = (channel - 1) % 8;
  if (state) {
    data_payload[20 - byte_index] |= (1 << bit_position);
  }

  for (auto *other_switch : this->get_all_switches()) {
    if (other_switch == this) continue;
    if (other_switch->get_target_relay_controller_addr() != this->get_target_relay_controller_addr()) continue;
    
    int other_channel = other_switch->get_bind_output();
    int other_byte_index = (other_channel - 1) / 8;
    int other_bit_position = (other_channel - 1) % 8;
    if (other_switch->state) {
      data_payload[20 - other_byte_index] |= (1 << other_bit_position);
    }
  }

  uint16_t crc = this->parent_->crc16(data_payload, sizeof(data_payload));
  uint8_t final_frame[23];
  memcpy(final_frame, data_payload, sizeof(data_payload));
  final_frame[21] = static_cast<uint8_t>(crc & 0x00FF);
  final_frame[22] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  this->parent_->send_command(final_frame, sizeof(final_frame));
  this->publish_state(state);
}

} // namespace kc868_ha
} // namespace esphome
