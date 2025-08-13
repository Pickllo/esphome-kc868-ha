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
void KC868HaComponent::update() {
}
void KC868HaComponent::loop() {
  const uint8_t FRAME_START_BYTE = 0x01;
  const size_t FRAME_LENGTH = 21;

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
  }
  while (!this->rx_buffer_.empty()) {
    if (this->rx_buffer_[0] != FRAME_START_BYTE) {
      ESP_LOGD(TAG, "Discarding foreign byte: 0x%02X", this->rx_buffer_[0]);
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue; 
    }

    if (this->rx_buffer_.size() < FRAME_LENGTH) {
      ESP_LOGD(TAG, "Found start byte, but waiting for full frame. Have %d of %d bytes.", this->rx_buffer_.size(), FRAME_LENGTH);
      break; 
    }
    uint8_t* frame_data = this->rx_buffer_.data();
    uint8_t crc_data[19];
    memcpy(crc_data, frame_data, 19);

    uint16_t calculated_crc = this->crc16(crc_data, 19);
    uint8_t calc_lo = static_cast<uint8_t>(calculated_crc & 0x00FF);
    uint8_t calc_hi = static_cast<uint8_t>((calculated_crc >> 8) & 0xFF);

    if (frame_data[19] == calc_lo && frame_data[20] == calc_hi) {
      ESP_LOGD(TAG, "CRC match! Received valid KC868 frame: %s", this->format_uart_data_(frame_data, FRAME_LENGTH));
      this->handle_frame_(frame_data, FRAME_LENGTH);
      
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + FRAME_LENGTH);
    } else {
      ESP_LOGD(TAG, "CRC Mismatch on potential frame. Discarding start byte.");
      this->rx_buffer_.erase(this->rx_buffer_.begin());
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

void KC868HaComponent::send_command(const uint8_t* data, size_t len) {
    ESP_LOGD(TAG, "Sending command: %s", this->format_uart_data_(data, len));
    
    this->pin_tx_->digital_write(true);

    this->write_array(data, len);
    this->flush();

    this->pin_tx_->digital_write(false);
}
}

} // namespace kc868_ha
} // namespace esphome
