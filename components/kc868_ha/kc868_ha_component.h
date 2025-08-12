#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include <vector>

namespace esphome {
namespace kc868_ha {

class KC868HaComponent;

class KC868HaBinarySensor : public Component, public binary_sensor::BinarySensor {
 public:
  void set_target_relay_controller_addr(uint8_t addr) { this->target_relay_controller_addr_ = addr; }
  uint8_t get_target_relay_controller_addr() { return this->target_relay_controller_addr_; }
  void set_switch_adapter_addr(uint8_t addr) { this->switch_adapter_addr_ = addr; }
  uint8_t get_switch_adapter_addr() { return this->switch_adapter_addr_; }
  void set_bind_output(uint8_t bind_output) { this->bind_output_ = bind_output; }
  uint8_t get_bind_output() { return this->bind_output_; }
  void setup() override;
  void dump_config() override;

 protected:
  uint8_t target_relay_controller_addr_;
  uint8_t switch_adapter_addr_;
  uint8_t bind_output_;
};

class KC868HaSwitch : public Component, public switch_::Switch {
 public:
  void set_parent(KC868HaComponent *parent) { this->parent_ = parent; }
  void set_target_relay_controller_addr(uint8_t addr) { this->target_relay_controller_addr_ = addr; }
  uint8_t get_target_relay_controller_addr() { return this->target_relay_controller_addr_; }
  void set_bind_output(uint8_t bind_output) { this->bind_output_ = bind_output; }
  uint8_t get_bind_output() { return this->bind_output_; }

  void setup() override;
  void write_state(bool state) override;
  void dump_config() override;
  
  const std::vector<KC868HaSwitch *>& get_all_switches();

 protected:
  KC868HaComponent *parent_;
  uint8_t target_relay_controller_addr_;
  uint8_t bind_output_;
};

class KC868HaComponent : public Component, public uart::UARTDevice {
 public:
  // THIS IS THE CORRECTED CONSTRUCTOR
  KC868HaComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void register_binary_sensor(KC868HaBinarySensor *obj) { this->binary_sensors_.push_back(obj); }
  void register_switch(KC868HaSwitch *obj) {
    obj->set_parent(this);
    this->switches_.push_back(obj);
  };
  
  const std::vector<KC868HaSwitch *>& get_switches() { return this->switches_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  
  void send_command(const uint8_t* data, size_t len);
  uint16_t crc16(const uint8_t *data, uint8_t length);

 private:
  char *format_uart_data_(const uint8_t *uart_data, int length);
  void handle_frame_(const uint8_t *frame, size_t len);

  std::vector<uint8_t> rx_buffer_; // Buffer for incoming serial data
  std::vector<KC868HaBinarySensor *> binary_sensors_;
  std::vector<KC868HaSwitch *> switches_;
};

} // namespace kc868_ha
} // namespace esphome
