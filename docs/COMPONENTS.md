# Component Interfaces and Implementation

This document provides detailed documentation of all component interfaces, classes, and their implementations in the ESPHome Emerson Vertiv R48 integration.

## Table of Contents

- [Component Overview](#component-overview)
- [EmersonR48Component](#emersonr48component)
- [EmersonR48Switch](#emersonr48switch)
- [EmersonR48Number](#emersonr48number)
- [EmersonR48Button](#emersonr48button)
- [MCP2515 CAN Controller](#mcp2515-can-controller)
- [Python Configuration Modules](#python-configuration-modules)

---

## Component Overview

### Component File Structure

```
components/
├── emerson_r48/
│   ├── __init__.py           # Main component schema and code generation
│   ├── emerson_r48.h         # EmersonR48Component header
│   ├── emerson_r48.cpp       # EmersonR48Component implementation
│   ├── sensor.py             # Sensor platform schema
│   ├── button/
│   │   ├── __init__.py       # Button platform schema
│   │   ├── emerson_r48_button.h
│   │   └── emerson_r48_button.cpp
│   ├── switch/
│   │   ├── __init__.py       # Switch platform schema
│   │   ├── switch.py         # Additional switch config
│   │   ├── emerson_switch.h
│   │   ├── emerson_switch.cpp
│   │   ├── empty_switch.h    # Template/empty switch
│   │   └── empty_switch.cpp
│   └── number/
│       ├── __init__.py       # Number platform schema
│       ├── emerson_r48_number.h
│       └── emerson_r48_number.cpp
└── mcp2515/
    ├── __init__.py           # MCP2515 component schema
    ├── canbus.py             # CAN bus integration
    ├── mcp2515.h             # MCP2515 header
    ├── mcp2515.cpp           # MCP2515 implementation
    └── mcp2515_defs.h        # Register definitions
```

### Inheritance Hierarchy

```
┌────────────────────────────────────────────────────────────────────────┐
│                         ESPHome Base Classes                            │
└────────────────────────────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────┐          ┌───────────────┐          ┌───────────────┐
│ Polling       │          │ switch_::     │          │ number::      │
│ Component     │          │ Switch        │          │ Number        │
└───────┬───────┘          └───────┬───────┘          └───────┬───────┘
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐          ┌───────────────┐          ┌───────────────┐
│ EmersonR48    │          │ EmersonR48    │          │ EmersonR48    │
│ Component     │          │ Switch        │          │ Number        │
└───────────────┘          └───────────────┘          └───────────────┘

┌───────────────┐          ┌───────────────────────────────────────────┐
│ button::      │          │ canbus::Canbus + spi::SPIDevice           │
│ Button        │          │                                           │
└───────┬───────┘          └─────────────────┬─────────────────────────┘
        │                                    │
        ▼                                    ▼
┌───────────────┐                   ┌───────────────┐
│ EmersonR48    │                   │   MCP2515     │
│ Button        │                   │               │
└───────────────┘                   └───────────────┘
```

---

## EmersonR48Component

### Header File: `emerson_r48.h`

**Location:** `components/emerson_r48/emerson_r48.h`

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/canbus/canbus.h"

namespace esphome {
namespace emerson_r48 {

class EmersonR48Component : public PollingComponent {
 public:
  // Constructor
  EmersonR48Component(canbus::Canbus *canbus);

  // ESPHome lifecycle methods
  void setup() override;
  void update() override;
  float get_setup_priority() const override;

  // Sensor setters (called from Python code generation)
  void set_input_voltage_sensor(sensor::Sensor *sensor);
  void set_input_frequency_sensor(sensor::Sensor *sensor);
  void set_input_current_sensor(sensor::Sensor *sensor);
  void set_input_power_sensor(sensor::Sensor *sensor);
  void set_input_temp_sensor(sensor::Sensor *sensor);
  void set_efficiency_sensor(sensor::Sensor *sensor);
  void set_output_voltage_sensor(sensor::Sensor *sensor);
  void set_output_current_sensor(sensor::Sensor *sensor);
  void set_max_output_current_sensor(sensor::Sensor *sensor);
  void set_output_power_sensor(sensor::Sensor *sensor);
  void set_output_temp_sensor(sensor::Sensor *sensor);

  // Control methods (public for child components)
  void set_output_voltage(float value, bool offline = false);
  void set_max_output_current(float value, bool offline = false);
  void set_max_input_current(float value);
  void set_offline_values();
  void set_control(uint8_t msgv);

  // CAN communication methods
  void sendSync();
  void sendSync2();
  void gimme5();
  void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);

  // Control state variables (public for switch access)
  bool dcOff_{false};
  bool fanFull_{false};
  bool flashLed_{false};
  bool acOff_{false};

 protected:
  // CAN bus reference
  canbus::Canbus *canbus_;

  // Sensor pointers
  sensor::Sensor *input_voltage_sensor_{nullptr};
  sensor::Sensor *input_frequency_sensor_{nullptr};
  sensor::Sensor *input_current_sensor_{nullptr};
  sensor::Sensor *input_power_sensor_{nullptr};
  sensor::Sensor *input_temp_sensor_{nullptr};
  sensor::Sensor *efficiency_sensor_{nullptr};
  sensor::Sensor *output_voltage_sensor_{nullptr};
  sensor::Sensor *output_current_sensor_{nullptr};
  sensor::Sensor *max_output_current_sensor_{nullptr};
  sensor::Sensor *output_power_sensor_{nullptr};
  sensor::Sensor *output_temp_sensor_{nullptr};

  // State tracking
  uint32_t lastUpdate_{0};
  uint8_t intervalCount_{0};

  // Helper methods
  void publish_sensor_state_(sensor::Sensor *sensor, float value);
  void float_to_bytearray(float value, uint8_t *bytes);
};

}  // namespace emerson_r48
}  // namespace esphome
```

### Implementation: `emerson_r48.cpp`

**Location:** `components/emerson_r48/emerson_r48.cpp`

#### Constants and Definitions

```cpp
// R48-3000e3 specifications
static const float EMR48_OUTPUT_VOLTAGE_MIN = 41.0;
static const float EMR48_OUTPUT_VOLTAGE_MAX = 58.5;
static const float EMR48_OUTPUT_CURRENT_RATED_VALUE = 62.5;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN = 10;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX = 121;
static const float EMR48_OUTPUT_CURRENT_MIN = 5.5;
static const float EMR48_OUTPUT_CURRENT_MAX = 62.5;

// CAN Message IDs
static const uint32_t CAN_ID_REQUEST = 0x06000783;
static const uint32_t CAN_ID_DATA = 0x60f8003;
static const uint32_t CAN_ID_DATA2 = 0x60f8007;
static const uint32_t CAN_ID_SET = 0x0607FF83;
static const uint32_t CAN_ID_SET2 = 0x0677FF83;
static const uint32_t CAN_ID_SET_CTL = 0x06080783;
static const uint32_t CAN_ID_SYNC = 0x0707FF83;
static const uint32_t CAN_ID_SYNC2 = 0x0717FF83;
static const uint32_t CAN_ID_GIMME5 = 0x06080783;

// Parameter codes
static const uint8_t EMR48_DATA_OUTPUT_V = 0x01;
static const uint8_t EMR48_DATA_OUTPUT_A = 0x02;
static const uint8_t EMR48_DATA_OUTPUT_AL = 0x03;
static const uint8_t EMR48_DATA_OUTPUT_T = 0x04;
static const uint8_t EMR48_DATA_OUTPUT_IV = 0x05;
```

#### Constructor

```cpp
EmersonR48Component::EmersonR48Component(canbus::Canbus *canbus)
    : canbus_(canbus) {}
```

#### setup() Method

```cpp
void EmersonR48Component::setup() {
  // Create CAN trigger for all messages
  auto trigger = new canbus::CanbusTrigger(this->canbus_, 0, 0, true);
  trigger->set_component_source("emerson_r48");
  App.register_component(trigger);

  // Create automation with callback
  auto automation = new Automation<uint32_t, bool, std::vector<uint8_t> &>(trigger);

  // Lambda callback to on_frame()
  auto lambdaaction = new LambdaAction<uint32_t, bool, std::vector<uint8_t> &>(
      [this](uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
        this->on_frame(can_id, rtr, data);
      });

  automation->add_actions({lambdaaction});

  // Initial sync
  this->sendSync();
  this->gimme5();
}
```

#### update() Method

```cpp
void EmersonR48Component::update() {
  this->intervalCount_ = (this->intervalCount_ + 1) % 6;

  std::vector<uint8_t> data;

  switch (this->intervalCount_) {
    case 0:  // Request output voltage
      data = {0x01, 0xF0, 0x00, EMR48_DATA_OUTPUT_V, 0x00, 0x00, 0x00, 0x00};
      this->canbus_->send(CAN_ID_REQUEST, true, data);
      break;

    case 1:  // Request output current
      data = {0x01, 0xF0, 0x00, EMR48_DATA_OUTPUT_A, 0x00, 0x00, 0x00, 0x00};
      this->canbus_->send(CAN_ID_REQUEST, true, data);
      break;

    case 2:  // Request current limit
      data = {0x01, 0xF0, 0x00, EMR48_DATA_OUTPUT_AL, 0x00, 0x00, 0x00, 0x00};
      this->canbus_->send(CAN_ID_REQUEST, true, data);
      break;

    case 3:  // Request temperature
      data = {0x01, 0xF0, 0x00, EMR48_DATA_OUTPUT_T, 0x00, 0x00, 0x00, 0x00};
      this->canbus_->send(CAN_ID_REQUEST, true, data);
      break;

    case 4:  // Request input voltage
      data = {0x01, 0xF0, 0x00, EMR48_DATA_OUTPUT_IV, 0x00, 0x00, 0x00, 0x00};
      this->canbus_->send(CAN_ID_REQUEST, true, data);
      break;

    case 5:  // Send control message every ~10s
      {
        uint8_t msgv = (this->dcOff_ << 7) |
                       (this->fanFull_ << 4) |
                       (this->flashLed_ << 3) |
                       (this->acOff_ << 2) | 1;
        this->set_control(msgv);
      }
      break;
  }

  // Timeout check
  uint32_t now = millis();
  if ((now - this->lastUpdate_) > (5 * this->get_update_interval())) {
    // Publish NaN to indicate disconnection
    this->publish_sensor_state_(this->output_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->output_current_sensor_, NAN);
    this->publish_sensor_state_(this->output_temp_sensor_, NAN);
    this->publish_sensor_state_(this->input_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->max_output_current_sensor_, NAN);

    // Attempt recovery
    this->sendSync();
    this->gimme5();
  }
}
```

#### set_output_voltage() Method

```cpp
void EmersonR48Component::set_output_voltage(float value, bool offline) {
  // Clamp value to valid range
  if (value < EMR48_OUTPUT_VOLTAGE_MIN) value = EMR48_OUTPUT_VOLTAGE_MIN;
  if (value > EMR48_OUTPUT_VOLTAGE_MAX) value = EMR48_OUTPUT_VOLTAGE_MAX;

  // Convert float to big-endian bytes
  uint8_t bytes[4];
  this->float_to_bytearray(value, bytes);

  // Parameter: 0x21 = online (RAM), 0x24 = offline (EEPROM)
  uint8_t param = offline ? 0x24 : 0x21;

  std::vector<uint8_t> data = {0x03, 0xF0, 0x00, param,
                                bytes[0], bytes[1], bytes[2], bytes[3]};
  this->canbus_->send(CAN_ID_SET, true, data);
}
```

#### set_max_output_current() Method

```cpp
void EmersonR48Component::set_max_output_current(float value, bool offline) {
  // Clamp to valid percentage range
  if (value < EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN)
    value = EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN;
  if (value > EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX)
    value = EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX;

  // Convert percentage to ratio (e.g., 50% -> 0.50)
  float ratio = value / 100.0;

  uint8_t bytes[4];
  this->float_to_bytearray(ratio, bytes);

  // Parameter: 0x22 = online (RAM), 0x19 = offline (EEPROM)
  uint8_t param = offline ? 0x19 : 0x22;

  std::vector<uint8_t> data = {0x03, 0xF0, 0x00, param,
                                bytes[0], bytes[1], bytes[2], bytes[3]};
  this->canbus_->send(CAN_ID_SET, true, data);
}
```

#### on_frame() Method

```cpp
void EmersonR48Component::on_frame(uint32_t can_id, bool rtr,
                                    std::vector<uint8_t> &data) {
  // Log received frame for debugging
  ESP_LOGD("emerson_r48", "Received CAN frame: ID=0x%08X, Data=[...]", can_id);

  // Only process data frames
  if (can_id != CAN_ID_DATA) return;

  // Extract float value from bytes 4-7 (big-endian)
  uint32_t raw = ((uint32_t)data[4] << 24) |
                 ((uint32_t)data[5] << 16) |
                 ((uint32_t)data[6] << 8) |
                 (uint32_t)data[7];

  float value;
  memcpy(&value, &raw, sizeof(value));

  // Route to appropriate sensor based on parameter code
  uint8_t param = data[3];

  switch (param) {
    case EMR48_DATA_OUTPUT_V:  // 0x01
      this->publish_sensor_state_(this->output_voltage_sensor_, value);
      break;

    case EMR48_DATA_OUTPUT_A:  // 0x02
      this->publish_sensor_state_(this->output_current_sensor_, value);
      break;

    case EMR48_DATA_OUTPUT_AL:  // 0x03
      // Convert ratio back to percentage
      this->publish_sensor_state_(this->max_output_current_sensor_, value * 100);
      break;

    case EMR48_DATA_OUTPUT_T:  // 0x04
      this->publish_sensor_state_(this->output_temp_sensor_, value);
      break;

    case EMR48_DATA_OUTPUT_IV:  // 0x05
      this->publish_sensor_state_(this->input_voltage_sensor_, value);
      // Update timestamp on final parameter
      this->lastUpdate_ = millis();
      break;
  }
}
```

#### float_to_bytearray() Helper

```cpp
void EmersonR48Component::float_to_bytearray(float value, uint8_t *bytes) {
  uint32_t temp;
  memcpy(&temp, &value, sizeof(temp));

  // Big-endian byte order
  bytes[0] = (temp >> 24) & 0xFF;  // MSB
  bytes[1] = (temp >> 16) & 0xFF;
  bytes[2] = (temp >> 8) & 0xFF;
  bytes[3] = temp & 0xFF;          // LSB
}
```

---

## EmersonR48Switch

### Header File: `emerson_switch.h`

**Location:** `components/emerson_r48/switch/emerson_switch.h`

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../emerson_r48.h"

namespace esphome {
namespace emerson_r48 {

// Function code definitions
static const int8_t SET_AC_FUNCTION = 0x0;
static const int8_t SET_DC_FUNCTION = 0x1;
static const int8_t SET_FAN_FUNCTION = 0x2;
static const int8_t SET_LED_FUNCTION = 0x3;

class EmersonR48Switch : public switch_::Switch, public Component {
 public:
  void set_parent(EmersonR48Component *parent, int8_t functionCode);
  void dump_config() override;

 protected:
  void write_state(bool state) override;

  EmersonR48Component *parent_{nullptr};
  int8_t functionCode_{0};
};

}  // namespace emerson_r48
}  // namespace esphome
```

### Implementation: `emerson_switch.cpp`

**Location:** `components/emerson_r48/switch/emerson_switch.cpp`

```cpp
#include "emerson_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const char *TAG = "emerson_r48.switch";

void EmersonR48Switch::set_parent(EmersonR48Component *parent,
                                   int8_t functionCode) {
  this->parent_ = parent;
  this->functionCode_ = functionCode;
}

void EmersonR48Switch::dump_config() {
  LOG_SWITCH("", "EmersonR48 Switch", this);
  ESP_LOGCONFIG(TAG, "  Function Code: 0x%02X", this->functionCode_);
}

void EmersonR48Switch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent not set!");
    return;
  }

  // Update parent state based on function code
  switch (this->functionCode_) {
    case SET_AC_FUNCTION:  // 0x0
      this->parent_->acOff_ = state;
      break;

    case SET_DC_FUNCTION:  // 0x1
      this->parent_->dcOff_ = state;
      break;

    case SET_FAN_FUNCTION:  // 0x2
      this->parent_->fanFull_ = state;
      break;

    case SET_LED_FUNCTION:  // 0x3
      this->parent_->flashLed_ = state;
      break;

    default:
      ESP_LOGW(TAG, "Unknown function code: 0x%02X", this->functionCode_);
      return;
  }

  // Build and send control message
  uint8_t msgv = (this->parent_->dcOff_ << 7) |
                 (this->parent_->fanFull_ << 4) |
                 (this->parent_->flashLed_ << 3) |
                 (this->parent_->acOff_ << 2) |
                 1;

  this->parent_->set_control(msgv);

  // Publish new state
  this->publish_state(state);
}

}  // namespace emerson_r48
}  // namespace esphome
```

### Switch Function Mapping

| Switch Name | Function Code | Parent Variable | Description |
|-------------|---------------|-----------------|-------------|
| `ac_sw` | `0x0` | `acOff_` | AC Input Control |
| `dc_sw` | `0x1` | `dcOff_` | DC Output Control |
| `fan_sw` | `0x2` | `fanFull_` | Fan Speed Control |
| `led_sw` | `0x3` | `flashLed_` | LED Flash Control |

---

## EmersonR48Number

### Header File: `emerson_r48_number.h`

**Location:** `components/emerson_r48/number/emerson_r48_number.h`

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../emerson_r48.h"

namespace esphome {
namespace emerson_r48 {

// Function code definitions
static const int8_t SET_VOLTAGE_FUNCTION = 0x0;
static const int8_t SET_CURRENT_FUNCTION = 0x3;
static const int8_t SET_INPUT_CURRENT_FUNCTION = 0x4;

class EmersonR48Number : public number::Number, public Component {
 public:
  void set_parent(EmersonR48Component *parent, int8_t functionCode);
  void dump_config() override;

 protected:
  void control(float value) override;

  EmersonR48Component *parent_{nullptr};
  int8_t functionCode_{0};
};

}  // namespace emerson_r48
}  // namespace esphome
```

### Implementation: `emerson_r48_number.cpp`

**Location:** `components/emerson_r48/number/emerson_r48_number.cpp`

```cpp
#include "emerson_r48_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const char *TAG = "emerson_r48.number";

void EmersonR48Number::set_parent(EmersonR48Component *parent,
                                   int8_t functionCode) {
  this->parent_ = parent;
  this->functionCode_ = functionCode;
}

void EmersonR48Number::dump_config() {
  LOG_NUMBER("", "EmersonR48 Number", this);
  ESP_LOGCONFIG(TAG, "  Function Code: 0x%02X", this->functionCode_);
}

void EmersonR48Number::control(float value) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent not set!");
    return;
  }

  // Route to appropriate parent method
  switch (this->functionCode_) {
    case SET_VOLTAGE_FUNCTION:  // 0x0
      this->parent_->set_output_voltage(value);
      break;

    case SET_CURRENT_FUNCTION:  // 0x3
      this->parent_->set_max_output_current(value);
      break;

    case SET_INPUT_CURRENT_FUNCTION:  // 0x4
      this->parent_->set_max_input_current(value);
      break;

    default:
      ESP_LOGW(TAG, "Unknown function code: 0x%02X", this->functionCode_);
      return;
  }

  // Publish new value
  this->publish_state(value);
}

}  // namespace emerson_r48
}  // namespace esphome
```

### Number Parameter Mapping

| Number Name | Function Code | Range | Unit | Method |
|-------------|---------------|-------|------|--------|
| `output_voltage` | `0x0` | 41.0 - 58.5 | V | `set_output_voltage()` |
| `max_output_current` | `0x3` | 10 - 121 | % | `set_max_output_current()` |
| `max_input_current` | `0x4` | 0 - 20 | A | `set_max_input_current()` |

---

## EmersonR48Button

### Header File: `emerson_r48_button.h`

**Location:** `components/emerson_r48/button/emerson_r48_button.h`

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "../emerson_r48.h"

namespace esphome {
namespace emerson_r48 {

class EmersonR48Button : public button::Button, public Component {
 public:
  void set_parent(EmersonR48Component *parent);
  void dump_config() override;

 protected:
  void press_action() override;

  EmersonR48Component *parent_{nullptr};
};

}  // namespace emerson_r48
}  // namespace esphome
```

### Implementation: `emerson_r48_button.cpp`

**Location:** `components/emerson_r48/button/emerson_r48_button.cpp`

```cpp
#include "emerson_r48_button.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const char *TAG = "emerson_r48.button";

void EmersonR48Button::set_parent(EmersonR48Component *parent) {
  this->parent_ = parent;
}

void EmersonR48Button::dump_config() {
  LOG_BUTTON("", "EmersonR48 Button", this);
}

void EmersonR48Button::press_action() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent not set!");
    return;
  }

  ESP_LOGI(TAG, "Setting offline values...");
  this->parent_->set_offline_values();
}

}  // namespace emerson_r48
}  // namespace esphome
```

### Button Purpose

The `set_offline_values` button triggers permanent storage of current settings:

1. Reads current voltage/current values from numbers
2. Calls `set_output_voltage(value, offline=true)`
3. Calls `set_max_output_current(value, offline=true)`
4. Values are stored in EEPROM and survive power cycles

---

## MCP2515 CAN Controller

### Header File: `mcp2515.h`

**Location:** `components/mcp2515/mcp2515.h`

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace mcp2515 {

enum CanClock { MCP_8MHZ, MCP_12MHZ, MCP_16MHZ, MCP_20MHZ };
enum CanSpeed { CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS };
enum CanMode { NORMAL, LOOPBACK, LISTENONLY };

class MCP2515 : public canbus::Canbus,
                public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                      spi::CLOCK_POLARITY_LOW,
                                      spi::CLOCK_PHASE_LEADING,
                                      spi::DATA_RATE_8MHZ> {
 public:
  MCP2515();

  // Configuration setters
  void set_clock(CanClock clock) { this->clock_ = clock; }
  void set_mode(CanMode mode) { this->mode_ = mode; }

  // ESPHome lifecycle
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Canbus interface implementation
  canbus::Error send_message(uint32_t can_id, bool ext_id,
                             const std::vector<uint8_t> &data) override;
  canbus::Error read_message(uint32_t *can_id, bool *ext_id,
                             std::vector<uint8_t> *data) override;

 protected:
  // Internal methods
  void reset_();
  uint8_t read_register_(uint8_t reg);
  void write_register_(uint8_t reg, uint8_t value);
  void modify_register_(uint8_t reg, uint8_t mask, uint8_t value);
  uint8_t get_status_();
  bool set_mode_(uint8_t mode);
  bool set_bitrate_(CanSpeed speed, CanClock clock);

  CanClock clock_{MCP_8MHZ};
  CanMode mode_{NORMAL};
};

}  // namespace mcp2515
}  // namespace esphome
```

### Key Implementation Details

**SPI Communication:**
```cpp
uint8_t MCP2515::read_register_(uint8_t reg) {
  this->enable();
  this->transfer_byte(MCP_READ);
  this->transfer_byte(reg);
  uint8_t value = this->transfer_byte(0x00);
  this->disable();
  return value;
}

void MCP2515::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->transfer_byte(MCP_WRITE);
  this->transfer_byte(reg);
  this->transfer_byte(value);
  this->disable();
}
```

**Sending CAN Messages:**
```cpp
canbus::Error MCP2515::send_message(uint32_t can_id, bool ext_id,
                                     const std::vector<uint8_t> &data) {
  // Find free TX buffer
  uint8_t status = this->get_status_();
  uint8_t txbuf;

  if (!(status & 0x04))      txbuf = 0;  // TXB0 free
  else if (!(status & 0x10)) txbuf = 1;  // TXB1 free
  else if (!(status & 0x40)) txbuf = 2;  // TXB2 free
  else return canbus::ERROR_ALLTXBUSY;

  // Load ID into buffer
  // ... ID encoding for standard/extended ...

  // Load data
  for (size_t i = 0; i < data.size() && i < 8; i++) {
    this->write_register_(TXB_BASE + txbuf * 0x10 + 5 + i, data[i]);
  }

  // Request transmission
  this->write_register_(TXB_BASE + txbuf * 0x10 + TXBCTRL, 0x08);

  return canbus::ERROR_OK;
}
```

---

## Python Configuration Modules

### Main Component: `__init__.py`

**Location:** `components/emerson_r48/__init__.py`

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import canbus
from esphome.const import CONF_ID

DEPENDENCIES = ["canbus"]
CODEOWNERS = ["@arnoutzw"]

CONF_CANBUS_ID = "canbus_id"
CONF_EMERSON_R48_ID = "emerson_r48_id"

emerson_r48_ns = cg.esphome_ns.namespace("emerson_r48")
EmersonR48Component = emerson_r48_ns.class_(
    "EmersonR48Component", cg.PollingComponent
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EmersonR48Component),
        cv.Required(CONF_CANBUS_ID): cv.use_id(canbus.CanbusComponent),
    }
).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    canbus_component = await cg.get_variable(config[CONF_CANBUS_ID])
    var = cg.new_Pvariable(config[CONF_ID], canbus_component)
    await cg.register_component(var, config)
```

### Sensor Platform: `sensor.py`

**Location:** `components/emerson_r48/sensor.py`

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_PERCENT,
)
from . import CONF_EMERSON_R48_ID, EmersonR48Component

# Sensor type definitions
CONF_INPUT_VOLTAGE = "input_voltage"
CONF_INPUT_FREQUENCY = "input_frequency"
CONF_INPUT_CURRENT = "input_current"
CONF_INPUT_POWER = "input_power"
CONF_INPUT_TEMP = "input_temp"
CONF_EFFICIENCY = "efficiency"
CONF_OUTPUT_VOLTAGE = "output_voltage"
CONF_OUTPUT_CURRENT = "output_current"
CONF_MAX_OUTPUT_CURRENT = "max_output_current"
CONF_OUTPUT_POWER = "output_power"
CONF_OUTPUT_TEMP = "output_temp"

# Sensor schemas with device class, accuracy, etc.
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EMERSON_R48_ID): cv.use_id(EmersonR48Component),

        cv.Optional(CONF_INPUT_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),

        cv.Optional(CONF_OUTPUT_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),

        # ... additional sensor definitions ...
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_EMERSON_R48_ID])

    if CONF_INPUT_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_INPUT_VOLTAGE])
        cg.add(parent.set_input_voltage_sensor(sens))

    if CONF_OUTPUT_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_OUTPUT_VOLTAGE])
        cg.add(parent.set_output_voltage_sensor(sens))

    # ... additional sensor registrations ...
```

### Number Platform: `number/__init__.py`

**Location:** `components/emerson_r48/number/__init__.py`

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
)
from .. import CONF_EMERSON_R48_ID, EmersonR48Component, emerson_r48_ns

EmersonR48Number = emerson_r48_ns.class_(
    "EmersonR48Number", number.Number, cg.Component
)

# Number configurations
CONF_OUTPUT_VOLTAGE = "output_voltage"
CONF_MAX_OUTPUT_CURRENT = "max_output_current"
CONF_MAX_INPUT_CURRENT = "max_input_current"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EMERSON_R48_ID): cv.use_id(EmersonR48Component),

        cv.Optional(CONF_OUTPUT_VOLTAGE): number.NUMBER_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(EmersonR48Number),
                cv.Optional(CONF_MIN_VALUE, default=41.0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=58.5): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
            }
        ),

        cv.Optional(CONF_MAX_OUTPUT_CURRENT): number.NUMBER_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(EmersonR48Number),
                cv.Optional(CONF_MIN_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=121): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
            }
        ),

        cv.Optional(CONF_MAX_INPUT_CURRENT): number.NUMBER_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(EmersonR48Number),
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=20): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
            }
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_EMERSON_R48_ID])

    if CONF_OUTPUT_VOLTAGE in config:
        conf = config[CONF_OUTPUT_VOLTAGE]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await number.register_number(
            var,
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP],
        )
        cg.add(var.set_parent(parent, 0x0))  # SET_VOLTAGE_FUNCTION

    # ... additional number registrations ...
```

### Switch Platform: `switch/__init__.py`

**Location:** `components/emerson_r48/switch/__init__.py`

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID
from .. import CONF_EMERSON_R48_ID, EmersonR48Component, emerson_r48_ns

EmersonR48Switch = emerson_r48_ns.class_(
    "EmersonR48Switch", switch.Switch, cg.Component
)

CONF_AC_SW = "ac_sw"
CONF_DC_SW = "dc_sw"
CONF_FAN_SW = "fan_sw"
CONF_LED_SW = "led_sw"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EMERSON_R48_ID): cv.use_id(EmersonR48Component),
        cv.Optional(CONF_AC_SW): switch.SWITCH_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Switch)}
        ),
        cv.Optional(CONF_DC_SW): switch.SWITCH_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Switch)}
        ),
        cv.Optional(CONF_FAN_SW): switch.SWITCH_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Switch)}
        ),
        cv.Optional(CONF_LED_SW): switch.SWITCH_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Switch)}
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_EMERSON_R48_ID])

    if CONF_AC_SW in config:
        conf = config[CONF_AC_SW]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await switch.register_switch(var, conf)
        cg.add(var.set_parent(parent, 0x0))  # SET_AC_FUNCTION

    if CONF_DC_SW in config:
        conf = config[CONF_DC_SW]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await switch.register_switch(var, conf)
        cg.add(var.set_parent(parent, 0x1))  # SET_DC_FUNCTION

    # ... additional switch registrations ...
```

---

## Component Interaction Summary

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           User Interface                                  │
│              (Home Assistant / MQTT / ESPHome Web Server)                │
└────────────────────────────────┬─────────────────────────────────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ EmersonR48Switch│    │ EmersonR48Number│    │ EmersonR48Button│
│                 │    │                 │    │                 │
│ write_state()   │    │ control()       │    │ press_action()  │
│       │         │    │       │         │    │       │         │
└───────┼─────────┘    └───────┼─────────┘    └───────┼─────────┘
        │                      │                      │
        │ set parent state     │ call parent method   │ set_offline_values()
        │ + set_control()      │                      │
        │                      │                      │
        └──────────────────────┼──────────────────────┘
                               │
                               ▼
                ┌──────────────────────────────┐
                │    EmersonR48Component       │
                │                              │
                │ • set_output_voltage()       │
                │ • set_max_output_current()   │
                │ • set_max_input_current()    │
                │ • set_control()              │
                │ • set_offline_values()       │
                │                              │
                │ • update() polling           │
                │ • on_frame() RX handling     │
                └──────────────┬───────────────┘
                               │
                               │ canbus_->send()
                               ▼
                ┌──────────────────────────────┐
                │      MCP2515 Component       │
                │                              │
                │ • send_message()             │
                │ • read_message()             │
                │ • SPI communication          │
                └──────────────┬───────────────┘
                               │
                          CAN Bus
                               │
                               ▼
                ┌──────────────────────────────┐
                │     Vertiv R48 Rectifier     │
                └──────────────────────────────┘
```
