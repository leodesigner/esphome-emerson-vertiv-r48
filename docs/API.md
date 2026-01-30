# API Reference

This document provides a complete API reference for all classes, methods, constants, and configuration options in the ESPHome Emerson Vertiv R48 integration.

## Table of Contents

- [Namespace](#namespace)
- [Constants](#constants)
- [EmersonR48Component Class](#emersonr48component-class)
- [EmersonR48Switch Class](#emersonr48switch-class)
- [EmersonR48Number Class](#emersonr48number-class)
- [EmersonR48Button Class](#emersonr48button-class)
- [MCP2515 Class](#mcp2515-class)
- [Enumerations](#enumerations)
- [Type Definitions](#type-definitions)
- [Python API](#python-api)

---

## Namespace

All classes are defined within the `esphome::emerson_r48` namespace:

```cpp
namespace esphome {
namespace emerson_r48 {
  // All R48 classes defined here
}
}
```

MCP2515 classes are in `esphome::mcp2515`:

```cpp
namespace esphome {
namespace mcp2515 {
  // MCP2515 classes defined here
}
}
```

---

## Constants

### R48 Specifications

**Location:** `components/emerson_r48/emerson_r48.cpp`

```cpp
// Output voltage limits
static const float EMR48_OUTPUT_VOLTAGE_MIN = 41.0f;      // Minimum output voltage (V)
static const float EMR48_OUTPUT_VOLTAGE_MAX = 58.5f;      // Maximum output voltage (V)

// Output current specifications
static const float EMR48_OUTPUT_CURRENT_RATED_VALUE = 62.5f;          // Rated current (A)
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN = 10.0f; // Minimum limit (%)
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX = 121.0f;// Maximum limit (%)
static const float EMR48_OUTPUT_CURRENT_MIN = 5.5f;       // Minimum current (A) ~10%
static const float EMR48_OUTPUT_CURRENT_MAX = 62.5f;      // Maximum current (A)
```

### CAN Message IDs

**Location:** `components/emerson_r48/emerson_r48.cpp`

```cpp
// Request/Response IDs
static const uint32_t CAN_ID_REQUEST = 0x06000783;   // Request parameter value
static const uint32_t CAN_ID_DATA = 0x060F8003;      // Data response
static const uint32_t CAN_ID_DATA2 = 0x060F8007;     // Extended data response

// Set Parameter IDs
static const uint32_t CAN_ID_SET = 0x0607FF83;       // Set parameter value
static const uint32_t CAN_ID_SET2 = 0x0677FF83;      // Alternative set command
static const uint32_t CAN_ID_SET_CTL = 0x06080783;   // Set control flags

// Synchronization IDs
static const uint32_t CAN_ID_SYNC = 0x0707FF83;      // Sync message 1
static const uint32_t CAN_ID_SYNC2 = 0x0717FF83;     // Sync message 2
static const uint32_t CAN_ID_GIMME5 = 0x06080783;    // Request all data
```

### Parameter Codes

**Location:** `components/emerson_r48/emerson_r48.cpp`

```cpp
// Read parameter codes
static const uint8_t EMR48_DATA_OUTPUT_V = 0x01;     // Output voltage
static const uint8_t EMR48_DATA_OUTPUT_A = 0x02;     // Output current
static const uint8_t EMR48_DATA_OUTPUT_AL = 0x03;    // Output current limit
static const uint8_t EMR48_DATA_OUTPUT_T = 0x04;     // Temperature
static const uint8_t EMR48_DATA_OUTPUT_IV = 0x05;    // Input voltage

// Write parameter codes (online/offline)
// Voltage: 0x21 (online), 0x24 (offline/EEPROM)
// Current: 0x22 (online), 0x19 (offline/EEPROM)
// Input current: 0x1A
```

### Function Codes

**Location:** `components/emerson_r48/switch/emerson_switch.h`

```cpp
// Switch function codes
static const int8_t SET_AC_FUNCTION = 0x0;           // AC input control
static const int8_t SET_DC_FUNCTION = 0x1;           // DC output control
static const int8_t SET_FAN_FUNCTION = 0x2;          // Fan speed control
static const int8_t SET_LED_FUNCTION = 0x3;          // LED control
```

**Location:** `components/emerson_r48/number/emerson_r48_number.h`

```cpp
// Number function codes
static const int8_t SET_VOLTAGE_FUNCTION = 0x0;      // Output voltage
static const int8_t SET_CURRENT_FUNCTION = 0x3;      // Output current limit
static const int8_t SET_INPUT_CURRENT_FUNCTION = 0x4;// Input current limit
```

---

## EmersonR48Component Class

**Header:** `components/emerson_r48/emerson_r48.h`
**Implementation:** `components/emerson_r48/emerson_r48.cpp`

### Class Definition

```cpp
class EmersonR48Component : public PollingComponent {
 public:
  // Constructor
  EmersonR48Component(canbus::Canbus *canbus);

  // ESPHome lifecycle
  void setup() override;
  void update() override;
  float get_setup_priority() const override;

  // Sensor setters
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

  // Control methods
  void set_output_voltage(float value, bool offline = false);
  void set_max_output_current(float value, bool offline = false);
  void set_max_input_current(float value);
  void set_offline_values();
  void set_control(uint8_t msgv);

  // Communication methods
  void sendSync();
  void sendSync2();
  void gimme5();
  void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);

  // Public state variables
  bool dcOff_;
  bool fanFull_;
  bool flashLed_;
  bool acOff_;
};
```

### Constructor

```cpp
EmersonR48Component(canbus::Canbus *canbus)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `canbus` | `canbus::Canbus*` | Pointer to CAN bus component |

**Description:** Creates a new EmersonR48Component instance with the specified CAN bus.

---

### setup()

```cpp
void setup() override
```

**Description:** Initializes the component. Called automatically by ESPHome during startup.

**Actions:**
1. Creates CAN trigger for receiving all messages
2. Registers trigger callback to `on_frame()`
3. Sends initial sync message
4. Sends gimme5 request

---

### update()

```cpp
void update() override
```

**Description:** Called periodically at `update_interval`. Cycles through 6 request types.

**Polling Cycle:**

| Count | Action |
|-------|--------|
| 0 | Request output voltage (0x01) |
| 1 | Request output current (0x02) |
| 2 | Request current limit (0x03) |
| 3 | Request temperature (0x04) |
| 4 | Request input voltage (0x05) |
| 5 | Send control message |

Also checks for timeout and initiates recovery if needed.

---

### get_setup_priority()

```cpp
float get_setup_priority() const override
```

**Returns:** `setup_priority::DATA` - Priority for component initialization order.

---

### Sensor Setters

```cpp
void set_input_voltage_sensor(sensor::Sensor *sensor)
void set_input_frequency_sensor(sensor::Sensor *sensor)
void set_input_current_sensor(sensor::Sensor *sensor)
void set_input_power_sensor(sensor::Sensor *sensor)
void set_input_temp_sensor(sensor::Sensor *sensor)
void set_efficiency_sensor(sensor::Sensor *sensor)
void set_output_voltage_sensor(sensor::Sensor *sensor)
void set_output_current_sensor(sensor::Sensor *sensor)
void set_max_output_current_sensor(sensor::Sensor *sensor)
void set_output_power_sensor(sensor::Sensor *sensor)
void set_output_temp_sensor(sensor::Sensor *sensor)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `sensor` | `sensor::Sensor*` | Pointer to sensor instance |

**Description:** Associates a sensor with this component for value publication.

---

### set_output_voltage()

```cpp
void set_output_voltage(float value, bool offline = false)
```

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `value` | `float` | - | Voltage in volts (41.0 - 58.5) |
| `offline` | `bool` | `false` | If true, store in EEPROM |

**Description:** Sets the DC output voltage.

**CAN Message:**
```
ID: 0x0607FF83
Data: [0x03, 0xF0, 0x00, param, float[3], float[2], float[1], float[0]]
  param: 0x21 (online) or 0x24 (offline)
```

**Example:**
```cpp
// Set voltage to 52.0V temporarily
parent->set_output_voltage(52.0f);

// Set voltage to 52.0V permanently
parent->set_output_voltage(52.0f, true);
```

---

### set_max_output_current()

```cpp
void set_max_output_current(float value, bool offline = false)
```

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `value` | `float` | - | Current limit in percent (10 - 121) |
| `offline` | `bool` | `false` | If true, store in EEPROM |

**Description:** Sets the output current limit as percentage of rated current.

**Conversion:** Value is divided by 100 before transmission (50% → 0.50)

**CAN Message:**
```
ID: 0x0607FF83
Data: [0x03, 0xF0, 0x00, param, float[3], float[2], float[1], float[0]]
  param: 0x22 (online) or 0x19 (offline)
```

---

### set_max_input_current()

```cpp
void set_max_input_current(float value)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `value` | `float` | AC current limit in amps (0 - 20) |

**Description:** Sets the AC input current limit.

**CAN Message:**
```
ID: 0x0607FF83
Data: [0x03, 0xF0, 0x00, 0x1A, float[3], float[2], float[1], float[0]]
```

---

### set_offline_values()

```cpp
void set_offline_values()
```

**Description:** Commits current voltage and current settings to EEPROM for persistence across power cycles.

**Actions:**
1. Calls `set_output_voltage()` with `offline=true`
2. Calls `set_max_output_current()` with `offline=true`

---

### set_control()

```cpp
void set_control(uint8_t msgv)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `msgv` | `uint8_t` | Control byte (see Control Byte Format) |

**Description:** Sends a control message to set AC/DC/Fan/LED states.

**CAN Message:**
```
ID: 0x06080783
Data: [0x00, 0xF0, msgv, 0x80, 0x00, 0x00, 0x00, 0x00]
```

**Control Byte Format:**
```
Bit 7: DC Off
Bit 4: Fan Full
Bit 3: LED Flash
Bit 2: AC Off
Bit 0: Always 1
```

---

### sendSync()

```cpp
void sendSync()
```

**Description:** Sends synchronization message to establish/re-establish communication.

**CAN Message:**
```
ID: 0x0707FF83
Data: [0x04, 0xF0, 0x01, 0x5A, 0x00, 0x00, 0x00, 0x00]
```

---

### sendSync2()

```cpp
void sendSync2()
```

**Description:** Sends alternative synchronization message.

**CAN Message:**
```
ID: 0x0717FF83
Data: [0x04, 0xF0, 0x01, 0x5A, 0x00, 0x00, 0x00, 0x00]
```

---

### gimme5()

```cpp
void gimme5()
```

**Description:** Requests the R48 to send all current data values.

**CAN Message:**
```
ID: 0x06080783
Data: [0x20, 0xF0, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00]
```

---

### on_frame()

```cpp
void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `can_id` | `uint32_t` | CAN message identifier |
| `rtr` | `bool` | Remote transmission request flag |
| `data` | `std::vector<uint8_t>&` | Message data bytes |

**Description:** Callback for received CAN frames. Parses data and publishes to sensors.

**Processing:**
1. Filters for `CAN_ID_DATA` (0x060F8003)
2. Extracts float from bytes 4-7 (big-endian)
3. Routes to sensor based on parameter code in byte 3

---

### Public State Variables

| Variable | Type | Description |
|----------|------|-------------|
| `dcOff_` | `bool` | DC output disabled state |
| `fanFull_` | `bool` | Fan full speed state |
| `flashLed_` | `bool` | LED flash state |
| `acOff_` | `bool` | AC input disabled state |

These are public to allow direct access from switch components.

---

## EmersonR48Switch Class

**Header:** `components/emerson_r48/switch/emerson_switch.h`
**Implementation:** `components/emerson_r48/switch/emerson_switch.cpp`

### Class Definition

```cpp
class EmersonR48Switch : public switch_::Switch, public Component {
 public:
  void set_parent(EmersonR48Component *parent, int8_t functionCode);
  void dump_config() override;

 protected:
  void write_state(bool state) override;

  EmersonR48Component *parent_;
  int8_t functionCode_;
};
```

### set_parent()

```cpp
void set_parent(EmersonR48Component *parent, int8_t functionCode)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `parent` | `EmersonR48Component*` | Parent component |
| `functionCode` | `int8_t` | Switch function identifier |

**Function Codes:**
| Code | Name | Parent Variable |
|------|------|-----------------|
| `0x0` | `SET_AC_FUNCTION` | `acOff_` |
| `0x1` | `SET_DC_FUNCTION` | `dcOff_` |
| `0x2` | `SET_FAN_FUNCTION` | `fanFull_` |
| `0x3` | `SET_LED_FUNCTION` | `flashLed_` |

---

### write_state()

```cpp
void write_state(bool state) override
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `state` | `bool` | New switch state |

**Description:** Called when switch state changes. Updates parent state and sends control message.

---

### dump_config()

```cpp
void dump_config() override
```

**Description:** Outputs configuration to log during startup.

---

## EmersonR48Number Class

**Header:** `components/emerson_r48/number/emerson_r48_number.h`
**Implementation:** `components/emerson_r48/number/emerson_r48_number.cpp`

### Class Definition

```cpp
class EmersonR48Number : public number::Number, public Component {
 public:
  void set_parent(EmersonR48Component *parent, int8_t functionCode);
  void dump_config() override;

 protected:
  void control(float value) override;

  EmersonR48Component *parent_;
  int8_t functionCode_;
};
```

### set_parent()

```cpp
void set_parent(EmersonR48Component *parent, int8_t functionCode)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `parent` | `EmersonR48Component*` | Parent component |
| `functionCode` | `int8_t` | Number function identifier |

**Function Codes:**
| Code | Name | Method Called |
|------|------|---------------|
| `0x0` | `SET_VOLTAGE_FUNCTION` | `set_output_voltage()` |
| `0x3` | `SET_CURRENT_FUNCTION` | `set_max_output_current()` |
| `0x4` | `SET_INPUT_CURRENT_FUNCTION` | `set_max_input_current()` |

---

### control()

```cpp
void control(float value) override
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `value` | `float` | New value to set |

**Description:** Called when number value changes. Routes to appropriate parent method.

---

## EmersonR48Button Class

**Header:** `components/emerson_r48/button/emerson_r48_button.h`
**Implementation:** `components/emerson_r48/button/emerson_r48_button.cpp`

### Class Definition

```cpp
class EmersonR48Button : public button::Button, public Component {
 public:
  void set_parent(EmersonR48Component *parent);
  void dump_config() override;

 protected:
  void press_action() override;

  EmersonR48Component *parent_;
};
```

### set_parent()

```cpp
void set_parent(EmersonR48Component *parent)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `parent` | `EmersonR48Component*` | Parent component |

---

### press_action()

```cpp
void press_action() override
```

**Description:** Called when button is pressed. Invokes `parent_->set_offline_values()`.

---

## MCP2515 Class

**Header:** `components/mcp2515/mcp2515.h`
**Implementation:** `components/mcp2515/mcp2515.cpp`

### Class Definition

```cpp
class MCP2515 : public canbus::Canbus,
                public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                      spi::CLOCK_POLARITY_LOW,
                                      spi::CLOCK_PHASE_LEADING,
                                      spi::DATA_RATE_8MHZ> {
 public:
  MCP2515();
  void set_clock(CanClock clock);
  void set_mode(CanMode mode);
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  canbus::Error send_message(uint32_t can_id, bool ext_id,
                             const std::vector<uint8_t> &data) override;
  canbus::Error read_message(uint32_t *can_id, bool *ext_id,
                             std::vector<uint8_t> *data) override;
};
```

### set_clock()

```cpp
void set_clock(CanClock clock)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `clock` | `CanClock` | Crystal frequency |

**Values:** `MCP_8MHZ`, `MCP_12MHZ`, `MCP_16MHZ`, `MCP_20MHZ`

---

### set_mode()

```cpp
void set_mode(CanMode mode)
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `mode` | `CanMode` | Operating mode |

**Values:** `NORMAL`, `LOOPBACK`, `LISTENONLY`

---

### send_message()

```cpp
canbus::Error send_message(uint32_t can_id, bool ext_id,
                           const std::vector<uint8_t> &data) override
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `can_id` | `uint32_t` | CAN identifier |
| `ext_id` | `bool` | Use extended (29-bit) ID |
| `data` | `const std::vector<uint8_t>&` | Data bytes (max 8) |

**Returns:** `canbus::Error` - Error code or `ERROR_OK`

---

### read_message()

```cpp
canbus::Error read_message(uint32_t *can_id, bool *ext_id,
                           std::vector<uint8_t> *data) override
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `can_id` | `uint32_t*` | Output: CAN identifier |
| `ext_id` | `bool*` | Output: Extended ID flag |
| `data` | `std::vector<uint8_t>*` | Output: Data bytes |

**Returns:** `canbus::Error` - Error code or `ERROR_OK`

---

## Enumerations

### CanClock

**Location:** `components/mcp2515/mcp2515.h`

```cpp
enum CanClock {
  MCP_8MHZ,    // 8 MHz crystal
  MCP_12MHZ,   // 12 MHz crystal
  MCP_16MHZ,   // 16 MHz crystal
  MCP_20MHZ    // 20 MHz crystal
};
```

### CanSpeed

**Location:** `components/mcp2515/mcp2515.h`

```cpp
enum CanSpeed {
  CAN_125KBPS,   // 125 kbps (required for R48)
  CAN_250KBPS,   // 250 kbps
  CAN_500KBPS,   // 500 kbps
  CAN_1000KBPS   // 1 Mbps
};
```

### CanMode

**Location:** `components/mcp2515/mcp2515.h`

```cpp
enum CanMode {
  NORMAL,      // Normal operation
  LOOPBACK,    // Internal loopback for testing
  LISTENONLY   // Receive only, no ACK
};
```

### canbus::Error

**Location:** ESPHome canbus component

```cpp
enum Error {
  ERROR_OK = 0,           // Success
  ERROR_FAIL,             // General failure
  ERROR_ALLTXBUSY,        // All TX buffers busy
  ERROR_FAILINIT,         // Initialization failed
  ERROR_FAILTX,           // Transmission failed
  ERROR_NOMSG             // No message available
};
```

---

## Type Definitions

### Sensor Value Types

All sensor values are `float`:

| Sensor | C++ Type | Range | Unit |
|--------|----------|-------|------|
| Input voltage | `float` | 0 - 300 | V |
| Input frequency | `float` | 0 - 100 | Hz |
| Input current | `float` | 0 - 50 | A |
| Input power | `float` | 0 - 5000 | W |
| Temperature | `float` | -40 - 125 | °C |
| Output voltage | `float` | 0 - 60 | V |
| Output current | `float` | 0 - 80 | A |
| Current limit | `float` | 0 - 121 | % |
| Output power | `float` | 0 - 5000 | W |

---

## Python API

### Configuration Schemas

#### Main Component Schema

**Location:** `components/emerson_r48/__init__.py`

```python
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EmersonR48Component),
        cv.Required(CONF_CANBUS_ID): cv.use_id(canbus.CanbusComponent),
    }
).extend(cv.polling_component_schema("5s"))
```

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `id` | `ID` | No | Auto | Component ID |
| `canbus_id` | `use_id` | Yes | - | CAN bus reference |
| `update_interval` | `Time` | No | `5s` | Polling interval |

#### Sensor Schema

**Location:** `components/emerson_r48/sensor.py`

```python
cv.Optional(CONF_OUTPUT_VOLTAGE): sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
)
```

| Sensor | Unit | Decimals | Device Class |
|--------|------|----------|--------------|
| `input_voltage` | V | 1 | voltage |
| `input_frequency` | Hz | 3 | frequency |
| `input_current` | A | 2 | current |
| `input_power` | W | 1 | power |
| `input_temp` | °C | 1 | temperature |
| `efficiency` | % | 0 | - |
| `output_voltage` | V | 2 | voltage |
| `output_current` | A | 2 | current |
| `max_output_current` | A | 2 | current |
| `output_power` | W | 1 | power |
| `output_temp` | °C | 1 | temperature |

#### Switch Schema

**Location:** `components/emerson_r48/switch/__init__.py`

```python
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EMERSON_R48_ID): cv.use_id(EmersonR48Component),
        cv.Optional(CONF_AC_SW): switch.SWITCH_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Switch)}
        ),
        cv.Optional(CONF_DC_SW): switch.SWITCH_SCHEMA.extend(...),
        cv.Optional(CONF_FAN_SW): switch.SWITCH_SCHEMA.extend(...),
        cv.Optional(CONF_LED_SW): switch.SWITCH_SCHEMA.extend(...),
    }
)
```

| Switch | Key | Function Code |
|--------|-----|---------------|
| AC Switch | `ac_sw` | `0x0` |
| DC Switch | `dc_sw` | `0x1` |
| Fan Switch | `fan_sw` | `0x2` |
| LED Switch | `led_sw` | `0x3` |

#### Number Schema

**Location:** `components/emerson_r48/number/__init__.py`

```python
cv.Optional(CONF_OUTPUT_VOLTAGE): number.NUMBER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(EmersonR48Number),
        cv.Optional(CONF_MIN_VALUE, default=41.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=58.5): cv.float_,
        cv.Optional(CONF_STEP, default=0.1): cv.float_,
    }
)
```

| Number | Min | Max | Step | Function Code |
|--------|-----|-----|------|---------------|
| `output_voltage` | 41.0 | 58.5 | 0.1 | `0x0` |
| `max_output_current` | 10 | 121 | 0.1 | `0x3` |
| `max_input_current` | 0 | 20 | 0.1 | `0x4` |

#### Button Schema

**Location:** `components/emerson_r48/button/__init__.py`

```python
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EMERSON_R48_ID): cv.use_id(EmersonR48Component),
        cv.Optional(CONF_SET_OFFLINE_VALUES): button.BUTTON_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(EmersonR48Button)}
        ),
    }
)
```

| Button | Key |
|--------|-----|
| Save to EEPROM | `set_offline_values` |

### Code Generation Functions

#### to_code() Examples

```python
# Main component
async def to_code(config):
    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    var = cg.new_Pvariable(config[CONF_ID], canbus)
    await cg.register_component(var, config)

# Sensor
async def to_code(config):
    parent = await cg.get_variable(config[CONF_EMERSON_R48_ID])
    if CONF_OUTPUT_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_OUTPUT_VOLTAGE])
        cg.add(parent.set_output_voltage_sensor(sens))

# Number
async def to_code(config):
    parent = await cg.get_variable(config[CONF_EMERSON_R48_ID])
    if CONF_OUTPUT_VOLTAGE in config:
        conf = config[CONF_OUTPUT_VOLTAGE]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await number.register_number(var, conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP])
        cg.add(var.set_parent(parent, 0x0))
```

---

## Usage Examples

### Lambda in ESPHome Automations

```yaml
# Access component from lambda
on_value:
  then:
    - lambda: |-
        id(r48)->set_output_voltage(52.0f);

# Read sensor value
sensor:
  - platform: template
    lambda: |-
      return id(output_voltage).state;
```

### Custom Component Integration

```cpp
// Access from custom component
auto *r48 = id(r48);
r48->set_output_voltage(52.0f, false);  // Online
r48->set_max_output_current(80.0f);     // 80% limit

// Read switch state
bool dc_is_off = r48->dcOff_;
```

### Direct CAN Communication

```cpp
// Send custom CAN message through component's bus
auto *canbus = id(can);
std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
canbus->send(0x06000783, true, data);
```
