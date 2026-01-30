# System Architecture

This document describes the overall architecture of the ESPHome Emerson Vertiv R48 integration, including component relationships, data flow, and design patterns.

## Table of Contents

- [High-Level Architecture](#high-level-architecture)
- [Component Hierarchy](#component-hierarchy)
- [Data Flow](#data-flow)
- [Design Patterns](#design-patterns)
- [Initialization Sequence](#initialization-sequence)
- [Update Cycle](#update-cycle)
- [Timeout and Recovery](#timeout-and-recovery)

---

## High-Level Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ESPHome Device                                  │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                        EmersonR48Component                             │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │  │
│  │  │   Sensors   │  │  Switches   │  │   Numbers   │  │   Buttons   │   │  │
│  │  │  (11 types) │  │  (4 types)  │  │  (3 types)  │  │  (1 type)   │   │  │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘   │  │
│  │         │                │                │                │          │  │
│  │         └────────────────┴────────────────┴────────────────┘          │  │
│  │                                    │                                   │  │
│  │                           Parent Reference                             │  │
│  │                                    │                                   │  │
│  │                            ┌───────▼───────┐                           │  │
│  │                            │  CAN Message  │                           │  │
│  │                            │   Handler     │                           │  │
│  │                            └───────┬───────┘                           │  │
│  └────────────────────────────────────┼───────────────────────────────────┘  │
│                                       │                                      │
│  ┌────────────────────────────────────┼───────────────────────────────────┐  │
│  │                           MCP2515 Component                             │  │
│  │                            (CAN Controller)                             │  │
│  └────────────────────────────────────┼───────────────────────────────────┘  │
│                                       │                                      │
│  ┌────────────────────────────────────┼───────────────────────────────────┐  │
│  │                           SPI Interface                                 │  │
│  │                     GPIO14=CLK, GPIO13=MOSI, GPIO12=MISO               │  │
│  └────────────────────────────────────┼───────────────────────────────────┘  │
└───────────────────────────────────────┼──────────────────────────────────────┘
                                        │
                                   CAN Bus (125kbps)
                                        │
┌───────────────────────────────────────┼──────────────────────────────────────┐
│                            Vertiv R48 Rectifier                              │
│  ┌────────────────────────────────────┼───────────────────────────────────┐  │
│  │                           CAN Interface                                 │  │
│  │                     Extended 29-bit Identifiers                         │  │
│  └────────────────────────────────────┼───────────────────────────────────┘  │
│                                       │                                      │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                  │
│  │   AC Input     │  │   DC Output    │  │   Control      │                  │
│  │   Metering     │  │   Metering     │  │   Registers    │                  │
│  └────────────────┘  └────────────────┘  └────────────────┘                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Layer Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Application Layer                          │
│  Home Assistant / MQTT / Web Interface / API                    │
├─────────────────────────────────────────────────────────────────┤
│                      ESPHome Framework Layer                    │
│  Sensors, Switches, Numbers, Buttons, Automations               │
├─────────────────────────────────────────────────────────────────┤
│                      Integration Layer                          │
│  EmersonR48Component - Protocol Translation                     │
├─────────────────────────────────────────────────────────────────┤
│                      Communication Layer                        │
│  MCP2515 CAN Controller Driver                                  │
├─────────────────────────────────────────────────────────────────┤
│                      Hardware Layer                             │
│  SPI Bus, GPIO, Physical CAN Bus                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## Component Hierarchy

### Class Inheritance

```
esphome::Component
├── esphome::PollingComponent
│   └── emerson_r48::EmersonR48Component      # Main component
│
├── esphome::sensor::Sensor                   # Base sensor class
│   └── (configured via sensor.py)            # 11 sensor instances
│
├── esphome::switch_::Switch
│   └── emerson_r48::EmersonR48Switch         # 4 switch instances
│
├── esphome::number::Number
│   └── emerson_r48::EmersonR48Number         # 3 number instances
│
└── esphome::button::Button
    └── emerson_r48::EmersonR48Button         # 1 button instance

esphome::canbus::Canbus
└── esphome::mcp2515::MCP2515                 # CAN controller
    └── esphome::spi::SPIDevice               # SPI interface mixin
```

### Component Relationships

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        EmersonR48Component                               │
│                                                                          │
│  Owned References (Sensors):                                             │
│  ┌──────────────────┬──────────────────┬──────────────────┐             │
│  │ input_voltage_   │ input_current_   │ input_power_     │             │
│  │ input_frequency_ │ input_temp_      │ efficiency_      │             │
│  │ output_voltage_  │ output_current_  │ output_power_    │             │
│  │ output_temp_     │ max_output_curr_ │                  │             │
│  └──────────────────┴──────────────────┴──────────────────┘             │
│                                                                          │
│  State Variables:                                                        │
│  ┌──────────────────┬──────────────────┬──────────────────┐             │
│  │ dcOff_ (bool)    │ fanFull_ (bool)  │ flashLed_ (bool) │             │
│  │ acOff_ (bool)    │ lastUpdate_      │ intervalCount_   │             │
│  └──────────────────┴──────────────────┴──────────────────┘             │
│                                                                          │
│  External Reference:                                                     │
│  ┌──────────────────┐                                                    │
│  │ canbus_ (ptr)    │ ◄──── MCP2515 CAN Controller                      │
│  └──────────────────┘                                                    │
└─────────────────────────────────────────────────────────────────────────┘
         ▲                    ▲                    ▲
         │                    │                    │
    Parent Ref           Parent Ref           Parent Ref
         │                    │                    │
┌────────┴────────┐  ┌────────┴────────┐  ┌────────┴────────┐
│ EmersonR48Switch │  │ EmersonR48Number │  │ EmersonR48Button │
│ (4 instances)    │  │ (3 instances)    │  │ (1 instance)     │
│ functionCode_    │  │ functionCode_    │  │                  │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

---

## Data Flow

### Sensor Data Flow (Read Path)

```
┌─────────────┐    CAN Frame     ┌─────────────────────┐
│  Vertiv R48 │ ─────────────────► │     MCP2515        │
│  Rectifier  │  ID: 0x060f8003  │  CAN Controller     │
└─────────────┘                   └──────────┬──────────┘
                                             │
                                    CAN Trigger Callback
                                             │
                                             ▼
                               ┌─────────────────────────┐
                               │ EmersonR48Component::   │
                               │    on_frame()           │
                               │                         │
                               │ 1. Parse CAN ID         │
                               │ 2. Extract float value  │
                               │ 3. Identify parameter   │
                               │ 4. Route to sensor      │
                               └────────────┬────────────┘
                                            │
                         ┌──────────────────┼──────────────────┐
                         │                  │                  │
                         ▼                  ▼                  ▼
              ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
              │ output_voltage_  │ │ output_current_  │ │ output_temp_     │
              │ ->publish_state()│ │ ->publish_state()│ │ ->publish_state()│
              └────────┬─────────┘ └────────┬─────────┘ └────────┬─────────┘
                       │                    │                    │
                       └────────────────────┼────────────────────┘
                                            │
                                            ▼
                               ┌─────────────────────────┐
                               │   ESPHome Framework     │
                               │  (MQTT/HA/API publish)  │
                               └─────────────────────────┘
```

### Control Data Flow (Write Path)

```
┌─────────────────────────┐
│  User Interface         │
│  (HA / MQTT / Web)      │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐      ┌─────────────────────────┐
│  EmersonR48Number::     │      │  EmersonR48Switch::     │
│    control(value)       │  OR  │    write_state(state)   │
└───────────┬─────────────┘      └───────────┬─────────────┘
            │                                │
            │  functionCode routing          │  functionCode routing
            │                                │
            ▼                                ▼
┌───────────────────────────────────────────────────────────────┐
│                    EmersonR48Component                         │
│                                                                │
│  Number handlers:              Switch handlers:                │
│  ┌────────────────────────┐   ┌────────────────────────┐      │
│  │ set_output_voltage()   │   │ set acOff_/dcOff_/     │      │
│  │ set_max_output_current()│   │     fanFull_/flashLed_│      │
│  │ set_max_input_current()│   │ set_control(msgv)      │      │
│  └───────────┬────────────┘   └───────────┬────────────┘      │
│              │                            │                    │
│              └────────────┬───────────────┘                    │
│                           │                                    │
│              Build CAN Frame & Send                            │
└───────────────────────────┬────────────────────────────────────┘
                            │
                            ▼
               ┌─────────────────────────┐
               │       MCP2515           │
               │    canbus_->send()      │
               └───────────┬─────────────┘
                           │
                      CAN Bus
                           │
                           ▼
               ┌─────────────────────────┐
               │     Vertiv R48          │
               │     Rectifier           │
               └─────────────────────────┘
```

### Polling Cycle Sequence

```
Time ────────────────────────────────────────────────────────────────────►

│ update() │ update() │ update() │ update() │ update() │ update() │
│ count=0  │ count=1  │ count=2  │ count=3  │ count=4  │ count=5  │
│          │          │          │          │          │          │
│  Request │  Request │  Request │  Request │  Request │  Send    │
│  Output  │  Output  │  Current │  Temp    │  Input   │  Control │
│  Voltage │  Current │  Limit   │  (0x04)  │  Voltage │  Message │
│  (0x01)  │  (0x02)  │  (0x03)  │          │  (0x05)  │          │
│          │          │          │          │          │          │
     ▼          ▼          ▼          ▼          ▼
┌─────────────────────────────────────────────────────────┐
│                  Response Processing                     │
│  on_frame() parses responses and publishes sensor data  │
└─────────────────────────────────────────────────────────┘
```

---

## Design Patterns

### 1. Polling Component Pattern

The main component extends `PollingComponent` for regular data updates:

```cpp
class EmersonR48Component : public PollingComponent {
  void update() override {
    // Called at configured interval (default: 5s)
    // Cycles through 6 different request types
  }
};
```

**Benefits:**
- Predictable update frequency
- ESPHome handles scheduling
- Configurable via YAML `update_interval`

### 2. Parent-Child Component Pattern

Sub-components (switches, numbers, buttons) hold references to the main component:

```cpp
class EmersonR48Switch : public switch_::Switch {
  EmersonR48Component *parent_;
  int8_t functionCode_;

  void write_state(bool state) override {
    // Route to parent based on functionCode
    parent_->set_control(...);
  }
};
```

**Benefits:**
- Centralized CAN communication
- Shared state management
- Clean separation of concerns

### 3. Function Code Router Pattern

A single component class handles multiple logical functions via function codes:

```cpp
// In switch component:
switch (functionCode_) {
  case SET_AC_FUNCTION:   // 0x0
    parent_->acOff_ = state;
    break;
  case SET_DC_FUNCTION:   // 0x1
    parent_->dcOff_ = state;
    break;
  // ...
}
```

**Benefits:**
- Single implementation for similar functionality
- Configuration-driven behavior
- Reduced code duplication

### 4. CAN Trigger Automation Pattern

Uses ESPHome's automation system for CAN message handling:

```cpp
void setup() override {
  auto trigger = new canbus::CanbusTrigger(canbus_, 0, 0, true);
  trigger->add_callback([this](uint32_t id, bool rtr,
                               std::vector<uint8_t> &data) {
    this->on_frame(id, rtr, data);
  });
}
```

**Benefits:**
- Integrates with ESPHome's event system
- Non-blocking message processing
- Clean callback registration

### 5. Online/Offline Parameter Pattern

Parameters can be set temporarily (online) or permanently (offline):

```cpp
void set_output_voltage(float value, bool offline = false) {
  // Online:  parameter = 0x21 (RAM only)
  // Offline: parameter = 0x24 (EEPROM storage)
  uint8_t param = offline ? 0x24 : 0x21;
}
```

**Benefits:**
- Safe testing with temporary changes
- Persistence across power cycles
- User-controlled commit via button

---

## Initialization Sequence

```
┌────────────────────────────────────────────────────────────────────────┐
│                        Startup Sequence                                 │
└────────────────────────────────────────────────────────────────────────┘

1. ESPHome Boot
       │
       ▼
2. MCP2515::setup()
   ├── SPI initialization
   ├── CAN controller reset
   ├── Bit timing configuration (125kbps)
   ├── Filter/mask setup
   └── Enter NORMAL mode
       │
       ▼
3. EmersonR48Component::setup()
   ├── Create CanbusTrigger
   │   └── Register on_frame() callback
   ├── Register trigger as component
   ├── Add trigger to automation
   │
   ├── sendSync()
   │   └── CAN: [0x04, 0xF0, 0x01, 0x5A, 0, 0, 0, 0]
   │            ID: 0x0707FF83
   │
   └── gimme5()
       └── CAN: [0x20, 0xF0, 0x00, 0x80, 0, 0, 0, 0]
                ID: 0x06080783
       │
       ▼
4. First update() call
   └── Begin polling cycle
       │
       ▼
5. on_frame() receives responses
   └── Sensors start publishing data
```

---

## Update Cycle

### Detailed Update Flow

```cpp
void update() override {
  intervalCount_ = (intervalCount_ + 1) % 6;

  switch (intervalCount_) {
    case 0:  // Request output voltage
      sendCANRequest(EMR48_DATA_OUTPUT_V);   // 0x01
      break;
    case 1:  // Request output current
      sendCANRequest(EMR48_DATA_OUTPUT_A);   // 0x02
      break;
    case 2:  // Request current limit
      sendCANRequest(EMR48_DATA_OUTPUT_AL);  // 0x03
      break;
    case 3:  // Request temperature
      sendCANRequest(EMR48_DATA_OUTPUT_T);   // 0x04
      break;
    case 4:  // Request input voltage
      sendCANRequest(EMR48_DATA_OUTPUT_IV);  // 0x05
      break;
    case 5:  // Send control message
      set_control(buildControlByte());
      break;
  }

  // Check for timeout (no response in 5 intervals)
  checkTimeout();
}
```

### Timing Diagram

```
With update_interval: 1s

Time:   0s    1s    2s    3s    4s    5s    6s    7s    ...
        │     │     │     │     │     │     │     │
        ▼     ▼     ▼     ▼     ▼     ▼     ▼     ▼
Req:   [V]   [A]   [AL]  [T]   [IV]  [CTL] [V]   [A]   ...
        │     │     │     │     │     │     │     │
Resp:  ───►  ───►  ───►  ───►  ───►        ───►  ───►

Legend:
  [V]   = Output Voltage request
  [A]   = Output Current request
  [AL]  = Current Limit request
  [T]   = Temperature request
  [IV]  = Input Voltage request
  [CTL] = Control message (AC/DC/Fan/LED state)
```

---

## Timeout and Recovery

### Timeout Detection

```cpp
void checkTimeout() {
  uint32_t now = millis();
  uint32_t timeout = 5 * get_update_interval();

  if ((now - lastUpdate_) > timeout) {
    // Connection lost - publish NaN for all sensors
    publishAllSensorsNaN();

    // Attempt recovery
    sendSync();
    gimme5();
  }
}
```

### Recovery Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                    Timeout Recovery Flow                         │
└─────────────────────────────────────────────────────────────────┘

Normal Operation:
    │
    │  No response for 5 × update_interval
    │
    ▼
┌─────────────────────┐
│ Timeout Detected    │
│ lastUpdate_ stale   │
└─────────┬───────────┘
          │
          ├──► Publish NaN to all sensors
          │    (indicates disconnection)
          │
          ├──► sendSync()
          │    Re-synchronize with device
          │
          └──► gimme5()
               Request fresh data
          │
          ▼
┌─────────────────────┐
│ Wait for Response   │
│ on_frame() updates  │
│ lastUpdate_         │
└─────────────────────┘
          │
          │  If response received:
          ▼
┌─────────────────────┐
│ Normal Operation    │
│ Resumed             │
└─────────────────────┘
```

### Sensor NaN Handling

When timeout occurs, all sensors are set to NaN (Not a Number):

```cpp
void publishAllSensorsNaN() {
  if (output_voltage_sensor_)   output_voltage_sensor_->publish_state(NAN);
  if (output_current_sensor_)   output_current_sensor_->publish_state(NAN);
  if (output_temp_sensor_)      output_temp_sensor_->publish_state(NAN);
  if (input_voltage_sensor_)    input_voltage_sensor_->publish_state(NAN);
  if (max_output_current_sensor_)
                                max_output_current_sensor_->publish_state(NAN);
}
```

This allows Home Assistant and other consumers to detect when the device is unreachable.

---

## Memory Layout

### Component Memory Footprint

| Component | Stack | Heap | Notes |
|-----------|-------|------|-------|
| EmersonR48Component | ~100B | ~200B | Main state, pointers |
| Per Sensor | ~8B | ~50B | Pointer + filter state |
| Per Switch | ~16B | ~100B | State + parent ref |
| Per Number | ~16B | ~100B | Value + parent ref |
| MCP2515 | ~50B | ~500B | Buffers, SPI config |

### CAN Buffer Management

```
MCP2515 Internal Buffers:
┌─────────────────────────────────────────────┐
│ TX Buffers (3)                              │
│  TXB0: [ID(4)] [DLC(1)] [DATA(8)] = 13B    │
│  TXB1: [ID(4)] [DLC(1)] [DATA(8)] = 13B    │
│  TXB2: [ID(4)] [DLC(1)] [DATA(8)] = 13B    │
├─────────────────────────────────────────────┤
│ RX Buffers (2)                              │
│  RXB0: [ID(4)] [DLC(1)] [DATA(8)] = 13B    │
│  RXB1: [ID(4)] [DLC(1)] [DATA(8)] = 13B    │
└─────────────────────────────────────────────┘
```

---

## Thread Safety Considerations

ESPHome runs in a single-threaded event loop, so:

- No mutex/lock requirements for state variables
- CAN callbacks execute in main loop context
- Sensor updates are atomic from application perspective
- State changes are immediately visible

However, care should be taken with:
- Long-running operations (can delay CAN processing)
- Large memory allocations (limited heap on ESP8266)
- Blocking I/O (should be avoided)
