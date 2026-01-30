# ESPHome Configuration Guide

This document provides comprehensive guidance for configuring the ESPHome Emerson Vertiv R48 integration in your YAML configuration files.

## Table of Contents

- [Quick Start](#quick-start)
- [Hardware Requirements](#hardware-requirements)
- [Basic Configuration](#basic-configuration)
- [Component Reference](#component-reference)
- [Sensor Configuration](#sensor-configuration)
- [Switch Configuration](#switch-configuration)
- [Number Configuration](#number-configuration)
- [Button Configuration](#button-configuration)
- [Advanced Configuration](#advanced-configuration)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

### Minimal Configuration

```yaml
esphome:
  name: emerson-r48
  platform: ESP8266
  board: esp01_1m

# Required external component
external_components:
  - source: github://arnoutzw/esphome-emerson-vertiv-r48

# SPI for MCP2515
spi:
  clk_pin: GPIO14
  mosi_pin: GPIO13
  miso_pin: GPIO12

# CAN bus configuration
canbus:
  - platform: mcp2515
    id: can
    cs_pin: GPIO15
    can_id: 0x0607FF83
    bit_rate: 125kbps
    use_extended_id: true

# Emerson R48 component
emerson_r48:
  canbus_id: can

# Basic sensors
sensor:
  - platform: emerson_r48
    output_voltage:
      name: "Output Voltage"
    output_current:
      name: "Output Current"
```

---

## Hardware Requirements

### Supported Microcontrollers

| Platform | Status | Notes |
|----------|--------|-------|
| ESP8266 | Tested | ESP01_1M, NodeMCU, Wemos D1 |
| ESP32 | Supported | Any ESP32 variant |
| ESP32-S2 | Supported | Native USB support |
| ESP32-S3 | Supported | Dual-core, native USB |
| ESP32-C3 | Supported | RISC-V, low power |

### MCP2515 CAN Controller

The MCP2515 is required for CAN bus communication.

**Wiring Diagram:**

```
ESP8266/ESP32          MCP2515 Module
┌───────────┐          ┌───────────┐
│           │          │           │
│    3.3V ──┼──────────┼── VCC     │
│    GND ───┼──────────┼── GND     │
│           │          │           │
│   GPIO14 ─┼── CLK ───┼── SCK     │
│   GPIO13 ─┼── MOSI ──┼── SI      │
│   GPIO12 ─┼── MISO ──┼── SO      │
│   GPIO15 ─┼── CS ────┼── CS      │
│           │          │           │
│   (opt) ──┼── INT ───┼── INT     │
│           │          │           │
└───────────┘          └───────────┘
                              │
                              │ CAN_H, CAN_L
                              ▼
                       ┌───────────┐
                       │ Vertiv    │
                       │ R48       │
                       │ CAN Port  │
                       └───────────┘
```

**Pin Mapping Options:**

| Function | ESP8266 Default | ESP32 Default | Customizable |
|----------|-----------------|---------------|--------------|
| CLK | GPIO14 | GPIO18 | Yes |
| MOSI | GPIO13 | GPIO23 | Yes |
| MISO | GPIO12 | GPIO19 | Yes |
| CS | GPIO15 | GPIO5 | Yes |
| INT | Optional | Optional | Yes |

### Vertiv R48 CAN Connection

The R48 power supply has a CAN bus interface, typically accessible via:
- Dedicated CAN port connector
- Communication module slot
- Service port (model dependent)

**CAN Bus Specifications:**
- Bit Rate: 125 kbps
- Termination: 120Ω (may be built-in)
- Protocol: CAN 2.0B Extended

---

## Basic Configuration

### ESPHome Core Configuration

```yaml
esphome:
  name: "emerson-vertiv-r48"
  friendly_name: "Emerson Vertiv R48 Power Supply"
  comment: "48V DC Rectifier Controller"

# Choose your platform
esp8266:
  board: esp01_1m
  # Or: nodemcuv2, d1_mini, etc.

# Alternative: ESP32
# esp32:
#   board: esp32dev
#   framework:
#     type: arduino
```

### External Component Source

```yaml
external_components:
  # From GitHub (recommended)
  - source: github://arnoutzw/esphome-emerson-vertiv-r48
    refresh: 1d  # Check for updates daily

  # Or from local path
  # - source:
  #     type: local
  #     path: components
```

### WiFi Configuration

```yaml
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # Static IP (recommended for reliability)
  manual_ip:
    static_ip: 192.168.1.100
    gateway: 192.168.1.1
    subnet: 255.255.255.0

  # Fallback hotspot
  ap:
    ssid: "Emerson-R48-Fallback"
    password: "fallback123"
```

### API and OTA

```yaml
api:
  encryption:
    key: !secret api_key

ota:
  password: !secret ota_password

# Optional web server
web_server:
  port: 80
```

### Logging

```yaml
logger:
  level: INFO  # DEBUG for troubleshooting
  logs:
    emerson_r48: DEBUG
    mcp2515: DEBUG
    canbus: DEBUG
```

---

## Component Reference

### SPI Configuration

```yaml
spi:
  id: spi_bus              # Optional ID
  clk_pin: GPIO14          # SPI Clock
  mosi_pin: GPIO13         # Master Out Slave In
  miso_pin: GPIO12         # Master In Slave Out
```

**Options:**

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `id` | No | - | SPI bus identifier |
| `clk_pin` | Yes | - | Clock pin |
| `mosi_pin` | Yes | - | MOSI pin |
| `miso_pin` | Yes | - | MISO pin |

### CAN Bus (MCP2515) Configuration

```yaml
canbus:
  - platform: mcp2515
    id: can_bus
    spi_id: spi_bus        # Reference to SPI bus (optional)
    cs_pin: GPIO15         # Chip select pin
    can_id: 0x0607FF83     # Default CAN ID for TX
    bit_rate: 125kbps      # CAN bit rate
    use_extended_id: true  # Use 29-bit extended IDs
    mode: NORMAL           # Operating mode
    clock: 8MHZ            # MCP2515 crystal frequency
    data_rate: 10Mhz       # SPI data rate
```

**Options:**

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `platform` | Yes | - | Must be `mcp2515` |
| `id` | Yes | - | CAN bus identifier |
| `cs_pin` | Yes | - | Chip select GPIO |
| `can_id` | Yes | - | Default transmit CAN ID |
| `bit_rate` | Yes | - | CAN bus speed |
| `use_extended_id` | Yes | - | Enable 29-bit IDs |
| `mode` | No | `NORMAL` | `NORMAL`, `LOOPBACK`, `LISTENONLY` |
| `clock` | No | `8MHZ` | `8MHZ`, `12MHZ`, `16MHZ`, `20MHZ` |
| `data_rate` | No | `8Mhz` | SPI clock speed |

**Bit Rate Options:**
- `125kbps` - Required for Vertiv R48
- `250kbps`, `500kbps`, `1000kbps` - Other devices

### Emerson R48 Component

```yaml
emerson_r48:
  id: r48                  # Optional component ID
  canbus_id: can_bus       # Reference to CAN bus
  update_interval: 1s      # Polling interval
```

**Options:**

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `id` | No | - | Component identifier |
| `canbus_id` | Yes | - | Reference to CAN bus component |
| `update_interval` | No | `5s` | Data polling interval |

**Update Interval Notes:**
- Minimum recommended: `500ms`
- Default: `5s`
- Full data cycle: 6 × update_interval
- Control messages: Every 6th update

---

## Sensor Configuration

### Available Sensors

```yaml
sensor:
  - platform: emerson_r48
    # Input (AC) sensors
    input_voltage:
      name: "AC Input Voltage"
    input_frequency:
      name: "AC Input Frequency"
    input_current:
      name: "AC Input Current"
    input_power:
      name: "AC Input Power"
    input_temp:
      name: "Internal Temperature"

    # Output (DC) sensors
    output_voltage:
      name: "DC Output Voltage"
    output_current:
      name: "DC Output Current"
    max_output_current:
      name: "DC Current Limit"
    output_power:
      name: "DC Output Power"
    output_temp:
      name: "Output Temperature"

    # Efficiency
    efficiency:
      name: "Conversion Efficiency"
```

### Sensor Properties

| Sensor | Unit | Accuracy | Device Class |
|--------|------|----------|--------------|
| `input_voltage` | V | 1 decimal | `voltage` |
| `input_frequency` | Hz | 3 decimals | `frequency` |
| `input_current` | A | 2 decimals | `current` |
| `input_power` | W | 1 decimal | `power` |
| `input_temp` | °C | 1 decimal | `temperature` |
| `output_voltage` | V | 2 decimals | `voltage` |
| `output_current` | A | 2 decimals | `current` |
| `max_output_current` | % | 2 decimals | `current` |
| `output_power` | W | 1 decimal | `power` |
| `output_temp` | °C | 1 decimal | `temperature` |
| `efficiency` | % | 0 decimals | - |

### Customizing Sensor Options

```yaml
sensor:
  - platform: emerson_r48
    output_voltage:
      name: "DC Output Voltage"
      id: dc_voltage
      unit_of_measurement: "V"
      accuracy_decimals: 3
      filters:
        - sliding_window_moving_average:
            window_size: 5
            send_every: 1
      on_value:
        then:
          - logger.log: "Voltage changed"
      on_value_range:
        - below: 45.0
          then:
            - logger.log: "Low voltage warning!"
```

### Computed Sensors (Templates)

```yaml
sensor:
  - platform: emerson_r48
    output_voltage:
      id: dc_voltage
      internal: true  # Don't expose to HA
    output_current:
      id: dc_current
      internal: true

  # Computed power
  - platform: template
    name: "Calculated Power"
    id: calc_power
    unit_of_measurement: "W"
    lambda: |-
      return id(dc_voltage).state * id(dc_current).state;
    update_interval: 1s
```

---

## Switch Configuration

### Available Switches

```yaml
switch:
  - platform: emerson_r48
    ac_sw:
      name: "AC Input Off"
    dc_sw:
      name: "DC Output Off"
    fan_sw:
      name: "Fan Full Speed"
    led_sw:
      name: "LED Flash"
```

### Switch Behavior

| Switch | OFF State | ON State |
|--------|-----------|----------|
| `ac_sw` | AC input enabled | AC input disabled |
| `dc_sw` | DC output enabled | DC output disabled |
| `fan_sw` | Normal fan speed | Maximum fan speed |
| `led_sw` | LED normal | LED flashing |

**Important:** The switches control "off" states. Turning the `dc_sw` ON will turn the DC output OFF.

### Customizing Switches

```yaml
switch:
  - platform: emerson_r48
    dc_sw:
      name: "DC Output Off"
      id: dc_off_switch
      icon: "mdi:power-plug-off"
      restore_mode: ALWAYS_OFF  # Start with DC enabled
      on_turn_on:
        - logger.log: "DC output disabled"
      on_turn_off:
        - logger.log: "DC output enabled"
```

### Inverted Switch (for intuitive control)

```yaml
switch:
  - platform: emerson_r48
    dc_sw:
      id: dc_off_internal
      internal: true

  - platform: template
    name: "DC Output Enable"
    id: dc_enable
    lambda: |-
      return !id(dc_off_internal).state;
    turn_on_action:
      - switch.turn_off: dc_off_internal
    turn_off_action:
      - switch.turn_on: dc_off_internal
```

---

## Number Configuration

### Available Numbers

```yaml
number:
  - platform: emerson_r48
    output_voltage:
      name: "Set Output Voltage"
    max_output_current:
      name: "Set Current Limit"
    max_input_current:
      name: "Set Input Current Limit"
```

### Number Properties

| Number | Min | Max | Step | Unit |
|--------|-----|-----|------|------|
| `output_voltage` | 41.0 | 58.5 | 0.1 | V |
| `max_output_current` | 10 | 121 | 0.1 | % |
| `max_input_current` | 0 | 20 | 0.1 | A |

### Customizing Numbers

```yaml
number:
  - platform: emerson_r48
    output_voltage:
      name: "Set Output Voltage"
      id: set_voltage
      min_value: 48.0      # Override minimum
      max_value: 54.0      # Override maximum
      step: 0.5            # Override step
      mode: slider         # Or: box
      icon: "mdi:flash"
      on_value:
        then:
          - logger.log:
              format: "Voltage set to %.2f V"
              args: ['x']
```

### Current Limit Calculation

The `max_output_current` is specified as a percentage of the rated current (62.5A):

| Percentage | Actual Current |
|------------|----------------|
| 10% | 6.25 A |
| 50% | 31.25 A |
| 100% | 62.5 A |
| 121% | 75.625 A |

```yaml
# Display actual current instead of percentage
sensor:
  - platform: emerson_r48
    max_output_current:
      id: current_limit_pct
      internal: true

  - platform: template
    name: "Current Limit (A)"
    unit_of_measurement: "A"
    lambda: |-
      return id(current_limit_pct).state * 0.625;  // 62.5A / 100%
```

---

## Button Configuration

### Set Offline Values Button

```yaml
button:
  - platform: emerson_r48
    set_offline_values:
      name: "Save Settings to EEPROM"
```

### Button Behavior

When pressed, the button:
1. Reads current voltage setting from `output_voltage` number
2. Reads current current limit from `max_output_current` number
3. Sends both values with "offline" flag to store in EEPROM
4. Settings persist across power cycles

### Customizing Button

```yaml
button:
  - platform: emerson_r48
    set_offline_values:
      name: "Save Settings"
      id: save_button
      icon: "mdi:content-save"
      on_press:
        - logger.log: "Saving settings to EEPROM..."
        - delay: 1s
        - logger.log: "Settings saved!"
```

---

## Advanced Configuration

### Complete Example Configuration

```yaml
esphome:
  name: "emerson-r48-controller"
  friendly_name: "R48 Power Controller"

esp8266:
  board: nodemcuv2

external_components:
  - source: github://arnoutzw/esphome-emerson-vertiv-r48

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  manual_ip:
    static_ip: 192.168.1.50
    gateway: 192.168.1.1
    subnet: 255.255.255.0

api:
  encryption:
    key: !secret api_key

ota:
  password: !secret ota_password

logger:
  level: INFO

web_server:
  port: 80

# MQTT (optional, alternative to API)
mqtt:
  broker: !secret mqtt_host
  username: !secret mqtt_user
  password: !secret mqtt_pass
  topic_prefix: "power/emerson_r48"

spi:
  clk_pin: GPIO14
  mosi_pin: GPIO13
  miso_pin: GPIO12

canbus:
  - platform: mcp2515
    id: can
    cs_pin: GPIO15
    can_id: 0x0607FF83
    bit_rate: 125kbps
    use_extended_id: true
    mode: NORMAL

emerson_r48:
  canbus_id: can
  update_interval: 1s

sensor:
  - platform: emerson_r48
    input_voltage:
      name: "AC Voltage"
    input_current:
      name: "AC Current"
    input_power:
      name: "AC Power"
    output_voltage:
      name: "DC Voltage"
    output_current:
      name: "DC Current"
    output_power:
      name: "DC Power"
    output_temp:
      name: "Temperature"
    max_output_current:
      name: "Current Limit %"

number:
  - platform: emerson_r48
    output_voltage:
      name: "Set Voltage"
    max_output_current:
      name: "Set Current Limit"
    max_input_current:
      name: "Set AC Current Limit"

switch:
  - platform: emerson_r48
    dc_sw:
      name: "DC Off"
    ac_sw:
      name: "AC Off"
    fan_sw:
      name: "Fan Max"

button:
  - platform: emerson_r48
    set_offline_values:
      name: "Save to EEPROM"

# Status LED
status_led:
  pin:
    number: GPIO2
    inverted: true
```

### Multiple R48 Units

```yaml
canbus:
  - platform: mcp2515
    id: can_bus_1
    cs_pin: GPIO15
    # ... config for unit 1

  - platform: mcp2515
    id: can_bus_2
    cs_pin: GPIO16
    # ... config for unit 2

emerson_r48:
  - id: r48_unit1
    canbus_id: can_bus_1

  - id: r48_unit2
    canbus_id: can_bus_2

sensor:
  - platform: emerson_r48
    emerson_r48_id: r48_unit1
    output_voltage:
      name: "Unit 1 Voltage"

  - platform: emerson_r48
    emerson_r48_id: r48_unit2
    output_voltage:
      name: "Unit 2 Voltage"
```

### Home Assistant Automations

```yaml
# In Home Assistant's automations.yaml
- alias: "Low Voltage Alert"
  trigger:
    platform: numeric_state
    entity_id: sensor.dc_voltage
    below: 46
  action:
    - service: notify.mobile_app
      data:
        message: "R48 voltage dropped below 46V!"

- alias: "Auto-adjust voltage for temperature"
  trigger:
    platform: numeric_state
    entity_id: sensor.temperature
    above: 50
  action:
    - service: number.set_value
      target:
        entity_id: number.set_current_limit
      data:
        value: 80  # Reduce to 80%
```

---

## Troubleshooting

### Common Issues

#### No Communication with R48

**Symptoms:** All sensors show "Unknown" or NaN

**Checklist:**
1. Verify wiring (CAN_H, CAN_L, GND)
2. Check bit rate is 125kbps
3. Confirm extended ID mode is enabled
4. Verify MCP2515 crystal frequency matches `clock` setting
5. Check for proper termination (120Ω)

**Debug:**
```yaml
logger:
  level: DEBUG
  logs:
    emerson_r48: DEBUG
    mcp2515: DEBUG
    canbus: DEBUG
```

#### Intermittent Connection

**Symptoms:** Sensors occasionally show NaN, then recover

**Causes:**
1. Poor CAN bus wiring/termination
2. Update interval too fast
3. Electrical interference

**Solutions:**
- Use twisted pair for CAN_H/CAN_L
- Add 120Ω termination resistors
- Increase update_interval to 2s or 5s
- Add ferrite beads to CAN lines

#### SPI Communication Errors

**Symptoms:** Component fails to initialize, frequent resets

**Checklist:**
1. Verify SPI pin connections
2. Check 3.3V power to MCP2515
3. Ensure CS pin is correct
4. Try reducing SPI data_rate

```yaml
canbus:
  - platform: mcp2515
    data_rate: 4Mhz  # Reduce from 10Mhz
```

#### Values Not Persisting

**Symptoms:** Settings reset after power cycle

**Cause:** Not using the "Save to EEPROM" button

**Solution:**
After changing voltage/current settings:
1. Press the "Save to EEPROM" button
2. Wait for confirmation
3. Settings will now persist

### Diagnostic Sensors

```yaml
# Add status sensors for debugging
sensor:
  - platform: wifi_signal
    name: "WiFi Signal"
    update_interval: 60s

  - platform: uptime
    name: "Uptime"

text_sensor:
  - platform: wifi_info
    ip_address:
      name: "IP Address"

binary_sensor:
  - platform: status
    name: "Status"
```

### Factory Reset

If the R48 is in an unknown state:

1. Power cycle the R48 unit
2. Restart the ESP device
3. The sync/gimme5 sequence should re-establish communication

### Log Analysis

Example healthy log output:
```
[I][emerson_r48:123]: Setup complete, sync sent
[D][emerson_r48:156]: Requesting output voltage
[D][emerson_r48:337]: Received CAN frame: ID=0x060F8003
[D][emerson_r48:362]: Parameter 0x01 (output_voltage): 48.000000
[D][emerson_r48:156]: Requesting output current
[D][emerson_r48:337]: Received CAN frame: ID=0x060F8003
[D][emerson_r48:362]: Parameter 0x02 (output_current): 15.500000
```

Example error log:
```
[W][emerson_r48:142]: Timeout - no response in 5 intervals
[I][emerson_r48:143]: Attempting recovery...
[W][mcp2515:234]: TX buffer full, message dropped
```
