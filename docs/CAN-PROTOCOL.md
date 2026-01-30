# CAN Bus Protocol Specification

This document describes the CAN bus communication protocol used to interface with the Emerson/Vertiv R48 series rectifier power supplies.

## Table of Contents

- [Overview](#overview)
- [Physical Layer](#physical-layer)
- [Message Identifiers](#message-identifiers)
- [Data Frame Formats](#data-frame-formats)
- [Parameter Codes](#parameter-codes)
- [Message Types](#message-types)
- [Float Encoding](#float-encoding)
- [Control Byte Format](#control-byte-format)
- [Protocol Sequence Examples](#protocol-sequence-examples)
- [Error Handling](#error-handling)

---

## Overview

The Vertiv R48 rectifier communicates via CAN 2.0B protocol using extended 29-bit identifiers. All messages use 8-byte data frames.

### Key Characteristics

| Property | Value |
|----------|-------|
| CAN Standard | CAN 2.0B (Extended) |
| Identifier Format | 29-bit Extended |
| Bit Rate | 125 kbps |
| Data Frame Length | 8 bytes |
| Byte Order | Big-endian |
| Float Format | IEEE 754 Single Precision |

---

## Physical Layer

### CAN Bus Wiring

```
┌────────────────┐                              ┌────────────────┐
│   MCP2515      │                              │   Vertiv R48   │
│   Controller   │                              │   Rectifier    │
│                │                              │                │
│      CAN_H ────┼──────────────────────────────┼──── CAN_H      │
│      CAN_L ────┼──────────────────────────────┼──── CAN_L      │
│      GND ──────┼──────────────────────────────┼──── GND        │
│                │                              │                │
└────────────────┘                              └────────────────┘
                        │                │
                      120Ω             120Ω
                    Termination      Termination
                    (if needed)      (built-in)
```

### SPI to CAN Controller

```
ESP8266/ESP32                    MCP2515
┌────────────┐                 ┌────────────┐
│            │                 │            │
│   GPIO14 ──┼── CLK ─────────►│ SCK        │
│   GPIO13 ──┼── MOSI ────────►│ SI         │
│   GPIO12 ◄─┼── MISO ────────►│ SO         │
│   GPIO15 ──┼── CS ──────────►│ CS         │
│            │                 │            │
│      3.3V ─┼─────────────────┼── VCC      │
│      GND ──┼─────────────────┼── GND      │
│            │                 │            │
└────────────┘                 └────────────┘
```

### Timing Configuration

```
CAN Bit Rate: 125 kbps
MCP2515 Clock: 8MHz (configurable: 8/12/16/20 MHz)
SPI Clock: 10 MHz

Bit Timing (8MHz oscillator, 125kbps):
  Sync Seg:     1 TQ
  Prop Seg:     3 TQ
  Phase Seg 1:  3 TQ
  Phase Seg 2:  3 TQ
  SJW:          1 TQ
  Total:        10 TQ per bit
  TQ = 800ns
```

---

## Message Identifiers

All CAN message identifiers are 29-bit extended format.

### Identifier Definitions

| Name | Hex Value | Direction | Purpose |
|------|-----------|-----------|---------|
| `CAN_ID_REQUEST` | `0x06000783` | TX | Request parameter data |
| `CAN_ID_DATA` | `0x060F8003` | RX | Data response from device |
| `CAN_ID_DATA2` | `0x060F8007` | RX | Extended data response |
| `CAN_ID_SET` | `0x0607FF83` | TX | Set parameter value |
| `CAN_ID_SET2` | `0x0677FF83` | TX | Alternative set command |
| `CAN_ID_SET_CTL` | `0x06080783` | TX | Set control flags |
| `CAN_ID_SYNC` | `0x0707FF83` | TX | Synchronization message |
| `CAN_ID_SYNC2` | `0x0717FF83` | TX | Synchronization message 2 |
| `CAN_ID_GIMME5` | `0x06080783` | TX | Request data packet |

### Identifier Bit Layout

```
29-bit Extended Identifier Format:

Bits [28:26] - Priority/Type field
Bits [25:16] - Address/Function field
Bits [15:8]  - Device identifier
Bits [7:0]   - Command/Parameter field

Example: 0x0607FF83
  Binary: 0000 0110 0000 0111 1111 1111 1000 0011

  [28:26] = 000  (Priority 0)
  [25:16] = 0110000001 (Function)
  [15:8]  = 11111111  (Broadcast/All devices)
  [7:0]   = 10000011  (Command type)
```

---

## Data Frame Formats

### General Frame Structure

```
┌──────────────────────────────────────────────────────────────┐
│                     8-Byte CAN Data Frame                     │
├────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ Byte 0 │ Byte 1 │ Byte 2 │ Byte 3 │ Byte 4 │ Byte 5 │ Byte 6 │ Byte 7 │
├────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤
│ Command│ Subaddr│ Resrvd │ Param  │  Data  │  Data  │  Data  │  Data  │
│  Type  │  0xF0  │  0x00  │  Code  │  MSB   │        │        │  LSB   │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
```

### Request Frame Format

```
ID: 0x06000783 (CAN_ID_REQUEST)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  0x01  │  0xF0  │  0x00  │ Param  │  0x00  │  0x00  │  0x00  │  0x00  │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
   │         │        │       │
   │         │        │       └─── Parameter code to request
   │         │        └─────────── Reserved (always 0x00)
   │         └──────────────────── Sub-address (always 0xF0)
   └────────────────────────────── Command type: Request (0x01)
```

### Response Frame Format

```
ID: 0x060F8003 (CAN_ID_DATA)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  Resp  │  0xF0  │  0x00  │ Param  │ Float  │ Float  │ Float  │ Float  │
│  Code  │        │        │  Code  │ Byte 3 │ Byte 2 │ Byte 1 │ Byte 0 │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                              │       │         │         │         │
                              │       └─────────┴─────────┴─────────┘
                              │                   │
                              │                   └─── IEEE 754 float (big-endian)
                              └─────────────────────── Parameter identifier
```

### Set Parameter Frame Format

```
ID: 0x0607FF83 (CAN_ID_SET)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  0x03  │  0xF0  │  0x00  │ Param  │ Float  │ Float  │ Float  │ Float  │
│        │        │        │  Code  │ Byte 3 │ Byte 2 │ Byte 1 │ Byte 0 │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
   │                          │       └─────────────────────────────────┘
   │                          │                      │
   │                          │                      └─── Value as IEEE 754 float
   │                          └─────────────────────────── Parameter (0x21/0x24/etc)
   └────────────────────────────────────────────────────── Command type: Set (0x03)
```

### Control Frame Format

```
ID: 0x06080783 (CAN_ID_SET_CTL)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  0x00  │  0xF0  │ Control│  0x80  │  0x00  │  0x00  │  0x00  │  0x00  │
│        │        │  Byte  │        │        │        │        │        │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                     │
                     └─── Control flags (see Control Byte Format section)
```

### Synchronization Frame Format

```
ID: 0x0707FF83 (CAN_ID_SYNC)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  0x04  │  0xF0  │  0x01  │  0x5A  │  0x00  │  0x00  │  0x00  │  0x00  │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
```

### Data Request (gimme5) Frame Format

```
ID: 0x06080783 (CAN_ID_GIMME5)

┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│  0x20  │  0xF0  │  0x00  │  0x80  │  0x00  │  0x00  │  0x00  │  0x00  │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
```

---

## Parameter Codes

### Read Parameters (Request Codes)

| Code | Hex | Description | Unit | Data Type |
|------|-----|-------------|------|-----------|
| OUTPUT_V | `0x01` | DC Output Voltage | Volts | Float |
| OUTPUT_A | `0x02` | DC Output Current | Amps | Float |
| OUTPUT_AL | `0x03` | Output Current Limit | Ratio (0-1.21) | Float |
| OUTPUT_T | `0x04` | Temperature | °C | Float |
| OUTPUT_IV | `0x05` | AC Input Voltage | Volts | Float |

### Write Parameters (Set Codes)

| Parameter | Online (RAM) | Offline (EEPROM) | Range | Unit |
|-----------|-------------|------------------|-------|------|
| Output Voltage | `0x21` | `0x24` | 41.0 - 58.5 | V |
| Output Current Limit | `0x22` | `0x19` | 0.10 - 1.21 | ratio |
| Input Current Limit | `0x1A` | `0x1A` | 0 - 20 | A |

### Parameter Value Ranges

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Output Voltage Range                                │
│                                                                          │
│    MIN                        DEFAULT                           MAX      │
│    41.0V                       48.0V                          58.5V      │
│    ├────────────────────────────┼────────────────────────────────┤      │
│    │◄─────────── Adjustable Range ────────────────────────────►│       │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                    Output Current Limit Range                            │
│                                                                          │
│    MIN                        100%                              MAX      │
│    10%                       (62.5A)                           121%      │
│    (6.25A)                                                    (75.6A)    │
│    ├────────────────────────────┼────────────────────────────────┤      │
│    │◄───────────────── Adjustable Range ─────────────────────►│        │
│                                                                          │
│    Note: Stored as ratio (0.10 to 1.21), displayed as percentage        │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Message Types

### 1. Request Message

Requests a specific parameter value from the rectifier.

**Sequence:**
```
Controller ──► Rectifier:  REQUEST (ID: 0x06000783)
                           Data: [0x01, 0xF0, 0x00, param, 0, 0, 0, 0]

Rectifier ──► Controller:  RESPONSE (ID: 0x060F8003)
                           Data: [resp, 0xF0, 0x00, param, float[3], float[2], float[1], float[0]]
```

**Example - Request Output Voltage:**
```
TX: ID=0x06000783, Data=[0x01, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
RX: ID=0x060F8003, Data=[0x02, 0xF0, 0x00, 0x01, 0x42, 0x40, 0x00, 0x00]
                                              (param)  (───── 48.0V ─────)
```

### 2. Set Parameter Message (Online)

Sets a parameter temporarily (stored in RAM, lost on power cycle).

**Frame:**
```
Controller ──► Rectifier:  SET (ID: 0x0607FF83)
                           Data: [0x03, 0xF0, 0x00, param, float[3], float[2], float[1], float[0]]
```

**Example - Set Output Voltage to 52.0V (Online):**
```
TX: ID=0x0607FF83, Data=[0x03, 0xF0, 0x00, 0x21, 0x42, 0x50, 0x00, 0x00]
                                          (0x21)  (───── 52.0V ─────)
```

### 3. Set Parameter Message (Offline)

Sets a parameter permanently (stored in EEPROM, persists across power cycles).

**Example - Set Output Voltage to 52.0V (Offline):**
```
TX: ID=0x0607FF83, Data=[0x03, 0xF0, 0x00, 0x24, 0x42, 0x50, 0x00, 0x00]
                                          (0x24)  (───── 52.0V ─────)
```

### 4. Control Message

Sets control flags for AC/DC switching, fan speed, and LED.

**Frame:**
```
Controller ──► Rectifier:  CONTROL (ID: 0x06080783)
                           Data: [0x00, 0xF0, ctrl, 0x80, 0x00, 0x00, 0x00, 0x00]
```

**Example - Turn DC Off:**
```
TX: ID=0x06080783, Data=[0x00, 0xF0, 0x81, 0x80, 0x00, 0x00, 0x00, 0x00]
                                     (ctrl=0x81: dcOff=1)
```

### 5. Synchronization Message

Re-establishes communication with the rectifier.

**Frame:**
```
Controller ──► Rectifier:  SYNC (ID: 0x0707FF83)
                           Data: [0x04, 0xF0, 0x01, 0x5A, 0x00, 0x00, 0x00, 0x00]
```

### 6. Data Request (gimme5)

Requests the rectifier to send all current data.

**Frame:**
```
Controller ──► Rectifier:  GIMME5 (ID: 0x06080783)
                           Data: [0x20, 0xF0, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00]
```

---

## Float Encoding

### IEEE 754 Single Precision Format

The R48 uses IEEE 754 single-precision floating-point numbers, transmitted in **big-endian** byte order.

```
IEEE 754 Single Precision (32 bits):

Bit:  31    30-23      22-0
      │      │          │
      │      │          └───── Mantissa (23 bits)
      │      └────────────────  Exponent (8 bits, biased by 127)
      └───────────────────────  Sign (0=positive, 1=negative)

Memory Layout (Big-Endian):
┌────────┬────────┬────────┬────────┐
│ Byte 3 │ Byte 2 │ Byte 1 │ Byte 0 │
│ (MSB)  │        │        │ (LSB)  │
└────────┴────────┴────────┴────────┘
   [31:24] [23:16]  [15:8]   [7:0]
```

### Conversion Functions

**C++ Implementation (float to bytes):**
```cpp
void float_to_bytearray(float value, uint8_t *bytes) {
    uint32_t temp;
    memcpy(&temp, &value, sizeof(temp));
    bytes[0] = (temp >> 24) & 0xFF;  // MSB
    bytes[1] = (temp >> 16) & 0xFF;
    bytes[2] = (temp >> 8) & 0xFF;
    bytes[3] = temp & 0xFF;          // LSB
}
```

**C++ Implementation (bytes to float):**
```cpp
float bytearray_to_float(uint8_t *bytes) {
    uint32_t temp = ((uint32_t)bytes[0] << 24) |
                    ((uint32_t)bytes[1] << 16) |
                    ((uint32_t)bytes[2] << 8) |
                    (uint32_t)bytes[3];
    float result;
    memcpy(&result, &temp, sizeof(result));
    return result;
}
```

### Common Value Encodings

| Decimal Value | Hex Bytes (BE) | Binary |
|---------------|----------------|--------|
| 41.0 V | `42 24 00 00` | 0 10000100 01001000000000000000000 |
| 48.0 V | `42 40 00 00` | 0 10000100 10000000000000000000000 |
| 52.0 V | `42 50 00 00` | 0 10000100 10100000000000000000000 |
| 58.5 V | `42 6A 00 00` | 0 10000100 11010100000000000000000 |
| 0.50 (50%) | `3F 00 00 00` | 0 01111110 00000000000000000000000 |
| 1.00 (100%) | `3F 80 00 00` | 0 01111111 00000000000000000000000 |
| 1.21 (121%) | `3F 9A E1 48` | 0 01111111 00110101110000101001000 |

---

## Control Byte Format

The control byte is located at data[2] in control messages.

### Bit Layout

```
Control Byte Structure:

Bit:   7      6      5      4      3      2      1      0
       │      │      │      │      │      │      │      │
       │      │      │      │      │      │      │      └── Always 1
       │      │      │      │      │      │      └───────── Reserved
       │      │      │      │      │      └──────────────── AC Off
       │      │      │      │      └─────────────────────── LED Flash
       │      │      │      └────────────────────────────── Fan Full Speed
       │      │      └───────────────────────────────────── Reserved
       │      └──────────────────────────────────────────── Reserved
       └─────────────────────────────────────────────────── DC Off
```

### Bit Definitions

| Bit | Name | 0 Value | 1 Value |
|-----|------|---------|---------|
| 7 | DC Off | DC Output ON | DC Output OFF |
| 4 | Fan Full | Normal Fan Speed | Maximum Fan Speed |
| 3 | LED Flash | LED Normal | LED Flashing |
| 2 | AC Off | AC Input ON | AC Input OFF |
| 0 | Always | Must be 1 | Must be 1 |

### Control Byte Calculation

```cpp
uint8_t buildControlByte(bool dcOff, bool fanFull, bool flashLed, bool acOff) {
    return (dcOff   << 7) |
           (fanFull << 4) |
           (flashLed << 3) |
           (acOff   << 2) |
           0x01;  // Bit 0 always set
}
```

### Example Control Bytes

| State | dcOff | fanFull | flashLed | acOff | Control Byte |
|-------|-------|---------|----------|-------|--------------|
| Normal operation | 0 | 0 | 0 | 0 | `0x01` |
| DC Off only | 1 | 0 | 0 | 0 | `0x81` |
| AC Off only | 0 | 0 | 0 | 1 | `0x05` |
| Fan max speed | 0 | 1 | 0 | 0 | `0x11` |
| LED flashing | 0 | 0 | 1 | 0 | `0x09` |
| All features active | 1 | 1 | 1 | 1 | `0x9D` |

---

## Protocol Sequence Examples

### Startup Sequence

```
Time    Direction   CAN ID       Data                          Description
────────────────────────────────────────────────────────────────────────────
T+0ms   TX          0x0707FF83   [04 F0 01 5A 00 00 00 00]     SYNC
T+10ms  TX          0x06080783   [20 F0 00 80 00 00 00 00]     GIMME5
T+50ms  RX          0x060F8003   [02 F0 00 01 42 40 00 00]     Output voltage: 48.0V
T+100ms TX          0x06000783   [01 F0 00 02 00 00 00 00]     Request current
T+150ms RX          0x060F8003   [02 F0 00 02 41 A0 00 00]     Output current: 20.0A
```

### Polling Cycle

```
Cycle 1 (T+0ms to T+1000ms):
────────────────────────────────────────────────────────────────────────────
T+0ms     TX  0x06000783  [01 F0 00 01 00 00 00 00]   Request output voltage
T+50ms    RX  0x060F8003  [02 F0 00 01 42 40 00 00]   Response: 48.0V

Cycle 2 (T+1000ms to T+2000ms):
────────────────────────────────────────────────────────────────────────────
T+1000ms  TX  0x06000783  [01 F0 00 02 00 00 00 00]   Request output current
T+1050ms  RX  0x060F8003  [02 F0 00 02 41 F0 00 00]   Response: 30.0A

Cycle 3 (T+2000ms to T+3000ms):
────────────────────────────────────────────────────────────────────────────
T+2000ms  TX  0x06000783  [01 F0 00 03 00 00 00 00]   Request current limit
T+2050ms  RX  0x060F8003  [02 F0 00 03 3F 80 00 00]   Response: 1.0 (100%)

Cycle 4 (T+3000ms to T+4000ms):
────────────────────────────────────────────────────────────────────────────
T+3000ms  TX  0x06000783  [01 F0 00 04 00 00 00 00]   Request temperature
T+3050ms  RX  0x060F8003  [02 F0 00 04 42 14 00 00]   Response: 37.0°C

Cycle 5 (T+4000ms to T+5000ms):
────────────────────────────────────────────────────────────────────────────
T+4000ms  TX  0x06000783  [01 F0 00 05 00 00 00 00]   Request input voltage
T+4050ms  RX  0x060F8003  [02 F0 00 05 43 69 00 00]   Response: 233.0V

Cycle 6 (T+5000ms to T+6000ms):
────────────────────────────────────────────────────────────────────────────
T+5000ms  TX  0x06080783  [00 F0 01 80 00 00 00 00]   Control message
```

### Set Voltage Sequence

```
Operator Action: Set output voltage to 52.0V (temporary)
────────────────────────────────────────────────────────────────────────────
T+0ms     TX  0x0607FF83  [03 F0 00 21 42 50 00 00]   Set voltage (online)
                                    │  │
                                    │  └── 52.0V as IEEE 754 float
                                    └───── 0x21 = online (RAM) parameter

Operator Action: Make setting permanent
────────────────────────────────────────────────────────────────────────────
T+5000ms  TX  0x0607FF83  [03 F0 00 24 42 50 00 00]   Set voltage (offline)
                                    │
                                    └───── 0x24 = offline (EEPROM) parameter
```

### Error Recovery Sequence

```
Scenario: No response received for 5 seconds
────────────────────────────────────────────────────────────────────────────
T+0ms     TX  0x06000783  [01 F0 00 01 00 00 00 00]   Request output voltage
T+5000ms  --  Timeout detected, no response
T+5001ms  --  Publish NaN to all sensors
T+5010ms  TX  0x0707FF83  [04 F0 01 5A 00 00 00 00]   SYNC (recovery)
T+5020ms  TX  0x06080783  [20 F0 00 80 00 00 00 00]   GIMME5 (recovery)
T+5100ms  RX  0x060F8003  [02 F0 00 01 42 40 00 00]   Response received
T+5101ms  --  Normal operation resumed
```

---

## Error Handling

### No Response

If no response is received within the timeout period (default: 5 × update_interval):
1. All sensor values are published as NaN
2. SYNC message is sent
3. GIMME5 message is sent
4. Polling continues normally

### Invalid Data

Response data is validated before publishing:
- Float values are checked for NaN/Inf
- Parameter codes are verified against expected values
- Out-of-range values are logged but may still be published

### CAN Bus Errors

The MCP2515 controller handles these automatically:
- Bit errors → automatic retransmission
- Acknowledgment errors → logged, retransmission attempted
- Bus-off recovery → automatic recovery with delay

### Debugging

Enable debug logging in ESPHome to see all CAN traffic:

```yaml
logger:
  level: DEBUG
  logs:
    emerson_r48: DEBUG
    mcp2515: DEBUG
```

Example debug output:
```
[D][emerson_r48:337]: Received CAN frame: ID=0x060F8003, Data=[02 F0 00 01 42 40 00 00]
[D][emerson_r48:362]: Parameter 0x01 (output_voltage): 48.000000
```
