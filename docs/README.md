# ESPHome Emerson Vertiv R48 Documentation

This documentation provides comprehensive information about the ESPHome integration for Emerson/Vertiv R48 series rectifier power supplies.

## Table of Contents

| Document | Description |
|----------|-------------|
| [Architecture](ARCHITECTURE.md) | System architecture, component relationships, and data flow |
| [CAN Protocol](CAN-PROTOCOL.md) | CAN bus communication protocol specification |
| [Components](COMPONENTS.md) | Component interfaces, classes, and implementations |
| [Configuration](CONFIGURATION.md) | ESPHome YAML configuration guide |
| [API Reference](API.md) | Complete API reference for all classes and methods |

## Overview

The ESPHome Emerson Vertiv R48 integration allows you to monitor and control Emerson/Vertiv R48 series 48V DC rectifier power supplies via CAN bus. These industrial power supplies are commonly used in telecommunications, data centers, and battery charging applications.

### Supported Hardware

- **Power Supply**: Emerson/Vertiv R48-3000e3 (and compatible R48 series)
- **CAN Controller**: MCP2515 CAN bus controller via SPI
- **Microcontroller**: ESP8266/ESP32 (tested with ESP01_1M)

### Key Features

- **Real-time Monitoring**
  - Input: AC voltage, frequency, current, power, temperature
  - Output: DC voltage, current, power, temperature
  - Efficiency calculation

- **Control Capabilities**
  - Adjustable output voltage (41.0V - 58.5V)
  - Configurable output current limit (10% - 121%)
  - AC input current limiting
  - AC/DC output enable/disable
  - Fan speed control
  - LED indicator control

- **Persistence Options**
  - Online (temporary) parameter changes
  - Offline (permanent/EEPROM) storage

## Quick Start

1. **Hardware Setup**: Connect MCP2515 CAN controller to your ESP8266/ESP32 and wire to the R48 CAN bus
2. **Install Component**: Add this repository as an external component in ESPHome
3. **Configure YAML**: Create your configuration based on the example
4. **Flash & Deploy**: Compile and upload to your device

See the [Configuration Guide](CONFIGURATION.md) for detailed setup instructions.

## Project Structure

```
esphome-emerson-vertiv-r48/
├── components/
│   ├── emerson_r48/           # Main R48 component
│   │   ├── __init__.py        # ESPHome component initialization
│   │   ├── emerson_r48.h      # Main header file
│   │   ├── emerson_r48.cpp    # Core implementation
│   │   ├── sensor.py          # Sensor integration
│   │   ├── button/            # Button component (save settings)
│   │   ├── switch/            # Switch component (AC/DC/Fan/LED)
│   │   └── number/            # Number component (voltage/current)
│   └── mcp2515/               # CAN bus controller driver
├── docs/                      # Documentation (you are here)
├── emerson_r48_example.yaml   # Example configuration
└── README.md                  # Project readme
```

## Communication Overview

```
┌─────────────────┐     SPI      ┌─────────────┐    CAN Bus    ┌─────────────┐
│  ESP8266/ESP32  │◄────────────►│   MCP2515   │◄─────────────►│  Vertiv R48 │
│  (ESPHome)      │  10MHz       │  CAN Ctrl   │  125kbps      │  Rectifier  │
└─────────────────┘              └─────────────┘               └─────────────┘
```

## Safety Considerations

- The R48 power supplies handle high voltages and currents
- Always follow proper electrical safety procedures
- Test configurations on isolated systems before production deployment
- Use the "offline values" button to persist changes across power cycles

## License

This project is open source. See the repository for license details.

## Contributing

Contributions are welcome! Please read the architecture documentation to understand the codebase structure before submitting changes.
