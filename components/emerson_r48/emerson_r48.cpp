#include "emerson_r48.h"
#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const char *const TAG = "emerson_r48";

static const float EMR48_OUTPUT_VOLTAGE_MIN = 41.0;
static const float EMR48_OUTPUT_VOLTAGE_MAX = 58.5;

static const float EMR48_OUTPUT_CURRENT_RATED_VALUE = 62.5;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN = 10;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX = 121;
static const float EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE = 121;
static const float EMR48_OUTPUT_CURRENT_MIN = 5.5; // 10%, rounded up to nearest 0.5A
static const float EMR48_OUTPUT_CURRENT_MAX = EMR48_OUTPUT_CURRENT_RATED_VALUE;

static const uint32_t CAN_ID_REQUEST = 0x06000783;
static const uint32_t CAN_ID_DATA = 0x60f8003; // 0x0707F803;
static const uint32_t CAN_ID_SET = 0x0607FF83; // set voltage and max current
static const uint32_t CAN_ID_SET_CTL = 0x06080783; // set control

static const uint8_t EMR48_DATA_OUTPUT_V = 0x01;
static const uint8_t EMR48_DATA_OUTPUT_A = 0x02;
static const uint8_t EMR48_DATA_OUTPUT_AL = 0x03;
static const uint8_t EMR48_DATA_OUTPUT_T = 0x04;
static const uint8_t EMR48_DATA_OUTPUT_IV = 0x05;

boolean dcOff = 0;
boolean fanFull = 0;
boolean flashLed = 0;
boolean acOff = 1;

EmersonR48Component::EmersonR48Component(canbus::Canbus *canbus) { this->canbus = canbus; }

void EmersonR48Component::setup() {
  Automation<std::vector<uint8_t>, uint32_t, bool> *automation;
  LambdaAction<std::vector<uint8_t>, uint32_t, bool> *lambdaaction;
  canbus::CanbusTrigger *canbus_canbustrigger;

  canbus_canbustrigger = new canbus::CanbusTrigger(this->canbus, 0, 0, true);
  canbus_canbustrigger->set_component_source("canbus");
  App.register_component(canbus_canbustrigger);
  automation = new Automation<std::vector<uint8_t>, uint32_t, bool>(canbus_canbustrigger);
  auto cb = [=](std::vector<uint8_t> x, uint32_t can_id, bool remote_transmission_request) -> void {
    this->on_frame(can_id, remote_transmission_request, x);
  };
  lambdaaction = new LambdaAction<std::vector<uint8_t>, uint32_t, bool>(cb);
  automation->add_actions({lambdaaction});
}

void EmersonR48Component::update() {
  static uint8_t cnt = 0;
  cnt++;

  if (cnt == 1) {
    ESP_LOGD(TAG, "Requesting output voltage message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 2) {
    ESP_LOGD(TAG, "Requesting output current message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 3) {
    ESP_LOGD(TAG, "Requesting output current limit message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 4) {
    ESP_LOGD(TAG, "Requesting temperature (C) message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }
  if (cnt == 5) {
    ESP_LOGD(TAG, "Requesting supply voltage message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }

  if (cnt == 5) { cnt = 0; }


  // no new value for 5* intervall -> set sensors to NAN)
  if (millis() - lastUpdate_ > this->update_interval_ * 10) {
    this->publish_sensor_state_(this->input_power_sensor_, NAN);
    this->publish_sensor_state_(this->input_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->input_current_sensor_, NAN);
    this->publish_sensor_state_(this->input_temp_sensor_, NAN);
    this->publish_sensor_state_(this->input_frequency_sensor_, NAN);
    this->publish_sensor_state_(this->output_power_sensor_, NAN);
    this->publish_sensor_state_(this->output_current_sensor_, NAN);
    this->publish_sensor_state_(this->output_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->output_temp_sensor_, NAN);
    this->publish_sensor_state_(this->efficiency_sensor_, NAN);
    this->publish_number_state_(this->max_output_current_number_, NAN);
  }
}

// Function to convert float to byte array
void float_to_bytearray(float value, uint8_t *bytes) {
    uint32_t temp;
    memcpy(&temp, &value, sizeof(temp));
    bytes[0] = (temp >> 24) & 0xFF; // Most significant byte
    bytes[1] = (temp >> 16) & 0xFF;
    bytes[2] = (temp >> 8) & 0xFF;
    bytes[3] = temp & 0xFF; // Least significant byte
}


// https://github.com/PurpleAlien/R48_Rectifier/blob/main/rectifier.py
// # Set the output voltage to the new value. 
// # The 'fixed' parameter 
// #  - if True makes the change permanent ('offline command')
// #  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
// # Voltage between 41.0 and 58.5V - fan will go high below 48V!
// def set_voltage(channel, voltage, fixed=False):
//    if OUTPUT_VOLTAGE_MIN <= voltage <= OUTPUT_VOLTAGE_MAX:
//        b = float_to_bytearray(voltage)
//        p = 0x21 if not fixed else 0x24
//        data = [0x03, 0xF0, 0x00, p, *b]
//        send_can_message(channel, data)
//    else:
//        print(f"Voltage should be between {OUTPUT_VOLTAGE_MIN}V and {OUTPUT_VOLTAGE_MAX}V")

void EmersonR48Component::set_output_voltage(float value, bool offline) {
  int32_t raw = 0;
  if (value > EMR48_OUTPUT_VOLTAGE_MIN && value < EMR48_OUTPUT_VOLTAGE_MAX) {
    memcpy(&raw, &value, sizeof(raw));
    uint8_t p = offline ? 0x21 : 0x24;
    std::vector<uint8_t> data = {
        0x03, 0xF0, 0x0, p, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
    this->canbus->send_data(CAN_ID_SET, true, data);

    size_t length = data.size();
    char buffer[3 * length + 1];

    // Format the data into the buffer
    size_t pos = 0;
    for (size_t i = 0; i < length; ++i) {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
    }

    // Log the entire line
    ESP_LOGD(TAG, "sent can_message.data: %s", buffer);

  } else {
    ESP_LOGD(TAG, "set output voltage is out of range: %f", value);
  }

}

//# The output current is set as a value
//# Possible values for 'current': 5.5A - 62.5A
//# The 'fixed' parameter
//#  - if True makes the change permanent ('offline command')
//#  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
//def set_current_value(channel, current, fixed=False): 
//    if OUTPUT_CURRENT_MIN <= current <= OUTPUT_CURRENT_MAX:
//        # 62.5A is the nominal current of Emerson/Vertiv R48-3000e and corresponds to 121%
//        percentage = (current/OUTPUT_CURRENT_RATED_VALUE)*OUTPUT_CURRENT_RATED_PERCENTAGE
//        set_current_percentage(channel , percentage, fixed)
//    else:
//       print(f"Current should be between {OUTPUT_CURRENT_MIN}A and {OUTPUT_CURRENT_MAX}A")

//# The output current is set in percent to the rated value of the rectifier written in the manual
//# Possible values for 'current': 10% - 121% (rated current in the datasheet = 121%)
//# The 'fixed' parameter
//#  - if True makes the change permanent ('offline command')
//#  - if False the change is temporary (30 seconds per command received, 'online command', repeat at 15 second intervals).
//def set_current_percentage(channel, current, fixed=False):
//    if OUTPUT_CURRENT_RATED_PERCENTAGE_MIN <= current <= OUTPUT_CURRENT_RATED_PERCENTAGE_MAX:
//        limit = current / 100
//        b = float_to_bytearray(limit)
//        p = 0x22 if not fixed else 0x19
//        data = [0x03, 0xF0, 0x00, p, *b]
//        send_can_message(channel, data)
//    else:
//        print(f"Current should be between {OUTPUT_CURRENT_RATED_PERCENTAGE_MIN}% and {OUTPUT_CURRENT_RATED_PERCENTAGE_MAX}%")

// Function to set current percentage
void EmersonR48Component::set_max_output_current(float value, bool offline) {

    if (value >= EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MIN && value <= EMR48_OUTPUT_CURRENT_RATED_PERCENTAGE_MAX) {
        float limit = value / 100.0f;
        uint8_t byte_array[4];
        float_to_bytearray(limit, byte_array);
        
        uint8_t p = offline ? 0x19 : 0x22;
        std::vector<uint8_t> data = { 0x03, 0xF0, 0x00, p, byte_array[0], byte_array[1], byte_array[2], byte_array[3] };
        
        this->canbus->send_data(CAN_ID_SET, true, data);

        size_t length = data.size();
        char buffer[3 * length + 1];

        // Format the data into the buffer
        size_t pos = 0;
        for (size_t i = 0; i < length; ++i) {
            pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
        }

        // Log the entire line
        ESP_LOGD(TAG, "sent can_message.data: %s", buffer);

    } else {
        ESP_LOGD(TAG, "Current should be between 10 and 121\n");
    }
}

/*
void EmersonR48Component::set_max_output_current2(float value, bool offline) {
  uint8_t functionCode = 0x3;
  if (offline)
    functionCode += 1;
  int32_t raw = 20.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  //this->canbus->send_data(CAN_ID_SET, true, data);
}
*/

//# AC input current limit (called Diesel power limit): gives the possibility to reduce the overall power of the rectifier
//def limit_input(channel, current):
//    b = float_to_bytearray(current)
//    data = [0x03, 0xF0, 0x00, 0x1A, *b]
//    send_can_message(channel, data)


void EmersonR48Component::set_offline_values() {
  if (output_voltage_number_) {
    set_output_voltage(output_voltage_number_->state, true);
  };
  if (max_output_current_number_) {
    set_max_output_current(max_output_current_number_->state, true);
  }
}

// https://github.com/anikrooz/Emerson-Vertiv-R48/blob/main/standalone/chargerManager/chargerManager.ino
void sendControl(){
   //control bits...
   uint8_t msg[8] = {0, 0xF0, 0, 0x80, 0, 0, 0, 0};
   msg[2] = dcOff << 7 | fanFull << 4 | flashLed <<3 | acOff << 2 | 1;
   //txId = 0x06080783; // CAN_ID_SET_CTL
   //sendcommand(txId, msg);

   //looptime = millis();
}


void EmersonR48Component::on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
  // Create a buffer to hold the formatted string
  // Each byte is represented by two hex digits and a space, +1 for null terminator
  size_t length = data.size();
  char buffer[3 * length + 1];

  // Format the data into the buffer
  size_t pos = 0;
  for (size_t i = 0; i < length; ++i) {
      pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02x ", data[i]);
  }

  // Log the entire line
  ESP_LOGD(TAG, "received can_message.data: %s", buffer);

  if (can_id == CAN_ID_DATA) {
    uint32_t value = (data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7];
    float conv_value = 0;
    memcpy(&conv_value, &value, sizeof(conv_value));
    switch (data[3]) {
      case EMR48_DATA_OUTPUT_V:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Output voltage: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_A:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_AL:
        //conv_value = value / 1.0;
        this->publish_number_state_(this->max_output_current_number_, conv_value);
        ESP_LOGV(TAG, "Output current limit: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_T:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Temperature: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_IV:
        //conv_value = value / 1.0;
        this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Input voltage: %f", conv_value);

        this->lastUpdate_ = millis();
        break;

      default:
        // printf("Unknown parameter 0x%02X, 0x%04X\r\n",frame[1], value);
        break;
    }
  }
}

void EmersonR48Component::publish_sensor_state_(sensor::Sensor *sensor, float value) {
  if (sensor) {
    sensor->publish_state(value);
  }
}

void EmersonR48Component::publish_number_state_(number::Number *number, float value) {
  if (number) {
    number->publish_state(value);
  }
}

}  // namespace huawei_r4850
}  // namespace esphome
