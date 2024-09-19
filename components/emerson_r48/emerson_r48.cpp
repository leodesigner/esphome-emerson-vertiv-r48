#include "emerson_r48.h"
#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace emerson_r48 {

static const char *const TAG = "emerson_r48";

static const uint32_t CAN_ID_REQUEST = 0x06000783;
static const uint32_t CAN_ID_DATA = 0x60f8003; // 0x0707F803;
static const uint32_t CAN_ID_SET = 0x0607FF83;

static const uint8_t EMR48_DATA_OUTPUT_V = 0x01;
static const uint8_t EMR48_DATA_OUTPUT_A = 0x02;
static const uint8_t EMR48_DATA_OUTPUT_AL = 0x03;
static const uint8_t EMR48_DATA_OUTPUT_T = 0x04;
static const uint8_t EMR48_DATA_OUTPUT_IV = 0x05;

static const uint8_t R48xx_DATA_INPUT_POWER = 0x70;
static const uint8_t R48xx_DATA_INPUT_FREQ = 0x71;
static const uint8_t R48xx_DATA_INPUT_CURRENT = 0x72;
static const uint8_t R48xx_DATA_OUTPUT_POWER = 0x73;
static const uint8_t R48xx_DATA_EFFICIENCY = 0x74;
static const uint8_t R48xx_DATA_OUTPUT_VOLTAGE = 0x75;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT_MAX = 0x76;
static const uint8_t R48xx_DATA_INPUT_VOLTAGE = 0x78;
static const uint8_t R48xx_DATA_OUTPUT_TEMPERATURE = 0x7F;
static const uint8_t R48xx_DATA_INPUT_TEMPERATURE = 0x80;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT = 0x81;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT1 = 0x82;

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
  if (cnt == 3) {
    ESP_LOGD(TAG, "Requesting supply voltage message");
    std::vector<uint8_t> data = {0x01, 0xF0, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
    this->canbus->send_data(CAN_ID_REQUEST, true, data);
  }

  if (cnt == 5) { cnt = 1; }


  // no new value for 5* intervall -> set sensors to NAN)
  if (millis() - lastUpdate_ > this->update_interval_ * 5) {
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

void EmersonR48Component::set_output_voltage(float value, bool offline) {
  uint8_t functionCode = 0x0;
  if (offline)
    functionCode += 1;
  int32_t raw = 1024.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  this->canbus->send_data(CAN_ID_SET, true, data);
}

void EmersonR48Component::set_max_output_current(float value, bool offline) {
  uint8_t functionCode = 0x3;
  if (offline)
    functionCode += 1;
  int32_t raw = 20.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  this->canbus->send_data(CAN_ID_SET, true, data);
}

void EmersonR48Component::set_offline_values() {
  if (output_voltage_number_) {
    set_output_voltage(output_voltage_number_->state, true);
  };
  if (max_output_current_number_) {
    set_max_output_current(max_output_current_number_->state, true);
  }
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
    switch (data[3]) {
      case EMR48_DATA_OUTPUT_V:
        conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Output voltage: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_A:
        conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_AL:
        conv_value = value / 1.0;
        this->publish_number_state_(this->max_output_current_number_, conv_value);
        ESP_LOGV(TAG, "Output current limit: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_T:
        conv_value = value / 1.0;
        this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Temperature: %f", conv_value);
        break;

      case EMR48_DATA_OUTPUT_IV:
        conv_value = value / 1.0;
        this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Input voltage: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_FREQ:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_frequency_sensor_, conv_value);
        ESP_LOGV(TAG, "Input frequency: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_CURRENT:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Input current: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_POWER:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_power_sensor_, conv_value);
        ESP_LOGV(TAG, "Output power: %f", conv_value);
        break;

      case R48xx_DATA_EFFICIENCY:
        conv_value = value / 1024.0 * 100;
        this->publish_sensor_state_(this->efficiency_sensor_, conv_value);
        ESP_LOGV(TAG, "Efficiency: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_VOLTAGE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Output voltage: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_CURRENT_MAX:
        conv_value = value / 20.0;
        this->publish_number_state_(this->max_output_current_number_, conv_value);
        ESP_LOGV(TAG, "Max Output current: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_VOLTAGE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Input voltage: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_TEMPERATURE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Output temperature: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_TEMPERATURE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Input temperature: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_CURRENT1:
        // printf("Output Current(1) %.02fA\r\n", value / 1024.0);
        // output_current = value / 1024.0;
        break;

      case R48xx_DATA_OUTPUT_CURRENT:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current: %f", conv_value);

        // this usually is the last message
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
