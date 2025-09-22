#include "ds248x_temperature_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

namespace esphome {

namespace ds248x {

static const uint8_t DALLAS_MODEL_DS18S20 = 0x10;
static const uint8_t DALLAS_MODEL_DS1822 = 0x22;
static const uint8_t DALLAS_MODEL_DS18B20 = 0x28;
static const uint8_t DALLAS_MODEL_DS1825 = 0x3B;
static const uint8_t DALLAS_MODEL_DS28EA00 = 0x42;

static const uint8_t DALLAS_COMMAND_START_CONVERSION = 0x44;
static const uint8_t DALLAS_COMMAND_READ_SCRATCH_PAD = 0xBE;
static const uint8_t DALLAS_COMMAND_WRITE_SCRATCH_PAD = 0x4E;
static const uint8_t DALLAS_COMMAND_SAVE_EEPROM = 0x48;

static const char *const TAG = "ds248x";

void DS248xTemperatureSensor::set_address(uint64_t address) { this->address_ = address; }

void DS248xTemperatureSensor::set_channel(uint8_t channel) { this->channel_ = channel; }

uint8_t DS248xTemperatureSensor::get_channel() const { return this->channel_; }

uint8_t DS248xTemperatureSensor::get_resolution() const { return this->resolution_; }

void DS248xTemperatureSensor::set_resolution(uint8_t resolution) { this->resolution_ = resolution; }

optional<uint8_t> DS248xTemperatureSensor::get_index() const { return this->index_; }

void DS248xTemperatureSensor::set_index(uint8_t index) { this->index_ = index; }

uint8_t *DS248xTemperatureSensor::get_address8() { return reinterpret_cast<uint8_t *>(&this->address_); }

const std::string &DS248xTemperatureSensor::get_address_name() {
  if (this->address_name_.empty()) {
    this->address_name_ = std::string("0x") + format_hex(this->address_);
  }

  return this->address_name_;
}

uint16_t DS248xTemperatureSensor::millis_to_wait_for_conversion() const {
  switch (this->resolution_) {
    case 9:
      return 94;
    case 10:
      return 188;
    case 11:
      return 375;
    default:
      return 750;
  }
}

bool IRAM_ATTR DS248xTemperatureSensor::read_scratch_pad() {
  this->total_reads_++;

  // Retry mechanism for reading scratch pad
  for (int retry = 0; retry < 3; retry++) {
    bool result = this->parent_->reset_devices_();
    if (!result) {
      ESP_LOGW(TAG, "Reset failed on attempt %d", retry + 1);
      delayMicroseconds(1000);  // Wait before retry
      continue;
    }

    this->parent_->select_(this->address_);
    this->parent_->write_to_wire_(DALLAS_COMMAND_READ_SCRATCH_PAD);

    // Add small delay after command
    delayMicroseconds(100);

    for (uint8_t &i : this->scratch_pad_) {
      i = this->parent_->read_from_wire_();
    }

    // Validate the read was successful by checking if we got non-zero data
    bool valid_read = false;
    for (int i = 0; i < 9; i++) {
      if (this->scratch_pad_[i] != 0x00 && this->scratch_pad_[i] != 0xFF) {
        valid_read = true;
        break;
      }
    }

    if (valid_read) {
      this->last_good_read_time_ = millis();
      return true;
    }

    ESP_LOGW(TAG, "Invalid scratch pad data on attempt %d", retry + 1);
    delayMicroseconds(1000);
  }

  this->failed_reads_++;
  this->parent_->status_set_warning();
  ESP_LOGE(TAG, "Failed to read scratch pad after 3 attempts");
  return false;
}

bool DS248xTemperatureSensor::setup_sensor() {
  bool r = this->read_scratch_pad();

  if (!r) {
    ESP_LOGE(TAG, "Reading scratchpad failed");
    return false;
  }
  if (!this->check_scratch_pad())
    return false;

  uint8_t resolution_register_val;
  switch (this->resolution_) {
    case 12:
      resolution_register_val = 0x7F;
      break;
    case 11:
      resolution_register_val = 0x5F;
      break;
    case 10:
      resolution_register_val = 0x3F;
      break;
    case 9:
    default:
      resolution_register_val = 0x1F;
      break;
  }

  if (this->scratch_pad_[4] == resolution_register_val)
    return true;

  if (this->get_address8()[0] == DALLAS_MODEL_DS18S20) {
    // DS18S20 doesn't support resolution.
    ESP_LOGW(TAG, "DS18S20 doesn't support setting resolution.");
    return false;
  }

  bool result = this->parent_->reset_devices_();
  if (!result) {
    ESP_LOGE(TAG, "Reset failed");
    return false;
  }

  this->parent_->select_(this->address_);
  this->parent_->write_to_wire_(DALLAS_COMMAND_WRITE_SCRATCH_PAD);
  this->parent_->write_to_wire_(this->scratch_pad_[2]);  // high alarm temp
  this->parent_->write_to_wire_(this->scratch_pad_[3]);  // low alarm temp
  this->parent_->write_to_wire_(this->scratch_pad_[4]);  // resolution

  result = this->parent_->reset_devices_();
  if (!result) {
    ESP_LOGE(TAG, "Reset failed");
    return false;
  }

  this->parent_->select_(this->address_);
  this->parent_->write_to_wire_(DALLAS_COMMAND_SAVE_EEPROM);

  delay(20);  // allow it to finish operation

  result = this->parent_->reset_devices_();
  if (!result) {
    ESP_LOGE(TAG, "Reset failed");
    return false;
  }
  return true;
}

bool DS248xTemperatureSensor::check_scratch_pad() {
  uint8_t calculated_crc = crc8(this->scratch_pad_, 8);
  bool chksum_validity = (calculated_crc == this->scratch_pad_[8]);
  bool config_validity = false;

  switch (this->get_address8()[0]) {
    case DALLAS_MODEL_DS18B20:
      config_validity = ((this->scratch_pad_[4] & 0x9F) == 0x1F);
      break;
    default:
      config_validity = ((this->scratch_pad_[4] & 0x10) == 0x10);
  }

  // Always log scratch pad contents for debugging checksum issues
  ESP_LOGD(TAG, "'%s' Scratch pad: %02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X (CRC calc=%02X, rx=%02X)",
           this->get_name().c_str(),
           this->scratch_pad_[0], this->scratch_pad_[1], this->scratch_pad_[2], this->scratch_pad_[3],
           this->scratch_pad_[4], this->scratch_pad_[5], this->scratch_pad_[6], this->scratch_pad_[7],
           this->scratch_pad_[8], calculated_crc, this->scratch_pad_[8]);

  if (!chksum_validity) {
    this->checksum_errors_++;
    ESP_LOGW(TAG, "'%s' - Scratch pad checksum invalid! Expected %02X, got %02X (Error #%u)",
             this->get_name().c_str(), calculated_crc, this->scratch_pad_[8], this->checksum_errors_);

    // Check for common error patterns
    bool all_zeros = true;
    bool all_ones = true;
    for (int i = 0; i < 9; i++) {
      if (this->scratch_pad_[i] != 0x00) all_zeros = false;
      if (this->scratch_pad_[i] != 0xFF) all_ones = false;
    }

    if (all_zeros) {
      ESP_LOGW(TAG, "'%s' - All zeros pattern detected (bus read failure)", this->get_name().c_str());
    } else if (all_ones) {
      ESP_LOGW(TAG, "'%s' - All ones pattern detected (no device response)", this->get_name().c_str());
    }

  } else if (!config_validity) {
    ESP_LOGW(TAG, "'%s' - Scratch pad config register invalid! Config byte: %02X",
             this->get_name().c_str(), this->scratch_pad_[4]);
  }

  return chksum_validity && config_validity;
}
float DS248xTemperatureSensor::get_temp_c() {
  int16_t temp = (int16_t(this->scratch_pad_[1]) << 11) | (int16_t(this->scratch_pad_[0]) << 3);
  if (this->get_address8()[0] == DALLAS_MODEL_DS18S20) {
    int diff = (this->scratch_pad_[7] - this->scratch_pad_[6]) << 7;
    temp = ((temp & 0xFFF0) << 3) - 16 + (diff / this->scratch_pad_[7]);
  }

  return temp / 128.0f;
}
std::string DS248xTemperatureSensor::unique_id() { return "dallas-" + str_lower_case(format_hex(this->address_)); }

void DS248xTemperatureSensor::log_sensor_stats() {
  float success_rate = this->get_success_rate();
  uint32_t time_since_last_good = millis() - this->last_good_read_time_;

  ESP_LOGI(TAG, "'%s' Stats: Reads=%u, Failed=%u, CRC_Errors=%u, Success=%.1f%%, LastGood=%ums ago",
           this->get_name().c_str(), this->total_reads_, this->failed_reads_,
           this->checksum_errors_, success_rate, time_since_last_good);

  if (success_rate < 80.0) {
    ESP_LOGW(TAG, "'%s' - Poor sensor reliability detected! Check wiring/connections", this->get_name().c_str());
  }
}

void DS248xTemperatureSensor::reset_error_stats() {
  this->total_reads_ = 0;
  this->failed_reads_ = 0;
  this->checksum_errors_ = 0;
  this->last_good_read_time_ = millis();
}

float DS248xTemperatureSensor::get_success_rate() const {
  if (this->total_reads_ == 0) return 100.0;
  return ((float)(this->total_reads_ - this->failed_reads_) / this->total_reads_) * 100.0;
}

}  // namespace ds248x

}  // namespace esphome
