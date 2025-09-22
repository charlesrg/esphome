#include "ds248x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32
#include <esp32-hal-cpu.h>
#elif defined(USE_ESP8266)
#include <Arduino.h>
#elif defined(USE_RP2040)
#include <hardware/timer.h>
#endif

namespace esphome {

namespace ds248x {

static const uint8_t DS2482_800_COMMAND_CHANNEL_SELECTION = 0xC3;

static const uint8_t DS248X_COMMAND_RESET = 0xF0;
static const uint8_t DS248X_COMMAND_SETREADPTR = 0xE1;
static const uint8_t DS248X_COMMAND_WRITECONFIG = 0xD2;
static const uint8_t DS248X_COMMAND_RESETWIRE = 0xB4;
static const uint8_t DS248X_COMMAND_WRITEBYTE = 0xA5;
static const uint8_t DS248X_COMMAND_READBYTE = 0x96;
static const uint8_t DS248X_COMMAND_SINGLEBIT = 0x87;
static const uint8_t DS248X_COMMAND_TRIPLET = 0x78;

static const uint8_t DS248X_POINTER_STATUS = 0xF0;
static const uint8_t DS248X_STATUS_BUSY = (1 << 0);
static const uint8_t DS248X_STATUS_PPD = (1 << 1);
static const uint8_t DS248X_STATUS_SD = (1 << 2);
static const uint8_t DS248X_STATUS_LL = (1 << 3);
static const uint8_t DS248X_STATUS_RST = (1 << 4);
static const uint8_t DS248X_STATUS_SBR = (1 << 5);
static const uint8_t DS248X_STATUS_TSB = (1 << 6);
static const uint8_t DS248X_STATUS_DIR = (1 << 7);

static const uint8_t DS248X_POINTER_DATA = 0xE1;

static const uint8_t DS248X_POINTER_CONFIG = 0xC3;
static const uint8_t DS248X_CONFIG_ACTIVE_PULLUP = (1 << 0);
static const uint8_t DS248X_CONFIG_POWER_DOWN = (1 << 1);
static const uint8_t DS248X_CONFIG_STRONG_PULLUP = (1 << 2);
static const uint8_t DS248X_CONFIG_1WIRE_SPEED = (1 << 3);

static const uint8_t WIRE_COMMAND_SKIP = 0xCC;
static const uint8_t WIRE_COMMAND_SELECT = 0x55;
static const uint8_t WIRE_COMMAND_SEARCH = 0xF0;

static const uint8_t DS248X_ERROR_TIMEOUT = (1 << 0);
static const uint8_t DS248X_ERROR_SHORT = (1 << 1);
static const uint8_t DS248X_ERROR_CONFIG = (1 << 2);

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

static const uint8_t CHANNEL_CODE[8] = {0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87};
static const uint8_t READ_CHANNEL_CODE[8] = {0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87};

void DS248xComponent::setup() {
  uint64_t address = 0;
  uint8_t channel = 0;
  uint8_t index = 0;
  std::vector<uint64_t> raw_sensors;
  std::vector<uint8_t> raw_channel_sensors;

  ESP_LOGCONFIG(TAG, "Setting up DS248x...");
  this->last_diagnostic_time_ = millis();

  if (this->sleep_pin_ != nullptr) {
    this->sleep_pin_->setup();
    this->sleep_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  }

  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    // Reset
    this->reset_hub_();
    address = 0;
    while (this->search_(&address)) {
      raw_sensors.push_back(address);
      raw_channel_sensors.push_back(0);
    }
  }

  if (this->ds248x_type_ == DS248xType::DS2482_800) {
    for (channel = 0; channel < NBR_CHANNELS; channel++) {
      // Reset
      this->reset_hub_();

      // 1-wire channel selection
      if (this->set_channel_(channel)) {
        // Search 1-wire components
        address = 0;
        while (this->search_(&address)) {
          raw_sensors.push_back(address);
          raw_channel_sensors.push_back(channel);
        }
      }
    }
    this->reset_hub_();
    this->set_channel_(0);
  }

  index = 0;
  for (auto &address : raw_sensors) {
    channel = raw_channel_sensors[index];
    index++;
    auto *address8 = reinterpret_cast<uint8_t *>(&address);
    if (crc8(address8, 7) != address8[7]) {
      ESP_LOGW(TAG, "Dallas device 0x%s has invalid CRC.", format_hex(address).c_str());
      continue;
    }
    if (address8[0] != DALLAS_MODEL_DS18S20 && address8[0] != DALLAS_MODEL_DS1822 &&
        address8[0] != DALLAS_MODEL_DS18B20 && address8[0] != DALLAS_MODEL_DS1825 &&
        address8[0] != DALLAS_MODEL_DS28EA00) {
      ESP_LOGW(TAG, "Unknown device type 0x%02X.", address8[0]);
      continue;
    }
    this->found_sensors_.push_back(address);
    this->found_channel_sensors_.push_back(channel);
  }

  // DS2482_100
  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    for (auto *sensor : this->sensors_) {
      if (sensor->get_index().has_value()) {
        if (*sensor->get_index() >= this->found_sensors_.size()) {
          this->status_set_error();
          continue;
        }
        sensor->set_address(this->found_sensors_[*sensor->get_index()]);
        // sensor->set_channel(this->found_channel_sensors_[*sensor->get_index()]);
        sensor->set_channel(0);
      }

      if (!sensor->setup_sensor()) {
        this->status_set_error();
      }
    }
  }

  // DS2482_800
  if (this->ds248x_type_ == DS248xType::DS2482_800) {
    for (channel = 0; channel < NBR_CHANNELS; channel++) {
      for (auto *sensor : this->channel_sensors_[channel]) {
        if (sensor->get_index().has_value()) {
          if (*sensor->get_index() >= this->found_sensors_.size()) {
            this->status_set_error();
            continue;
          }
          sensor->set_address(this->found_sensors_[*sensor->get_index()]);
          // sensor->set_channel(this->found_channel_sensors_[*sensor->get_index()]);
          sensor->set_channel(channel);
        }

        if (!sensor->setup_sensor()) {
          this->status_set_error();
        }
      }
    }
  }

  // Setup diagnostic reporting
  this->setup_diagnostic_timer_();
}

void DS248xComponent::dump_config() {
  int idx = 0;
  uint8_t channel = 0;
  ESP_LOGCONFIG(TAG, "DS248x:");
  if (this->sleep_pin_) {
    LOG_PIN("  Sleep Pin: ", this->sleep_pin_);
  }
  LOG_I2C_DEVICE(this);

  switch (this->ds248x_type_) {
    case DS248xType::DS2482_100:
      ESP_LOGCONFIG(TAG, "  Type: DD2482-100");
      break;
    case DS248xType::DS2482_800:
      ESP_LOGCONFIG(TAG, "  Type: DS2482-800");
      break;
  }

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with DS248x failed!");
  }
  if (this->found_sensors_.empty()) {
    ESP_LOGW(TAG, "  Found no sensors!");
  } else {
    ESP_LOGD(TAG, "  Found sensors:");
    for (auto &address : this->found_sensors_) {
      channel = this->found_channel_sensors_[idx++];
      ESP_LOGD(TAG, "    0x%s (channel=%u)", format_hex(address).c_str(), channel);
    }
  }
  LOG_UPDATE_INTERVAL(this);

  // DS2482-100
  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    for (auto *sensor : this->sensors_) {
      LOG_SENSOR("  ", "Device", sensor);
      if (sensor->get_index().has_value()) {
        ESP_LOGCONFIG(TAG, "    Index %u", *sensor->get_index());
        if (*sensor->get_index() >= this->found_sensors_.size()) {
          ESP_LOGE(TAG, "Couldn't find sensor by index - not connected. Proceeding without it.");
          continue;
        }
      }
      ESP_LOGCONFIG(TAG, "    Address: %s", sensor->get_address_name().c_str());
      ESP_LOGCONFIG(TAG, "    Channel: %u", sensor->get_channel());
      ESP_LOGCONFIG(TAG, "    Resolution: %u", sensor->get_resolution());
    }
  }

  // DS2482-800
  if (this->ds248x_type_ == DS248xType::DS2482_800) {
    for (channel = 0; channel < NBR_CHANNELS; channel++) {
      for (auto *sensor : this->channel_sensors_[channel]) {
        LOG_SENSOR("  ", "Device", sensor);
        if (sensor->get_index().has_value()) {
          ESP_LOGCONFIG(TAG, "    Index %u", *sensor->get_index());
          if (*sensor->get_index() >= this->found_sensors_.size()) {
            ESP_LOGE(TAG, "Couldn't find sensor by index - not connected. Proceeding without it.");
            continue;
          }
        }
        ESP_LOGCONFIG(TAG, "    Address: %s", sensor->get_address_name().c_str());
        ESP_LOGCONFIG(TAG, "    Channel: %u", sensor->get_channel());
        ESP_LOGCONFIG(TAG, "    Resolution: %u", sensor->get_resolution());
      }
    }
  }
}

void DS248xComponent::register_sensor(DS248xTemperatureSensor *sensor) {
  this->sensors_.push_back(sensor);

  // DS2482-100
  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    this->channel_sensors_[0].push_back(sensor);
  }

  // DS2482-800
  if (this->ds248x_type_ == DS248xType::DS2482_800) {
    this->channel_sensors_[sensor->get_channel()].push_back(sensor);
  }
}

void DS248xComponent::update() {
  uint8_t channel = 0;
  uint8_t nbr_channels = 1;
  int nbr_sensors_on_channel = 0;

  if (this->ds248x_type_ == DS248xType::DS2482_800) {
    nbr_channels = NBR_CHANNELS;
  }

  for (channel = 0; channel < nbr_channels; channel++) {
    if (this->ds248x_type_ == DS248xType::DS2482_800) {
      ESP_LOGV(TAG, "Channel: %u", channel);
    }

    nbr_sensors_on_channel = channel_sensors_[channel].size();

    ESP_LOGV(TAG, "Start sensor update for %i sensors", nbr_sensors_on_channel);

    this->status_clear_warning();
    if (nbr_sensors_on_channel && this->set_channel_(channel)) {
      ESP_LOGD(TAG, "Starting conversion on channel %u with %d sensors", channel, nbr_sensors_on_channel);

      if (this->enable_bus_sleep_) {
        this->write_config_(this->read_config_() & ~DS248X_CONFIG_POWER_DOWN);
        delayMicroseconds(200);  // Allow bus to wake up
      }

      // Multiple reset attempts for reliability
      bool result = false;
      for (int reset_attempts = 0; reset_attempts < 3; reset_attempts++) {
        result = this->reset_devices_();
        if (result) {
          break;
        }
        ESP_LOGW(TAG, "Reset failed on attempt %d for channel %u", reset_attempts + 1, channel);
        delayMicroseconds(1000);
      }

      if (!result) {
        this->status_set_warning();
        ESP_LOGE(TAG, "All reset attempts failed for channel %u", channel);
        return;
      }

      delayMicroseconds(100);  // Additional settling after reset

      this->write_to_wire_(WIRE_COMMAND_SKIP);
      delayMicroseconds(50);  // Ensure command is processed

      if (this->enable_strong_pullup_) {
        this->write_config_(this->read_config_() | DS248X_CONFIG_STRONG_PULLUP);
        delayMicroseconds(100);  // Allow strong pullup to engage
      }

      this->write_to_wire_(DALLAS_COMMAND_START_CONVERSION);
      delayMicroseconds(200);  // Ensure conversion command is sent

      ESP_LOGV(TAG, "Conversion started on channel %u", channel);
    } else {
      ESP_LOGW(TAG, "Cannot change to channel %u (nbr_sensors=%d)", channel, nbr_sensors_on_channel);
    }
  }

  uint16_t max_wait_time = 0;

  // Calculate max wait time across all sensors that will be read
  for (auto *sensor : this->sensors_) {
    auto sensor_wait_time = sensor->millis_to_wait_for_conversion();
    if (max_wait_time < sensor_wait_time) {
      max_wait_time = sensor_wait_time;
    }
  }

  read_idx_ = 0;

  this->set_timeout(TAG, max_wait_time, [this] {
    ESP_LOGV(TAG, "Sensors read completed");
    this->set_interval(TAG, 50, [this]() {
      if (sensors_.size() <= read_idx_) {
        if (this->enable_bus_sleep_) {
          this->write_config_(this->read_config_() | DS248X_CONFIG_POWER_DOWN);
        }
        this->cancel_interval(TAG);
        return;
      }
      ESP_LOGV(TAG, "Update Sensor idx: %i", read_idx_);

      DS248xTemperatureSensor *sensor = sensors_[read_idx_];

      // Ensure we're on the correct channel before reading
      if (!this->set_channel_(sensor->get_channel())) {
        ESP_LOGW(TAG, "Failed to set channel %u for sensor %s", sensor->get_channel(), sensor->get_name().c_str());
        sensor->publish_state(NAN);
        this->status_set_warning();
        read_idx_++;
        return;
      }

      // Additional settling time after channel switch for problematic sensors
      delayMicroseconds(500);

      read_idx_++;

      // Multiple attempts with aggressive bus recovery
      bool res = false;
      for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
          ESP_LOGD(TAG, "'%s' - Retry attempt %d after failed read", sensor->get_name().c_str(), attempt + 1);

          // Aggressive bus recovery between retries
          this->reset_hub_();
          delayMicroseconds(1000);

          if (!this->set_channel_(sensor->get_channel())) {
            ESP_LOGW(TAG, "Failed to set channel %u on retry %d", sensor->get_channel(), attempt + 1);
            continue;
          }
          delayMicroseconds(500);
        }

        res = sensor->read_scratch_pad();
        if (res && sensor->check_scratch_pad()) {
          break;  // Success!
        }

        // Log detailed failure info
        if (!res) {
          ESP_LOGW(TAG, "'%s' - Read scratch pad failed on attempt %d", sensor->get_name().c_str(), attempt + 1);
        } else {
          ESP_LOGW(TAG, "'%s' - Scratch pad validation failed on attempt %d", sensor->get_name().c_str(), attempt + 1);
        }

        if (attempt < 2) {
          delayMicroseconds(2000);  // Wait before retry
        }
      }

      if (!res || !sensor->check_scratch_pad()) {
        ESP_LOGE(TAG, "'%s' - All read attempts failed, bus may need reset", sensor->get_name().c_str());
        sensor->publish_state(NAN);
        this->status_set_warning();

        // Force bus reset for next sensor
        this->reset_hub_();
        return;
      }

      float tempc = sensor->get_temp_c();
      ESP_LOGD(TAG, "'%s': Got Temperature=%.1f°C", sensor->get_name().c_str(), tempc);
      sensor->publish_state(tempc);
    });
  });
}

float DS248xComponent::get_setup_priority() const { return setup_priority::DATA; }

uint8_t DS248xComponent::read_config_() {
  std::array<uint8_t, 2> cmd;
  cmd[0] = DS248X_COMMAND_SETREADPTR;
  cmd[1] = DS248X_POINTER_CONFIG;
  auto err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to set config read pointer: %d", err);
    return 0;
  }

  uint8_t cfg_byte;
  err = this->read(&cfg_byte, sizeof(cfg_byte));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read config: %d", err);
    return 0;
  }

  return cfg_byte;
}

void DS248xComponent::write_config_(uint8_t cfg) {
  std::array<uint8_t, 2> cmd;
  cmd[0] = DS248X_COMMAND_WRITECONFIG;
  cmd[1] = cfg | ((~cfg) << 4);  // NOLINT
  auto err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write config: %d", err);
    return;
  }
  delayMicroseconds(100);  // Allow config write to settle
}

uint8_t DS248xComponent::wait_while_busy_() {
  std::array<uint8_t, 2> cmd;
  cmd[0] = DS248X_COMMAND_SETREADPTR;
  cmd[1] = DS248X_POINTER_STATUS;
  auto err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write status pointer command: %d", err);
    return DS248X_STATUS_BUSY;  // Return busy to indicate error
  }

  uint8_t status;
  for (int i = 1000; i > 0; i--) {
    err = this->read(&status, sizeof(status));
    if (err != esphome::i2c::ERROR_OK) {
      ESP_LOGE(TAG, "Failed to read status: %d", err);
      return DS248X_STATUS_BUSY;  // Return busy to indicate error
    }
    if (!(status & DS248X_STATUS_BUSY))
      break;
    delayMicroseconds(10);  // Small delay between status checks
  }
  return status;
}

void DS248xComponent::reset_hub_() {
  if (this->sleep_pin_ != nullptr) {
    this->sleep_pin_->digital_write(true);
    delayMicroseconds(100);  // Allow sleep pin to settle
  }

  uint8_t cmd = DS248X_COMMAND_RESET;
  auto err = this->write(&cmd, sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to reset hub: %d", err);
    this->status_set_error();
    return;
  }

  delayMicroseconds(500);  // Allow reset to complete

  if (this->enable_active_pullup_) {
    this->DS248xComponent::write_config_(DS248X_CONFIG_ACTIVE_PULLUP);
  }

  last_device_found_ = false;
  search_address_ = 0;
  search_last_discrepancy_ = 0;
}

bool DS248xComponent::set_channel_(uint8_t channel) {
  // For DS2482-100, channel switching is not needed
  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    this->channel_ = 0;
    return true;
  }

  std::array<uint8_t, 2> cmd;
  uint8_t data_byte;

  cmd[0] = DS2482_800_COMMAND_CHANNEL_SELECTION;
  cmd[1] = CHANNEL_CODE[channel];

  auto status = this->wait_while_busy_();
  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master busy before channel selection");
    return false;
  }

  auto err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write channel selection command: %d", err);
    return false;
  }

  delayMicroseconds(50);  // Allow channel switch to settle

  err = this->read(&data_byte, sizeof(data_byte));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read channel confirmation: %d", err);
    return false;
  }

  if (READ_CHANNEL_CODE[channel] == data_byte) {
    this->channel_ = channel;
    ESP_LOGV(TAG, "Successfully switched to channel %u", channel);
    return true;
  }

  ESP_LOGE(TAG, "Channel switch failed: expected 0x%02X, got 0x%02X", READ_CHANNEL_CODE[channel], data_byte);
  return false;
}

uint8_t DS248xComponent::get_channel_() { return (this->channel_); }

bool DS248xComponent::reset_devices_() {
  auto status = wait_while_busy_();
  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master never finished command");
    return false;
  }

  uint8_t cmd = DS248X_COMMAND_RESETWIRE;
  auto err = this->write(&cmd, sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Resetwire write failed %i", err);
    return false;
  }

  status = wait_while_busy_();

  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master never finished command");
    return false;
  }
  if (status & DS248X_STATUS_SD) {
    ESP_LOGE(TAG, "Bus is shorted");
    return false;
  }

  return true;
}

void DS248xComponent::write_command_(uint8_t command, uint8_t data) {
  auto status = wait_while_busy_();

  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master busy before write command 0x%02X", command);
    return;
  }

  std::array<uint8_t, 2> cmd;
  cmd[0] = command;
  cmd[1] = data;
  auto err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write command 0x%02X: %d", command, err);
    return;
  }
}

void DS248xComponent::select_(uint64_t address) {
  this->write_command_(DS248X_COMMAND_WRITEBYTE, WIRE_COMMAND_SELECT);

  for (int i = 0; i < 8; i++) {
    this->write_command_(DS248X_COMMAND_WRITEBYTE, (address >> (i * 8)) & 0xff);
  }
}

void DS248xComponent::write_to_wire_(uint8_t data) { this->write_command_(DS248X_COMMAND_WRITEBYTE, data); }

uint8_t DS248xComponent::read_from_wire_() {
  auto status = wait_while_busy_();

  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master busy before read from wire");
    return 0;
  }

  uint8_t command = DS248X_COMMAND_READBYTE;
  auto err = this->write(&command, sizeof(command));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write read command: %d", err);
    return 0;
  }

  status = wait_while_busy_();

  if (status & DS248X_STATUS_BUSY) {
    ESP_LOGE(TAG, "Master busy after read command");
    return 0;
  }

  std::array<uint8_t, 2> cmd;
  cmd[0] = DS248X_COMMAND_SETREADPTR;
  cmd[1] = DS248X_POINTER_DATA;
  err = this->write(cmd.data(), sizeof(cmd));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to set read pointer: %d", err);
    return 0;
  }

  uint8_t data_byte;
  err = this->read(&data_byte, sizeof(data_byte));
  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read data: %d", err);
    return 0;
  }

  return data_byte;
}

bool DS248xComponent::search_(uint64_t *address) {
  if (last_device_found_)
    return false;

  bool result = this->reset_devices_();
  if (!result) {
    this->status_set_warning();
    ESP_LOGE(TAG, "Reset failed");
    return false;
  }

  write_to_wire_(WIRE_COMMAND_SEARCH);

  uint8_t direction;
  uint8_t last_zero = 0;
  for (uint8_t i = 0; i < 64; i++) {
    uint64_t search_bit = 1LL << i;

    if (i < search_last_discrepancy_) {
      direction = (search_address_ & search_bit) != 0;
    } else {
      direction = i == search_last_discrepancy_;
    }

    write_command_(DS248X_COMMAND_TRIPLET, direction ? 0x80 : 0x00);

    uint8_t status = wait_while_busy_();
    ESP_LOGVV(TAG, "Search: i: %i dir: %i, status: %i bit: %llX", i, direction, status, search_bit);

    uint8_t id = status & DS248X_STATUS_SBR;
    uint8_t comp_id = status & DS248X_STATUS_TSB;
    direction = status & DS248X_STATUS_DIR;

    if (id && comp_id) {
      return false;
    } else if (!id && !comp_id && !direction) {
      last_zero = i;
    }

    if (direction) {
      search_address_ |= search_bit;
    } else {
      search_address_ &= ~search_bit;
    }
  }

  search_last_discrepancy_ = last_zero;

  if (!last_zero)
    last_device_found_ = true;

  *address = search_address_;

  return true;
}

bool DS248xComponent::verify_channel_(uint8_t expected_channel) {
  if (this->ds248x_type_ == DS248xType::DS2482_100) {
    return true;  // DS2482-100 only has one channel
  }

  uint8_t current_channel = this->get_channel_();
  if (current_channel != expected_channel) {
    ESP_LOGW(TAG, "Channel verification failed: expected %u, current %u", expected_channel, current_channel);
    return false;
  }
  return true;
}

void DS248xComponent::force_bus_recovery_() {
  ESP_LOGW(TAG, "Performing aggressive bus recovery");

  // Multiple reset cycles with delays
  for (int i = 0; i < 3; i++) {
    this->reset_hub_();
    delayMicroseconds(2000);

    // Try to verify basic communication
    uint8_t config = this->read_config_();
    if (config != 0) {
      ESP_LOGD(TAG, "Bus recovery cycle %d successful, config: 0x%02X", i + 1, config);
      break;
    }
    ESP_LOGW(TAG, "Bus recovery cycle %d failed", i + 1);
  }

  // Reset to known good configuration
  if (this->enable_active_pullup_) {
    this->write_config_(DS248X_CONFIG_ACTIVE_PULLUP);
  }

  delayMicroseconds(1000);
  ESP_LOGD(TAG, "Bus recovery completed");
}

void DS248xComponent::log_all_sensor_stats_() {
  ESP_LOGI(TAG, "=== DS248x Sensor Diagnostics ===");

  uint32_t total_sensors = this->sensors_.size();
  uint32_t total_reads = 0;
  uint32_t total_failures = 0;
  uint32_t total_crc_errors = 0;

  for (auto *sensor : this->sensors_) {
    sensor->log_sensor_stats();
    total_reads += sensor->get_total_reads();
    total_failures += sensor->get_failed_reads();
    total_crc_errors += sensor->get_checksum_errors();
  }

  float overall_success = total_reads > 0 ? ((float)(total_reads - total_failures) / total_reads) * 100.0 : 100.0;

  ESP_LOGI(TAG, "Overall: %u sensors, %u reads, %u failures, %u CRC errors, %.1f%% success",
           total_sensors, total_reads, total_failures, total_crc_errors, overall_success);

  if (overall_success < 90.0) {
    ESP_LOGW(TAG, "Poor overall reliability! Check power supply, I2C wiring, and 1-Wire connections");
  }

  ESP_LOGI(TAG, "=== End Diagnostics ===");
}

void DS248xComponent::setup_diagnostic_timer_() {
  // Log diagnostics every 5 minutes
  this->set_interval("diagnostics", 300000, [this]() {
    this->log_all_sensor_stats_();
  });
}

}  // namespace ds248x

}  // namespace esphome
