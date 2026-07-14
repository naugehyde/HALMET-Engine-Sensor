#ifndef HALMET_ANALOG_H_
#define HALMET_ANALOG_H_

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp_base_app.h"

namespace halmet {

// Default HALMET voltage divider scale factor. This is the ratio of the
// hardware voltage divider that precedes each ADS1115 analog input.
const float kDefaultVoltageDividerScale = 33.3 / 3.3;

/**
 * @brief User-configurable HALMET analog input voltage divider scale
 * factor.
 *
 * All of HALMET's analog inputs share the same fixed hardware voltage
 * divider (nominally 33.3:3.3). Boards with a modified divider, or users who
 * need to compensate for component tolerances, can adjust the ratio from the
 * web UI instead of editing source code.
 */
class VoltageDividerScale : public sensesp::FileSystemSaveable {
 public:
  VoltageDividerScale(const String& config_path)
      : sensesp::FileSystemSaveable(config_path) {
    load();
  }

  float scale = kDefaultVoltageDividerScale;

  virtual bool to_json(JsonObject& root) override {
    root["scale"] = scale;
    return true;
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["scale"].is<float>()) {
      return false;
    }
    scale = config["scale"];
    return true;
  }
};

inline const String ConfigSchema(const VoltageDividerScale& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "scale": { "title": "Voltage divider scale", "type": "number", "description": "HALMET analog input hardware voltage divider ratio (default 33.3/3.3)" }
      }
    })###";

  return SCHEMA;
}

inline const bool ConfigRequiresRestart(const VoltageDividerScale& obj) {
  return true;
}

sensesp::FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          VoltageDividerScale* voltage_divider_scale,
                                          bool enable_signalk_output = true);

class ADS1115VoltageInput : public sensesp::FloatSensor {
 public:
  ADS1115VoltageInput(Adafruit_ADS1115* ads1115, int channel,
                      VoltageDividerScale* voltage_divider_scale,
                      const String& config_path,
                      unsigned int read_interval = 500,
                      float calibration_factor = 1.0)
      : sensesp::FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel},
        voltage_divider_scale_{voltage_divider_scale},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor} {
    load();

    repeat_event_ = set_repeat_event(read_interval_);
  }

  void update() {
    int16_t adc_output = ads1115_->readADC_SingleEnded(channel_);
    float adc_output_volts = ads1115_->computeVolts(adc_output);
    this->emit(calibration_factor_ * voltage_divider_scale_->scale *
               adc_output_volts);
  }

  virtual bool to_json(JsonObject& root) override {
    root["calibration_factor"] = calibration_factor_;
    return true;
  };

  virtual bool from_json(const JsonObject& config) override {
    if (config["calibration_factor"].is<float>()) {
      calibration_factor_ = config["calibration_factor"];
      return true;
    }
    return false;
  }

 protected:
  reactesp::RepeatEvent* repeat_event_ = nullptr;

  reactesp::RepeatEvent* set_repeat_event(unsigned int read_interval) {
    if (repeat_event_ != nullptr) {
      repeat_event_->remove(sensesp::event_loop());
    }

    repeat_event_ = sensesp::event_loop()->onRepeat(
        read_interval, [this]() { this->update(); });
    return repeat_event_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  VoltageDividerScale* voltage_divider_scale_;
  unsigned int read_interval_;
  float calibration_factor_;
};

inline const String ConfigSchema(const ADS1115VoltageInput& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "calibration_factor": { "title": "Calibration factor", "type": "number", "description": "Multiplier to apply to the raw input value" }
      }
    })###";

  return SCHEMA;
}

inline const bool ConfigRequiresRestart(const ADS1115VoltageInput& obj) {
  return true;
}

}  // namespace halmet

#endif
