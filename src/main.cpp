// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halriot")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    // EDIT: Enable OTA updates with a password.
                    //->enable_ota("my_ota_  word")
                    ->get_app();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app->get(), &display, i2c);

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  bool enable_signalk_output = true;

  // HALMET's analog inputs share a fixed hardware voltage divider (default
  // 33.3/3.3). Expose it as a user-configurable value in the web UI in case
  // a board revision or component tolerances require adjustment.
  auto voltage_divider_scale =
      new VoltageDividerScale("/Analog Inputs/Voltage Divider Scale");

  ConfigItem(voltage_divider_scale)
      ->set_title("Analog Input Voltage Divider Scale")
      ->set_description(
          "HALMET analog input hardware voltage divider ratio (default "
          "33.3/3.3)")
      ->set_sort_order(2990);

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume =
      ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
                        voltage_divider_scale, enable_signalk_output);
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "A2", "tanks.a2", 3010, voltage_divider_scale);
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "A3", "tanks.a3", 3020, voltage_divider_scale);
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4", "tanks.a4", 3030, voltage_divider_scale);

#ifdef ENABLE_NMEA2000_OUTPUT
  // Tank 1, instance 0. Capacity 200 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 200, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A1")
      ->set_sort_order(3005);

  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_));
#endif  // ENABLE_NMEA2000_OUTPUT

  if (display_present) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
  }

  ///////////////////////////////////////////////////////////////////
  // Coolant temperature sender on analog input A2 (ADS1115 channel 1).
  //
  // This expects a US-standard resistive temperature sender (Faria /
  // Stewart-Warner style) with a nominal range of ~450 ohms at 100 F
  // down to ~29 ohms at 250 F. The HALMET analog front-end drives the
  // sender with a constant ~10 mA current, so the sender resistance is
  // computed from the measured voltage.
  constexpr float kCoolantTempMeasurementCurrent = 0.01f;  // A

  auto coolant_temp_resistance = new RepeatSensor<float>(
      500, [ads1115, voltage_divider_scale]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(1);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return voltage_divider_scale->scale * adc_output_volts /
               kCoolantTempMeasurementCurrent;
      });

  // Piecewise linear curve mapping sender resistance (ohms) to coolant
  // temperature (Kelvin). Defaults match a US-standard 450-29 ohm sender
  // with a 100-250 F range. The curve is editable from the web UI.
  auto coolant_temp_curve =
      (new CurveInterpolator(nullptr, "/Engine/Coolant Temperature/Curve"))
          ->set_input_title("Sender Resistance (ohms)")
          ->set_output_title("Coolant Temperature (K)");

  ConfigItem(coolant_temp_curve)
      ->set_title("Coolant Temperature Curve")
      ->set_description(
          "Piecewise linear curve mapping sender resistance (ohms) to "
          "coolant temperature (K). Default: US-standard 450-29 ohm, "
          "100-250 F sender.")
      ->set_sort_order(3025);

  if (coolant_temp_curve->get_samples().empty()) {
    // F to K: K = (F - 32) * 5/9 + 273.15
    coolant_temp_curve->clear_samples();
    coolant_temp_curve->add_sample(
        CurveInterpolator::Sample(450.0f, 310.93f));  // 100 F
    coolant_temp_curve->add_sample(
        CurveInterpolator::Sample(200.0f, 338.71f));  // 150 F
    coolant_temp_curve->add_sample(
        CurveInterpolator::Sample(78.0f, 366.48f));   // 200 F
    coolant_temp_curve->add_sample(
        CurveInterpolator::Sample(29.0f, 394.26f));   // 250 F
  }

  coolant_temp_resistance->connect_to(coolant_temp_curve);

  coolant_temp_resistance->connect_to(new SKOutputFloat(
      "propulsion.main.coolantTemperature.senderResistance",
      "/Engine/Coolant Temperature/Sender Resistance SK Path",
      new SKMetadata("ohm", "Coolant Temperature Sender Resistance")));

  coolant_temp_curve->connect_to(new SKOutputFloat(
      "propulsion.main.coolantTemperature",
      "/Engine/Coolant Temperature/SK Path",
      new SKMetadata("K", "Engine Coolant Temperature")));

  ///////////////////////////////////////////////////////////////////
  // Oil pressure sender on analog input A3 (ADS1115 channel 2).
  //
  // This expects a US-standard resistive oil pressure sender (VDO /
  // Stewart-Warner style) with a nominal range of 240 ohms at 0 psi
  // down to ~33 ohms at 80 psi. The HALMET analog front-end drives the
  // sender with a constant ~10 mA current, so the sender resistance is
  // computed from the measured voltage.
  constexpr float kOilPressureMeasurementCurrent = 0.01f;  // A

  auto oil_pressure_resistance =
      new RepeatSensor<float>(500, [ads1115, voltage_divider_scale]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(2);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return voltage_divider_scale->scale * adc_output_volts /
               kOilPressureMeasurementCurrent;
      });

  // Piecewise linear curve mapping sender resistance (ohms) to oil
  // pressure (Pa). Defaults match a US-standard 240-33 ohm sender with
  // a linear 0-80 psi range. The curve is editable from the web UI.
  auto oil_pressure_curve = (new CurveInterpolator(
                                 nullptr, "/Engine/Oil Pressure/Curve"))
                                ->set_input_title("Sender Resistance (ohms)")
                                ->set_output_title("Oil Pressure (Pa)");

  ConfigItem(oil_pressure_curve)
      ->set_title("Oil Pressure Curve")
      ->set_description(
          "Piecewise linear curve mapping sender resistance (ohms) to oil "
          "pressure (Pa). Default: US-standard 240-33 ohm, 0-80 psi sender.")
      ->set_sort_order(3020);

  if (oil_pressure_curve->get_samples().empty()) {
    // psi to Pa: 1 psi = 6894.757 Pa
    oil_pressure_curve->clear_samples();
    oil_pressure_curve->add_sample(
        CurveInterpolator::Sample(240.0f, 0.0f));
    oil_pressure_curve->add_sample(
        CurveInterpolator::Sample(103.0f, 40.0f * 6894.757f));
    oil_pressure_curve->add_sample(
        CurveInterpolator::Sample(33.0f, 80.0f * 6894.757f));
    oil_pressure_curve->add_sample(
        CurveInterpolator::Sample(0.0f, 100.0f * 6894.757f));
  }

  oil_pressure_resistance->connect_to(oil_pressure_curve);

  // Publish raw sender resistance and converted pressure to Signal K.
  oil_pressure_resistance->connect_to(new SKOutputFloat(
      "propulsion.main.oilPressure.senderResistance",
      "/Engine/Oil Pressure/Sender Resistance SK Path",
      new SKMetadata("ohm", "Oil Pressure Sender Resistance")));

  oil_pressure_curve->connect_to(new SKOutputFloat(
      "propulsion.main.oilPressure", "/Engine/Oil Pressure/SK Path",
      new SKMetadata("Pa", "Engine Oil Pressure")));

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  alarm_d2_input->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d3_inverted->connect_to(engine_dynamic_sender->over_temperature_);

  // Feed the A3 oil pressure measurement (Pa) into the NMEA 2000
  // dynamic engine parameter sender (PGN 127489) for engine instance 0.
  oil_pressure_curve
      ->connect_to(new LambdaTransform<float, double>(
          [](float value) { return static_cast<double>(value); }))
      ->connect_to(engine_dynamic_sender->oil_pressure_);

  // Feed the A2 coolant temperature measurement (K) into the NMEA 2000
  // dynamic engine parameter sender (PGN 127489) for engine instance 0.
  coolant_temp_curve
      ->connect_to(new LambdaTransform<float, double>(
          [](float value) { return static_cast<double>(value); }))
      ->connect_to(engine_dynamic_sender->temperature_);

  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");

  // Connect outputs to the N2k senders.
  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));

  if (display_present) {
    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
  }

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    event_loop()->onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    event_loop()->onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
