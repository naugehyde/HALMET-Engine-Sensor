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
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/curveinterpolator.h"

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


// Analog channel map
const int kFuelSenderChannel = 0;  // A1
const int kCoolantChannel    = 1;  // A2
const int kOilChannel        = 2;  // A3
const int kSupplyChannel     = 3;  // A4  (shared IGN-feed tap)
 

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
                    ->set_hostname("patricialynn-halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    // EDIT: Enable OTA updates with a password.
                    //->enable_ota("my_ota_password")
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

    /////////////////////////////////////////////////////////////////////
  // Ratiometric reader: Vs/Vcc. The 33.3/3.3 front end is identical on both
  // channels so it cancels; no voltage-divider scale needed. Reads both
  // channels back-to-back so numerator and denominator are contemporaneous.
  auto make_ratio_sensor = [ads1115](int sender_channel) {
    return new RepeatSensor<float>(500, [ads1115, sender_channel]() -> float {
      float v_sender =
          ads1115->computeVolts(ads1115->readADC_SingleEnded(sender_channel));
      float v_supply =
          ads1115->computeVolts(ads1115->readADC_SingleEnded(kSupplyChannel));
      if (v_supply <= 0.05f) return NAN;  // key off / no supply
      return v_sender / v_supply;
    });
  };
  // REGULATED-GAUGE FALLBACK: if the supply-sensitivity test showed Vs steady
  // while the battery moved, replace a channel's make_ratio_sensor(ch) with a
  // raw-volts reader and enter recorded Vs (not the ratio) in that curve:
  //   [ads1115](){ return 10.09f * ads1115->computeVolts(
  //                    ads1115->readADC_SingleEnded(ch)); }
 
  ///////////////////////////////////////////////////////////////////
  // FUEL  (A1)  ->  level 0..1  ->  volume
  auto fuel_ratio = make_ratio_sensor(kFuelSenderChannel);
 
  auto fuel_level = (new CurveInterpolator(nullptr, "/Tanks/Fuel/Level Curve"))
                        ->set_input_title("Sender Ratio (Vs/Vcc)")
                        ->set_output_title("Fuel Level (ratio)");
  ConfigItem(fuel_level)
      ->set_title("Fuel Tank Level Curve")
      ->set_description("Vs/Vcc ratio -> tank level, calibrated to the OEM gauge.")
      ->set_sort_order(3000);
  if (fuel_level->get_samples().empty()) {
    fuel_level->clear_samples();
    // CALIBRATE: Sample(Vs/Vcc , level). Replace levels with needle marks.
    /*
    12.51	2.87	1
    12.52	4.55	0.75
    12.51	5.63	5
    12.53	6.4	0.25
    12.51	7.5	0
    */
    fuel_level->add_sample(CurveInterpolator::Sample(7.5 / 12.51, 0.00));  // E
    fuel_level->add_sample(CurveInterpolator::Sample(6.4 / 12.51, 0.25));  // 1/4
    fuel_level->add_sample(CurveInterpolator::Sample(5.63 / 12.51, 0.50));  // 1/2
    fuel_level->add_sample(CurveInterpolator::Sample(4.55 / 12.51, 0.75));  // 3/4
    fuel_level->add_sample(CurveInterpolator::Sample(2.87 / 12.51, 1.00));  // F
  }
  fuel_ratio->connect_to(fuel_level);
 
  auto fuel_volume = new Linear(0.076f, 0.0f, "/Tanks/Fuel/Total Volume");
  ConfigItem(fuel_volume)
      ->set_title("Fuel Tank Total Volume")
      ->set_description("Tank capacity in m3 (0.076 = ~20 US gal)")
      ->set_sort_order(3002);
  fuel_level->connect_to(fuel_volume);
 
  fuel_ratio->connect_to(new SKOutputFloat(
      "tanks.fuel.main.senderRatio", "/Tanks/Fuel/Ratio SK Path",
      new SKMetadata("ratio", "Fuel sender Vs/Vcc")));
  fuel_level->connect_to(new SKOutputFloat(
      "tanks.fuel.main.currentLevel", "/Tanks/Fuel/Level SK Path",
      new SKMetadata("ratio", "Fuel tank level")));
  fuel_volume->connect_to(new SKOutputFloat(
      "tanks.fuel.main.currentVolume", "/Tanks/Fuel/Volume SK Path",
      new SKMetadata("m3", "Fuel tank volume")));
 
#ifdef ENABLE_NMEA2000_OUTPUT
  N2kFluidLevelSender* tank_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 120, nmea2000);
  ConfigItem(tank_sender)
      ->set_title("Fuel Tank NMEA 2000")
      ->set_sort_order(3005);
  fuel_level->connect_to(&(tank_sender->tank_level_));
#endif
 
  ///////////////////////////////////////////////////////////////////
  // COOLANT TEMPERATURE  (A2)  ->  Kelvin
  auto coolant_ratio = make_ratio_sensor(kCoolantChannel);
 
  auto coolant_temp_curve =
      (new CurveInterpolator(nullptr, "/Engine/Coolant Temperature/Curve"))
          ->set_input_title("Sender Ratio (Vs/Vcc)")
          ->set_output_title("Coolant Temperature (K)");
  ConfigItem(coolant_temp_curve)
      ->set_title("Coolant Temperature Curve")
      ->set_description("Vs/Vcc ratio -> coolant temp (K), calibrated to the OEM gauge.")
      ->set_sort_order(3025);
  if (coolant_temp_curve->get_samples().empty()) {
    coolant_temp_curve->clear_samples();
    // CALIBRATE: Sample(Vs/Vcc , Kelvin).  K = (F-32)*5/9 + 273.15
    // Fill ratios at the gauge's F marks; cluster near 160-200F.
    /*
    12.51	1.85	120
    12.51	2.4	    100
    12.51	3.24	80
    12.51	4.57	60
    12.51	5.85	40
    */

     coolant_temp_curve->add_sample(CurveInterpolator::Sample(5.85/12.51, 40+273.15)); 
     coolant_temp_curve->add_sample(CurveInterpolator::Sample(4.57/12.51, 60+273.15));
     coolant_temp_curve->add_sample(CurveInterpolator::Sample(3.24/12.51, 80+273.15));
     coolant_temp_curve->add_sample(CurveInterpolator::Sample(2.4/12.51,  100+273.15));     
     coolant_temp_curve->add_sample(CurveInterpolator::Sample(1.85/12.51, 120+273.15)); 

  }
  coolant_ratio->connect_to(coolant_temp_curve);
 
  coolant_ratio->connect_to(new SKOutputFloat(
      "propulsion.main.coolantTemperature.senderRatio",
      "/Engine/Coolant Temperature/Ratio SK Path",
      new SKMetadata("ratio", "Coolant sender Vs/Vcc")));
  coolant_temp_curve->connect_to(new SKOutputFloat(
      "propulsion.main.coolantTemperature",
      "/Engine/Coolant Temperature/SK Path",
      new SKMetadata("K", "Engine Coolant Temperature")));
 
  ///////////////////////////////////////////////////////////////////
  // OIL PRESSURE  (A3)  ->  Pascals
  auto oil_ratio = make_ratio_sensor(kOilChannel);
 
  auto oil_pressure_curve =
      (new CurveInterpolator(nullptr, "/Engine/Oil Pressure/Curve"))
          ->set_input_title("Sender Ratio (Vs/Vcc)")
          ->set_output_title("Oil Pressure (Pa)");
  ConfigItem(oil_pressure_curve)
      ->set_title("Oil Pressure Curve")
      ->set_description("Vs/Vcc ratio -> oil pressure (Pa), calibrated to the OEM gauge.")
      ->set_sort_order(3020);
  if (oil_pressure_curve->get_samples().empty()) {
    oil_pressure_curve->clear_samples();
    // CALIBRATE: Sample(Vs/Vcc , Pa).  1 psi = 6894.757 Pa
    /*12.51	2.64	500
12.51	4.45	375
12.51	5.5	250
12.51	6.2	125
12.51	7.5	0*/
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(7.5/12.51,  0.0f));          
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(6.2/12.51,  125.0f*1000.0f*0.145f));          
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(5.5/12.51,  250.0f*1000.0f*0.145f));          
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(4.45/12.51,  375.0f*1000.0f*0.145f));          
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(2.64/12.51,  500.0f*1000.0f*0.145f));          
  }
  oil_ratio->connect_to(oil_pressure_curve);
 
  oil_ratio->connect_to(new SKOutputFloat(
      "propulsion.main.oilPressure.senderRatio",
      "/Engine/Oil Pressure/Ratio SK Path",
      new SKMetadata("ratio", "Oil sender Vs/Vcc")));
  oil_pressure_curve->connect_to(new SKOutputFloat(
      "propulsion.main.oilPressure", "/Engine/Oil Pressure/SK Path",
      new SKMetadata("Pa", "Engine Oil Pressure")));
 
  
  // Helper to create a voltage reader for an ADS1115 channel and A-slot.
  auto make_voltage_input = [&](int channel) {
    const int a_slot = channel + 1;  // A1 = slot 1, A2 = slot 2, etc.
    const String config_path = String("/Voltage A") + String(a_slot);
    auto sensor = new ADS1115VoltageInput(ads1115, channel, config_path);

    ConfigItem(sensor)
        ->set_title(String("Analog Voltage A") + String(a_slot))
        ->set_description(String("Voltage level of analog input A") + String(a_slot))
        ->set_sort_order(3000 + a_slot);

    sensor->connect_to(new LambdaConsumer<float>([a_slot](float value) {
      debugD("Voltage A%d: %f", a_slot, value);
    }));

    if (sensor->reporting_enabled()) {
      const String sk_path = String("sensors.halmet.a") + String(a_slot) + ".voltage";
      sensor->connect_to(new SKOutputFloat(sk_path.c_str(), (String("Analog Voltage A") + String(a_slot)).c_str(),
                                           new SKMetadata("V", (String("Analog Voltage A") + String(a_slot)).c_str())));
    }

    return sensor;
  };

  // Read the voltage level of analog input A2
  auto a1_voltage = make_voltage_input(0);
  auto a2_voltage = make_voltage_input(1);
  auto a3_voltage = make_voltage_input(2);
  auto a4_voltage = make_voltage_input(3);


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
