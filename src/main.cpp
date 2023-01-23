// This is a version of a rudder angle sensor running on version1
// of SensESP to be able to run specifically on a ESP8266.
// The rudder sensor is a 5k pot connected with a belt to the
// rudder axle.

#include <Arduino.h>

#include "math.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensors/analog_input.h"
#include "signalk/signalk_output.h"
#include "transforms/lambda_transform.h"
#include "transforms/linear.h"
#include "wiring_helpers.h"

#define SERIAL_DEBUG_DISABLED

ReactESP app([]() {

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Create a new SensESPApp object. This is the direct constructor call, and
  // an equivalent alternative to using the SensESPAppBuilder class.

  // sensesp_app = new SensESPApp("rudder_angle_sensor", 
  //                              "PangoPi",
  //                              "mikeharris", 
  //                              "192.168.5.1", 3000);

  SensESPAppBuilder builder;

  // Set whatever options you want, then create the global SensESPApp() object with get_app():
  sensesp_app = builder.set_standard_sensors(NONE)
              ->set_hostname("rudderAngleSensor")
              ->set_sk_server("192.168.5.1", 3000)
              ->set_wifi("PangoPi", "mikeharris")
              // ->set_led_pin(13)
              // ->set_led_blinker(true, 1000, 2500, 4000)
              ->get_app();

  // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html
  const char* sk_path = "steering.rudderAngle";
  const char* analog_in_config_path = "/steering/rudderAngle";

#ifdef ESP8266
  uint8_t rudder_pin = A0;
#elif defined(ESP32)
  uint8_t rudder_pin = 32;
#endif

#ifdef ESP8266
  uint8_t temp_pin = D6;
#elif defined(ESP32)
  uint8_t temp_pin = 4;
#endif

  uint16_t rudder_read_delay = 500;
  
  float output_scale = 3.3;

  // Use AnalogInput as an example sensor. Connect it e.g. to a photoresistor
  // or a potentiometer with a voltage divider to get an illustrative test
  // input.

  auto* analog_input =
      new AnalogInput(rudder_pin, rudder_read_delay, analog_in_config_path, output_scale);

  // This is the transform function to transform the raw data to a rudder
  // angle value in radians.
  auto angle_function = [](float input, 
                           float offset, 
                           float prt_angle,
                           float prt_angle_value,
                           float stb_angle,
                           float stb_angle_value) -> float {
    // Note the map function is integer based. Alternative is:
    // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    float angle = (input - prt_angle_value) * (stb_angle - prt_angle) / (stb_angle_value - prt_angle_value) + prt_angle;
    angle += offset;
    //float angle = map(input, prt_angle_value, stb_angle_value, prt_angle, stb_angle) + offset;
    return angle * 0.0174533; // convert to rad
  };

  // This is an array of parameter information, providing the keys and
  // descriptions required to display user-friendly values in the configuration
  // interface.

  const ParamInfo* angle_lambda_param_data = new ParamInfo[5]{
      {"offset", "Offset"}, 
      {"prt_angle", "Port Angle"}, 
      {"prt_angle_value", "Port value"}, 
      {"stb_angle", "Starboard Angle"}, 
      {"stb_angle_value", "Starboard value"}};

  // Here we create a new LambdaTransform objects. The template parameters
  // (five floats in this example) correspond to the following types:
  //
  // 1. Input type of the transform function
  // 2. Output type of the transform function
  // 3. Type of parameter 1
  // 4. Type of parameter 2
  // 5. Type of parameter 3
  //
  // The function arguments are:
  // 1. The tranform function
  // 2-4. Default values for parameters
  // 5. Parameter data for the web config UI
  // 6. Configuration path for the transform

  auto angle_transform = new LambdaTransform<float, float, float, float, float, float, float>(
      angle_function, 0, -35, 0, 35, 3.3, angle_lambda_param_data,
      "/Transforms/Angle Transform");

  // Finally, connect the analog input via the freshly-generated log_transform
  // to an SKOutputNumber object

  analog_input->connect_to(angle_transform)
      ->connect_to(new SKOutputNumber(
        sk_path, analog_in_config_path,
        new SKMetadata("rad", "Rudder Angle")));


  DallasTemperatureSensors* dts = new DallasTemperatureSensors(temp_pin);

  // Define how often SensESP should read the sensor(s) in milliseconds
  uint16_t temp_read_delay = 10e3;

  // Below are temperatures sampled and sent to Signal K server
  // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html

  // Measure temperature
  auto* bilge_temp =
      new OneWireTemperature(dts, temp_read_delay, "/aftBilgeTemperature/oneWire");

  bilge_temp->connect_to(new Linear(1.0, 0.0, "/aftBilgeTemperature/linear"))
      ->connect_to(
          new SKOutputNumber("environment.inside.aftBilge.temperature",
                             "/aftBilgeTemperature/skPath"));

  sensesp_app->enable();
});