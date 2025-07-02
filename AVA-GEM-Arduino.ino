#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <array>

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

// Motor pin assignments
const int motor_pins[4] = {16, 19, 17, 18}; // left_front, left_back, right_front, right_back
int motor_status[4] = {0, 0, 0, 0};
int motor_strength = 0;
int frequency = 0;       // in Hz
int duty_cycle = 0;      // percentage [0-100]
int t1 = 0;              // off duration (ms)
int t2 = 0;              // on duration (ms)
int metadata_count = 0;

// Non-blocking timing
unsigned long previousMillis = 0;
bool motorsOn = false;

// Forward declarations
void avrc_metadata_callback(uint8_t id, const uint8_t *text);
bool parse_text(const uint8_t *text);
void calc_timing();
std::array<int, 4> calc_motor_vals();
void set_motors(const std::array<int, 4> &vals);

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  if (parse_text(text)) {
    calc_timing();
    Serial.print("metadata parsed! count=");
    Serial.println(metadata_count);
    metadata_count++;
  }
}

bool parse_text(const uint8_t *text) {
  const char *str = reinterpret_cast<const char *>(text);
  int cnt = sscanf(str,
                   "%d %d %d %d %d %d %d",
                   &motor_status[0], &motor_status[1],
                   &motor_status[2], &motor_status[3],
                   &motor_strength, &frequency, &duty_cycle);

  Serial.print("motor_strength: ");
  Serial.println(motor_strength);
    Serial.print("frequency: ");
  Serial.println(frequency);
    Serial.print("duty_cycle: ");
  Serial.println(duty_cycle);
  return (cnt == 7);
}

void calc_timing() {
  if (frequency <= 0 || duty_cycle <= 0) {
    t1 = 1;
    t2 = 0;
    Serial.print("t1: ");
    Serial.println(t1);
    Serial.print("t2: ");
    Serial.println(t2);
    return;
  }
  float period_ms = 1000.0f / frequency;
  t2 = period_ms * (duty_cycle / 100.0f);
  t1 = period_ms - t2;
  Serial.print("t1: ");
  Serial.println(t1);
  Serial.print("t2: ");
  Serial.println(t2);
}

std::array<int, 4> calc_motor_vals() {
  std::array<int, 4> vals;
  for (int i = 0; i < 4; i++) {
    vals[i] = constrain(motor_status[i] * motor_strength, 0, 255);
  }
  return vals;
}

void set_motors(const std::array<int, 4> &vals) {
  static const char *motor_names[4] = {"left front", "left back", "right front", "right back"};
  for (int i = 0; i < 4; i++) {
    analogWrite(motor_pins[i], vals[i]);
    // Serial.print(motor_names[i]);
    // Serial.print(": ");
    // Serial.println(vals[i]);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  // I2S configuration
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.pin_bck = 14;
  cfg.pin_ws  = 15;
  cfg.pin_data = 22;
  i2s.begin(cfg);

  // Bluetooth A2DP setup
  a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.start("AVA-GEM");

  // Initialize timing reference
  previousMillis = millis();
  motorsOn = false;
  set_motors({0, 0, 0, 0});
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long interval = motorsOn ? t2 : t1;

  // Check if it's time to toggle motor state
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    motorsOn = !motorsOn;
  }
  if (motorsOn) {
      set_motors(calc_motor_vals());
      Serial.println("motors_on");
    } else {
      set_motors({0, 0, 0, 0});
      Serial.println("motors_off");
    }
}