#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <WiFi.h>

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

// WiFi credentials
const char* ssid = "Google";
const char* password = "123ggg123";

// NTP
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

// Motor pin assignments
const int left_front = 16;
const int left_back  = 17;
const int right_front = 18;
const int right_back  = 19;

int count=0;

// Function declarations
void avrc_metadata_callback(uint8_t id, const uint8_t *text);
bool parse_text(const uint8_t *text, int vals[4]);
void setMotors(int vals[4]);

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  // Serial.println("we are receiving metadata!");
  Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  int vals[4] = {0, 0, 0, 0};
  if (parse_text(text, vals)) {
    setMotors(vals);
    Serial.print("metadata parsed!");
    Serial.println(count);
    count++;
  } else {
    // Serial.println("Failed to parse metadata.");
  }
}
void calc_latency(const uint8_t *text){
  const char* timeStr = (const char*) text;
  uint64_t phoneTime = strtoull(timeStr, NULL, 10);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t esp32Time = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;

  long latency = esp32Time - phoneTime;

  Serial.print("Latency: ");
  Serial.println(latency);
}
bool parse_text(const uint8_t *text, int vals[4]) {
  // Convert the incoming data to a string pointer.
  const char *str = reinterpret_cast<const char *>(text);
  // Expect four integer values. The format specifier for int is "%d".
  int count = sscanf(str, "%d %d %d %d", &vals[0], &vals[1], &vals[2], &vals[3]);
  return (count == 4);
}

void setMotors(int vals[4]) {
  // Convert the parsed double values to integers within a PWM range of 0-255.
  int lf = constrain(vals[0], 0, 255);
  int lb = constrain(vals[1], 0, 255);
  int rf = constrain(vals[2], 0, 255);
  int rb = constrain(vals[3], 0, 255);
  
  analogWrite(left_front, lf);
  analogWrite(left_back,  lb);
  analogWrite(right_front, rf);
  analogWrite(right_back,  rb);
  
  Serial.print("Left Front: ");
  Serial.println(lf);
  Serial.print("Left Back: ");
  Serial.println(lb);
  Serial.print("Right Front: ");
  Serial.println(rf);
  Serial.print("Right Back: ");
  Serial.println(rb);
}

void setup() {
  Serial.begin(115200);
  // connect_wifi();

  // Initialize motor pins as outputs.
  pinMode(left_front, OUTPUT);
  pinMode(left_back,  OUTPUT);
  pinMode(right_front, OUTPUT);
  pinMode(right_back,  OUTPUT);

    auto cfg = i2s.defaultConfig(TX_MODE);
    cfg.i2s_format = I2S_STD_FORMAT;

    cfg.pin_bck = 14;
    cfg.pin_ws = 15;
    cfg.pin_data = 22;
    i2s.begin(cfg);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.start("AVA-GEM");
}

void connect_wifi(){
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void loop() {
  // No recurring operations needed.
  // Serial.println("1 sec passed");
  // delay(1000);
}