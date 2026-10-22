#include <Wire.h>
#include <math.h>
#include <driver/i2s.h>
#include "HX711.h"
#include <WiFi.h>
#include <HTTPClient.h>

// =====================================================
// WIFI
// =====================================================

const char* WIFI_SSID = "SAMEEKSHA 5811";
const char* WIFI_PASSWORD = "0282R(y3";

// Example:
// http://192.168.x.x:5000/api/esp32
const char* SERVER_URL = "http://192.168.137.1:5000/api/esp32";

// =====================================================
// PIN DEFINITIONS
// =====================================================

// HX711
#define HX711_DT   33
#define HX711_SCK  19

// INMP441
#define I2S_SCK    14
#define I2S_WS     15
#define I2S_SD     32

// Vibration motor
#define MOTOR_PIN  4

// MPU6500
#define MPU_ADDR   0x68
#define SDA_PIN    21
#define SCL_PIN    22

#define PWR_MGMT_1    0x6B
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define ACCEL_XOUT_H  0x3B

// =====================================================
// HX711
// =====================================================

HX711 scale;

float calibrationFactor = 20.8;

float weight_g = 0;

const float WEIGHT_THRESHOLD = 0.0;

unsigned long lastWeightRead = 0;
const unsigned long WEIGHT_INTERVAL = 1000;

// =====================================================
// MPU6500
// =====================================================

float ax = 0;
float ay = 0;
float az = 0;

float gx = 0;
float gy = 0;
float gz = 0;

float accelerationMagnitude = 0;

float roll = 0;
float pitch = 0;

String currentPosition = "UNKNOWN";
String lastPosition = "";

int postureChangeCount = 0;

unsigned long lastPosturePrint = 0;
unsigned long postureWindowStart = 0;

const unsigned long POSTURE_WINDOW = 60000;

const int RESTLESS_THRESHOLD = 10;

const float TILT_THRESHOLD = 10.0;

// =====================================================
// MICROPHONE
// =====================================================

#define MIC_RATE 16000
#define MIC_N    512

int32_t micRaw[MIC_N];

float micThreshold = 0;
float micNoiseFloor = 0;

const float MIC_MULT = 2.0;

float soundLevel = 0;

// =====================================================
// SOUND WINDOW
// =====================================================

// IMPORTANT:
// Sound is collected for 60 seconds before classification.

const unsigned long WINDOW_DURATION = 60000;

unsigned long windowStart = 0;

const int MAX_EVENTS = 50;

unsigned long pulseDurations[MAX_EVENTS];
unsigned long gapDurations[MAX_EVENTS];

int eventCount = 0;

unsigned long pulseStart = 0;
unsigned long lastPulseEnd = 0;

bool inPulse = false;

// =====================================================
// LIVE SOUND STATUS
// =====================================================

// This is shown immediately on the website.

String liveSoundEvent = "QUIET";

// This is ONLY changed after 60-second classification.

String soundEvent = "NO SIGNIFICANT SOUND";

bool snoringDetected = false;
bool sleepTalkingDetected = false;

// Increases by one every time classifyPattern() completes a
// 60-second window. Sent to the website so the server knows
// exactly when a new classification happened.

unsigned long classificationId = 0;

unsigned long lastSoundDetectedAt = 0;

// Keep "SOUND DETECTED" on dashboard for 2 seconds
// after the sound disappears.

const unsigned long LIVE_SOUND_HOLD_MS = 2000;

// =====================================================
// VIBRATION
// =====================================================

bool vibrationActive = false;

unsigned long vibrationStart = 0;
unsigned long vibrationDuration = 0;

// =====================================================
// I2S HIGH PASS FILTER
// =====================================================

float hpX = 0;
float hpY = 0;

// =====================================================
// MPU WRITE
// =====================================================

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// =====================================================
// MPU READ
// =====================================================

bool readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  int received = Wire.requestFrom(MPU_ADDR, 14);

  if (received < 14)
  {
    return false;
  }

  byte data[14];

  for (int i = 0; i < 14; i++)
  {
    data[i] = Wire.read();
  }

  int16_t rawAx =
      ((int16_t)data[0] << 8) | data[1];

  int16_t rawAy =
      ((int16_t)data[2] << 8) | data[3];

  int16_t rawAz =
      ((int16_t)data[4] << 8) | data[5];

  int16_t rawGx =
      ((int16_t)data[8] << 8) | data[9];

  int16_t rawGy =
      ((int16_t)data[10] << 8) | data[11];

  int16_t rawGz =
      ((int16_t)data[12] << 8) | data[13];

  // ±2g
  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;

  // ±250 dps
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;

  accelerationMagnitude =
      sqrt(
        ax * ax +
        ay * ay +
        az * az
      );

  return true;
}

// =====================================================
// POSTURE
// =====================================================

void handlePosture(unsigned long now)
{
  if (!readMPU())
  {
    currentPosition = "MPU ERROR";
    return;
  }

  roll =
      atan2(ay, az) * 180.0 / PI;

  pitch =
      atan2(
        -ax,
        sqrt(
          ay * ay +
          az * az
        )
      ) * 180.0 / PI;

  // Slight pillow tilt is allowed
  const float TILT_THRESHOLD = 10.0;

  if (az < -0.5)
  {
    currentPosition = "UPSIDE DOWN";
  }
  else if (roll > TILT_THRESHOLD)
  {
    currentPosition = "TURNED RIGHT";
  }
  else if (roll < -TILT_THRESHOLD)
  {
    currentPosition = "TURNED LEFT";
  }
  else
  {
    currentPosition = "NORMAL";
  }

  if (
      lastPosition != "" &&
      currentPosition != lastPosition
     )
  {
    postureChangeCount++;

    Serial.print("POSITION CHANGED -> ");
    Serial.println(currentPosition);
  }

  lastPosition = currentPosition;

  if (now - lastPosturePrint >= 1000)
  {
    lastPosturePrint = now;

    Serial.print("POSITION: ");
    Serial.print(currentPosition);

    Serial.print(" | Roll: ");
    Serial.print(roll, 1);

    Serial.print(" | Pitch: ");
    Serial.print(pitch, 1);

    Serial.print(" | Accel: ");
    Serial.println(accelerationMagnitude, 2);
  }
}

// =====================================================
// I2S INITIALIZATION
// =====================================================

void i2sInit()
{
  Serial.println();
  Serial.println("Initializing INMP441...");

  i2s_config_t cfg =
  {
    .mode =
      (i2s_mode_t)(
        I2S_MODE_MASTER |
        I2S_MODE_RX
      ),

    .sample_rate = MIC_RATE,

    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_32BIT,

    .channel_format =
      I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format =
      I2S_COMM_FORMAT_STAND_I2S,

    .intr_alloc_flags =
      ESP_INTR_FLAG_LEVEL1,

    .dma_buf_count = 4,

    .dma_buf_len = MIC_N,

    .use_apll = false,

    .tx_desc_auto_clear = false,

    .fixed_mclk = 0
  };

  i2s_pin_config_t pins =
  {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t result;

  result =
      i2s_driver_install(
        I2S_NUM_0,
        &cfg,
        0,
        NULL
      );

  Serial.print("I2S driver install: ");
  Serial.println(esp_err_to_name(result));

  result =
      i2s_set_pin(
        I2S_NUM_0,
        &pins
      );

  Serial.print("I2S pin setup: ");
  Serial.println(esp_err_to_name(result));

  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("INMP441 initialization complete.");
}

// =====================================================
// READ ONE MIC WINDOW
// =====================================================

bool readMicWindow(float &rms)
{
  size_t bytes = 0;

  esp_err_t result =
      i2s_read(
        I2S_NUM_0,
        micRaw,
        sizeof(micRaw),
        &bytes,
        portMAX_DELAY
      );

  if (result != ESP_OK)
  {
    Serial.print("I2S READ ERROR: ");
    Serial.println(esp_err_to_name(result));

    return false;
  }

  int n =
      bytes / sizeof(int32_t);

  if (n <= 0)
  {
    return false;
  }

  // 24-bit microphone data
  // stored in 32-bit words

  long long sum = 0;

  for (int i = 0; i < n; i++)
  {
    micRaw[i] >>= 8;
    sum += micRaw[i];
  }

  int32_t mean =
      sum / n;

  double sq = 0;

  for (int i = 0; i < n; i++)
  {
    float x =
        micRaw[i] - mean;

    float y =
        x -
        hpX +
        0.995f * hpY;

    hpX = x;
    hpY = y;

    sq +=
        (double)y * y;
  }

  rms =
      sqrt(sq / n);

  return true;
}

// =====================================================
// MIC CALIBRATION
// =====================================================

void calibrateMic()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("       INMP441 CALIBRATION");
  Serial.println("Keep the room SILENT");
  Serial.println("for 6 seconds...");
  Serial.println("================================");

  float rms;

  float total = 0;
  int count = 0;

  unsigned long start =
      millis();

  while (
      millis() - start < 6000
  )
  {
    if (!readMicWindow(rms))
    {
      continue;
    }

    // Ignore first 3 seconds
    if (millis() - start > 3000)
    {
      total += rms;
      count++;
    }
  }

  if (count > 0)
  {
    micNoiseFloor =
        total / count;

    micThreshold =
        micNoiseFloor * MIC_MULT;

    // Prevent threshold from being zero
    if (micThreshold < 1)
    {
      micThreshold = 1;
    }

    Serial.print("Noise floor: ");
    Serial.println(micNoiseFloor);

    Serial.print("Mic threshold: ");
    Serial.println(micThreshold);
  }
  else
  {
    Serial.println("MIC CALIBRATION FAILED");

    micNoiseFloor = 0;

    // Fallback
    micThreshold = 1;
  }

  Serial.println("================================");
}

// =====================================================
// HANDLE MICROPHONE
//
// IMPORTANT:
// NO CLASSIFICATION HERE
// NO VIBRATION HERE
//
// Only:
// 1. Detect sound immediately
// 2. Show live sound
// 3. Collect sound pattern
// =====================================================

void handleMic(unsigned long now)
{
  float rms;

  if (!readMicWindow(rms))
  {
    return;
  }

  now = millis();

  // =================================================
  // SOUND LEVEL FOR WEBSITE
  // =================================================

  if (micThreshold > 0)
  {
    soundLevel =
        30.0 +
        20.0 *
        log10(
          max(
            rms / micThreshold,
            0.01f
          )
        );
  }
  else
  {
    soundLevel = 20;
  }

  soundLevel =
      constrain(
        soundLevel,
        20.0,
        100.0
      );

  // =================================================
  // SOUND DETECTION
  // =================================================

  bool soundDetected =
      (rms > micThreshold);

  // =================================================
  // IMMEDIATE WEBSITE UPDATE
  // =================================================

  if (soundDetected)
  {
    liveSoundEvent =
        "SOUND DETECTED";

    lastSoundDetectedAt =
        now;
  }
  else
  {
    if (
        now - lastSoundDetectedAt >
        LIVE_SOUND_HOLD_MS
       )
    {
      liveSoundEvent =
          "QUIET";
    }
  }

  // =================================================
  // SOUND START
  // =================================================

  if (
      soundDetected &&
      !inPulse
     )
  {
    inPulse = true;

    pulseStart = now;

    // Calculate gap from previous sound
    if (
        lastPulseEnd > 0 &&
        eventCount < MAX_EVENTS
       )
    {
      gapDurations[eventCount] =
          now - lastPulseEnd;
    }

    Serial.println(
      "LIVE SOUND DETECTED"
    );
  }

  // =================================================
  // SOUND END
  // =================================================

  if (
      !soundDetected &&
      inPulse
     )
  {
    inPulse = false;

    unsigned long duration =
        now - pulseStart;

    lastPulseEnd =
        now;

    if (
        eventCount < MAX_EVENTS
       )
    {
      pulseDurations[eventCount] =
          duration;

      eventCount++;
    }

    Serial.print("Sound pulse: ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}

// =====================================================
// CLASSIFY THE LAST 60 SECOND WINDOW
//
// THIS FUNCTION IS THE ONLY PLACE WHERE
// SNORING / SLEEP TALKING CLASSIFICATION OCCURS.
//
// VIBRATION ALSO STARTS ONLY HERE.
// =====================================================

void classifyPattern()
{
  // Every call completes one 60-second window,
  // whatever the result is.
  classificationId++;

  Serial.println();
  Serial.println("================================");
  Serial.println("60 SECOND SOUND CLASSIFICATION");
  Serial.println("================================");

  Serial.print("Sound events: ");
  Serial.println(eventCount);

  // Reset previous classification flags

  snoringDetected = false;
  sleepTalkingDetected = false;

  // -----------------------------------------------
  // Not enough data
  // -----------------------------------------------

  if (eventCount < 2)
  {
    soundEvent =
        "NO SIGNIFICANT SOUND";

    Serial.println(
      "Not enough sound events."
    );

    Serial.println(
      "Classification: NO SIGNIFICANT SOUND"
    );

    Serial.println("No vibration.");

    return;
  }

  // -----------------------------------------------
  // Average pulse duration
  // -----------------------------------------------

  unsigned long totalDuration = 0;

  for (
      int i = 0;
      i < eventCount;
      i++
     )
  {
    totalDuration +=
        pulseDurations[i];
  }

  unsigned long avgDuration =
      totalDuration /
      eventCount;

  // -----------------------------------------------
  // Average gap
  // -----------------------------------------------

  unsigned long totalGap = 0;

  int gapCount = 0;

  for (
      int i = 1;
      i < eventCount;
      i++
     )
  {
    totalGap +=
        gapDurations[i];

    gapCount++;
  }

  if (gapCount == 0)
  {
    soundEvent =
        "NOISE";

    return;
  }

  unsigned long avgGap =
      totalGap /
      gapCount;

  // -----------------------------------------------
  // GAP VARIANCE
  // -----------------------------------------------

  double variance = 0;

  for (
      int i = 1;
      i < eventCount;
      i++
     )
  {
    double diff =
        (double)gapDurations[i] -
        (double)avgGap;

    variance +=
        diff * diff;
  }

  variance /=
      gapCount;

  // -----------------------------------------------
  // RHYTHMICITY
  // -----------------------------------------------

  bool isRhythmic =
      variance <
      (
        (double)avgGap *
        (double)avgGap *
        0.30
      );

  // -----------------------------------------------
  // PRINT STATISTICS
  // -----------------------------------------------

  Serial.print("Average pulse: ");
  Serial.print(avgDuration);
  Serial.println(" ms");

  Serial.print("Average gap: ");
  Serial.print(avgGap);
  Serial.println(" ms");

  Serial.print("Rhythmic: ");

  if (isRhythmic)
  {
    Serial.println("YES");
  }
  else
  {
    Serial.println("NO");
  }

  // =================================================
  // SNORING
  // =================================================

  // Rhythmic patterns are prioritized as SNORING regardless of duration.
  // Non-rhythmic patterns are SLEEP TALKING.
  bool snoring =
      (
        isRhythmic &&
        avgDuration >= 80 &&
        avgDuration <= 2000
      );

  // =================================================
  // SLEEP TALKING
  // =================================================

  bool sleepTalking =
      (
        !isRhythmic &&
        avgDuration >= 80 &&
        avgDuration <= 1500
      );

  // =================================================
  // FINAL CLASSIFICATION
  // =================================================

  if (snoring)
  {
    soundEvent =
        "SNORING";

    snoringDetected =
        true;

    sleepTalkingDetected =
        false;

    Serial.println();
    Serial.println(
      ">>> SNORING DETECTED <<<"
    );

    Serial.println(
      "Starting vibration AFTER classification..."
    );

    // VIBRATION ONLY HERE
    startVibration(5000);
  }

  else if (sleepTalking)
  {
    soundEvent =
        "SLEEP TALKING";

    snoringDetected =
        false;

    sleepTalkingDetected =
        true;

    Serial.println();
    Serial.println(
      ">>> SLEEP TALKING DETECTED <<<"
    );

    Serial.println(
      "Starting vibration AFTER classification..."
    );

    // VIBRATION ONLY HERE
    startVibration(5000);
  }

  else
  {
    soundEvent =
        "NOISE";

    snoringDetected =
        false;

    sleepTalkingDetected =
        false;

    Serial.println();
    Serial.println(
      ">>> NOISE / UNCLASSIFIED <<<"
    );

    Serial.println(
      "No vibration."
    );
  }

  Serial.print(
    "FINAL CLASSIFICATION: "
  );

  Serial.println(
    soundEvent
  );

  Serial.println("================================");
}

// =====================================================
// START VIBRATION
// =====================================================

void startVibration(
    unsigned long duration
)
{
  digitalWrite(
    MOTOR_PIN,
    HIGH
  );

  vibrationActive =
      true;

  vibrationStart =
      millis();

  vibrationDuration =
      duration;

  Serial.print(
    "VIBRATION STARTED FOR "
  );

  Serial.print(
    duration
  );

  Serial.println(" ms");
}

// =====================================================
// HANDLE VIBRATION
// =====================================================

void handleVibration()
{
  if (!vibrationActive)
  {
    return;
  }

  if (
      millis() -
      vibrationStart >=
      vibrationDuration
     )
  {
    digitalWrite(
      MOTOR_PIN,
      LOW
    );

    vibrationActive =
        false;

    snoringDetected =
        false;

    sleepTalkingDetected =
        false;

    soundEvent = "NONE";

    Serial.println(
      "VIBRATION FINISHED — classification flags cleared"
    );
  }
}

// =====================================================
// RESTLESSNESS VIBRATION
// =====================================================

void restlessVibration()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(
      MOTOR_PIN,
      HIGH
    );

    delay(300);

    digitalWrite(
      MOTOR_PIN,
      LOW
    );

    delay(300);
  }
}

// =====================================================
// RESET WHEN PERSON LEAVES PILLOW
// =====================================================

void resetSleepData()
{
  eventCount = 0;

  lastPulseEnd = 0;

  pulseStart = 0;

  inPulse = false;

  postureChangeCount = 0;

  lastPosition = "";

  currentPosition = "UNKNOWN";

  liveSoundEvent = "QUIET";

  soundEvent =
      "NO SIGNIFICANT SOUND";

  snoringDetected = false;

  sleepTalkingDetected = false;

  digitalWrite(
    MOTOR_PIN,
    LOW
  );

  vibrationActive = false;
}

// =====================================================
// HX711 READ
// =====================================================

void handleWeight(unsigned long now)
{
  if (
      now - lastWeightRead <
      WEIGHT_INTERVAL
     )
  {
    return;
  }

  lastWeightRead =
      now;

  if (
      scale.wait_ready_timeout(100)
     )
  {
    float newWeight =
        scale.get_units(3);

    if (newWeight < 0)
    {
      newWeight = 0;
    }

    weight_g =
        newWeight;

    Serial.print("WEIGHT: ");
    Serial.print(
      weight_g,
      1
    );

    Serial.println(" g");
  }
}

// =====================================================
// AUTOMATIC TARE
// =====================================================

void tareLoadCell()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("LOAD CELL TARE");
  Serial.println("REMOVE ALL WEIGHT");
  Serial.println("Taring in 5 seconds...");
  Serial.println("================================");

  delay(5000);

  if (
      scale.wait_ready_timeout(1000)
     )
  {
    scale.tare(15);

    Serial.print(
      "Tare offset: "
    );

    Serial.println(
      scale.get_offset()
    );

    Serial.println(
      "TARE COMPLETE"
    );
  }
  else
  {
    Serial.println(
      "HX711 NOT READY - TARE FAILED"
    );
  }
}

// =====================================================
// WIFI
// =====================================================

void connectWiFi()
{
  Serial.println();
  Serial.print(
    "Connecting to WiFi: "
  );

  Serial.println(
    WIFI_SSID
  );

  WiFi.mode(
    WIFI_STA
  );

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  int attempts = 0;

  while (
      WiFi.status() != WL_CONNECTED &&
      attempts < 30
     )
  {
    delay(500);

    Serial.print(".");

    attempts++;
  }

  Serial.println();

  if (
      WiFi.status() ==
      WL_CONNECTED
     )
  {
    Serial.println(
      "WiFi CONNECTED!"
    );

    Serial.print(
      "ESP32 IP: "
    );

    Serial.println(
      WiFi.localIP()
    );

    Serial.print(
      "Server: "
    );

    Serial.println(
      SERVER_URL
    );
  }
  else
  {
    Serial.println(
      "WiFi CONNECTION FAILED"
    );
  }
}

// =====================================================
// SEND DATA TO FLASK
// =====================================================

void sendDataToServer()
{
  if (
      WiFi.status() !=
      WL_CONNECTED
     )
  {
    connectWiFi();

    return;
  }

  HTTPClient http;

  http.begin(
    SERVER_URL
  );

  http.addHeader(
    "Content-Type",
    "application/json"
  );

  // ================================================
  // Remaining seconds in current 60-sec window
  // ================================================

  unsigned long elapsed =
      millis() -
      windowStart;

  int secondsRemaining = 0;

  if (elapsed < WINDOW_DURATION)
  {
    secondsRemaining =
        (WINDOW_DURATION - elapsed) / 1000;
  }

  // ================================================
  // JSON
  // ================================================

  String json = "{";

  json +=
      "\"sound\":" +
      String(
        soundLevel,
        1
      );

  json +=
      ",\"pressure\":" +
      String(
        weight_g,
        1
      );

  json +=
      ",\"weight_g\":" +
      String(
        weight_g,
        1
      );

  json +=
      ",\"position\":\"" +
      currentPosition +
      "\"";

  json +=
      ",\"accel_magnitude\":" +
      String(
        accelerationMagnitude,
        2
      );

  json +=
      ",\"roll\":" +
      String(
        roll,
        1
      );

  json +=
      ",\"pitch\":" +
      String(
        pitch,
        1
      );

  // ================================================
  // IMMEDIATE SOUND STATUS
  // ================================================

  json +=
      ",\"live_sound\":\"" +
      liveSoundEvent +
      "\"";

  // ================================================
  // LAST COMPLETED CLASSIFICATION
  // ================================================

  json +=
      ",\"sound_event\":\"" +
      soundEvent +
      "\"";

  json +=
      ",\"snoring\":" +
      String(
        snoringDetected
        ? "true"
        : "false"
      );

  json +=
      ",\"sleep_talking\":" +
      String(
        sleepTalkingDetected
        ? "true"
        : "false"
      );

  json +=
      ",\"classification_id\":" +
      String(
        classificationId
      );

  // ================================================
  // CLASSIFICATION WINDOW
  // ================================================

  json +=
      ",\"classification_seconds_remaining\":" +
      String(
        secondsRemaining
      );

  json +=
      ",\"sound_events\":" +
      String(
        eventCount
      );

  json +=
      ",\"vibration\":" +
      String(
        vibrationActive
        ? "true"
        : "false"
      );

  json += "}";

  Serial.println();
  Serial.println(
    "Sending data:"
  );

  Serial.println(
    json
  );

  int response =
      http.POST(
        json
      );

  Serial.print(
    "HTTP response: "
  );

  Serial.println(
    response
  );

  if (response > 0)
  {
    String serverResponse =
        http.getString();

    Serial.print(
      "Server response: "
    );

    Serial.println(
      serverResponse
    );
  }
  else
  {
    Serial.print(
      "HTTP ERROR: "
    );

    Serial.println(
      http.errorToString(
        response
      )
    );
  }

  http.end();
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1000);

  // =================================================
  // HX711
  // =================================================

  scale.begin(
    HX711_DT,
    HX711_SCK
  );

  scale.set_scale(
    calibrationFactor
  );

  tareLoadCell();

  // =================================================
  // MOTOR
  // =================================================

  pinMode(
    MOTOR_PIN,
    OUTPUT
  );

  digitalWrite(
    MOTOR_PIN,
    LOW
  );

  // =================================================
  // MPU6500
  // =================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(
    400000
  );

  delay(500);

  writeRegister(
    PWR_MGMT_1,
    0x00
  );

  writeRegister(
    ACCEL_CONFIG,
    0x00
  );

  writeRegister(
    GYRO_CONFIG,
    0x00
  );

  delay(500);

  // =================================================
  // INMP441
  // =================================================

  i2sInit();

  calibrateMic();

  // =================================================
  // WIFI
  // =================================================

  connectWiFi();

  // =================================================
  // TIMERS
  // =================================================

  windowStart =
      millis();

  postureWindowStart =
      millis();

  lastWeightRead =
      millis();

  Serial.println();
  Serial.println("================================");
  Serial.println("       SLEEP MONITOR");
  Serial.println("================================");

  Serial.println(
    "System ready."
  );

  Serial.println(
    "Sound classification window: 60 seconds"
  );

  Serial.println(
    "Vibration occurs ONLY after classification."
  );
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  unsigned long now =
      millis();

  // =================================================
  // WEIGHT
  // =================================================

  handleWeight(now);

  // =================================================
  // PERSON ON PILLOW
  // =================================================

  if (
      weight_g >
      WEIGHT_THRESHOLD
     )
  {
    // -----------------------------------------------
    // MPU
    // -----------------------------------------------

    handlePosture(now);

    // -----------------------------------------------
    // MICROPHONE
    // -----------------------------------------------

    handleMic(now);

    // -----------------------------------------------
    // VIBRATION
    //
    // This ONLY handles an already-approved vibration.
    // It does NOT start vibration.
    // -----------------------------------------------

    handleVibration();

    // -----------------------------------------------
    // 60 SECOND SOUND CLASSIFICATION
    // -----------------------------------------------

    now = millis();

    if (
        now - windowStart >=
        WINDOW_DURATION
       )
    {
      // =============================================
      // CLASSIFY FIRST
      // =============================================

      classifyPattern();

      // =============================================
      // RESET SOUND COLLECTION
      // =============================================

      eventCount = 0;

      lastPulseEnd = 0;

      pulseStart = 0;

      inPulse = false;

      // =============================================
      // START NEW 60 SECOND WINDOW
      // =============================================

      windowStart =
          now;

      Serial.println();
      Serial.println(
        "NEW 60 SECOND SOUND WINDOW STARTED"
      );
    }

    // -----------------------------------------------
    // RESTLESSNESS
    // -----------------------------------------------

    if (
        now - postureWindowStart >=
        POSTURE_WINDOW
       )
    {
      Serial.print(
        "Posture changes: "
      );

      Serial.println(
        postureChangeCount
      );

      if (
          postureChangeCount >
          RESTLESS_THRESHOLD
         )
      {
        Serial.println(
          "RESTLESSNESS DETECTED"
        );

        // restlessVibration() disabled — vibration only after sound classification
        Serial.println("(vibration skipped — only sound classification triggers motor)");
      }

      postureChangeCount = 0;

      postureWindowStart =
          now;
    }
  }

  // =================================================
  // NOBODY ON PILLOW
  // =================================================

  else
  {
    digitalWrite(
      MOTOR_PIN,
      LOW
    );

    vibrationActive =
        false;

    resetSleepData();

    // Don't accumulate time while
    // nobody is on the pillow.

    windowStart =
        millis();

    postureWindowStart =
        millis();

    delay(10);
  }

  // =================================================
  // SEND TO WEBSITE EVERY 1 SECOND
  // =================================================

  static unsigned long lastSend = 0;

  if (
      millis() - lastSend >=
      1000
     )
  {
    lastSend =
        millis();

    sendDataToServer();
  }
}
