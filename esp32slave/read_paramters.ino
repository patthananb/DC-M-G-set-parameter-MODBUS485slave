// Sampling helper for esp32slave — no setup() / loop() here.
// Those live in esp32slave.ino alongside the Modbus RTU logic.
//
// ESP32-C6 measurement sampling — 100 Hz via esp_timer
// GPIO4  : vmot  (analog, ADC1_CH4, 11 dB attenuation → ~0–3.9 V)
// GPIO5  : vgen  (analog, ADC1_CH5, 11 dB attenuation → ~0–3.9 V)
// GPIO14 : speed (digital pulse input, rising-edge count per 10 ms window)
//
// Voltage mapping (piecewise linear around zero-cross):
//   1.45 V → -12 V motor voltage
//   1.65 V →   0 V motor voltage  (zero-cross)
//   3.80 V → +12 V motor voltage
//
// g_speed unit: pulses per second (Hz).  Convert to RPM:
//   RPM = g_speed * 60 / pulses_per_revolution

#include <Arduino.h>
#include "esp_timer.h"

constexpr uint8_t  VMOT_PIN  = 4;
constexpr uint8_t  VGEN_PIN  = 5;
constexpr uint8_t  SPEED_PIN = 14;

constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;   // 10 000 µs

constexpr float   ZERO_CROSS_V = 1.65f;
constexpr float   MAX_SENSOR_V = 3.80f;
constexpr float   MIN_SENSOR_V = 1.45f;
constexpr float   MOTOR_V_MAX  = 12.0f;
constexpr uint8_t MA_WINDOW    = 8;

// Shared measurement state — written by timer task, read by loop()
static portMUX_TYPE g_mux      = portMUX_INITIALIZER_UNLOCKED;
volatile float   g_vmot        = 0.0f;
volatile float   g_vgen        = 0.0f;
volatile int16_t g_speed       = 0;
volatile bool    g_newSample   = false;

// Pulse counter — written by GPIO ISR, consumed by timer task
static portMUX_TYPE g_pulseMux  = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t   g_pulseCount = 0;

static esp_timer_handle_t g_sampleTimer = nullptr;

// ── Voltage mapping ───────────────────────────────────────────────────────────

static float mapSensorToMotorVoltage(float v) {
  float motorV;
  if (v >= ZERO_CROSS_V) {
    motorV = ((v - ZERO_CROSS_V) / (MAX_SENSOR_V - ZERO_CROSS_V)) * MOTOR_V_MAX;
  } else {
    motorV = ((v - ZERO_CROSS_V) / (ZERO_CROSS_V - MIN_SENSOR_V)) * MOTOR_V_MAX;
  }
  if (motorV >  MOTOR_V_MAX) motorV =  MOTOR_V_MAX;
  if (motorV < -MOTOR_V_MAX) motorV = -MOTOR_V_MAX;
  return motorV;
}

// ── ISR: count rising edges from speed sensor on GPIO14 ──────────────────────

void IRAM_ATTR onSpeedPulse() {
  portENTER_CRITICAL_ISR(&g_pulseMux);
  g_pulseCount++;
  portEXIT_CRITICAL_ISR(&g_pulseMux);
}

// ── Timer callback: runs at 100 Hz in esp_timer task context ─────────────────

static void onSampleTimer(void* /*arg*/) {
  static float   vmotBuf[MA_WINDOW]  = {};
  static float   vgenBuf[MA_WINDOW]  = {};
  static int16_t speedBuf[MA_WINDOW] = {};
  static float   vmotSum  = 0.0f;
  static float   vgenSum  = 0.0f;
  static int32_t speedSum = 0;
  static uint8_t idx      = 0;
  static uint8_t filled   = 0;

  // analogReadMilliVolts is safe in task context
  const float vmotV = analogReadMilliVolts(VMOT_PIN) / 1000.0f;
  const float vgenV = analogReadMilliVolts(VGEN_PIN) / 1000.0f;

  // Snapshot and reset pulse counter atomically
  portENTER_CRITICAL(&g_pulseMux);
  const uint32_t pulses = g_pulseCount;
  g_pulseCount = 0;
  portEXIT_CRITICAL(&g_pulseMux);

  // Convert ADC voltage → motor voltage
  const float   vmotMV  = mapSensorToMotorVoltage(vmotV);
  const float   vgenMV  = mapSensorToMotorVoltage(vgenV);

  // Speed: pulses in this 10 ms window × 100 = pulses per second
  const uint32_t rawHz   = pulses * SAMPLE_HZ;
  const int16_t  speedHz = (rawHz > (uint32_t)INT16_MAX)
                           ? INT16_MAX
                           : static_cast<int16_t>(rawHz);

  // Moving-average filter (MA_WINDOW samples)
  vmotSum  -= vmotBuf[idx];
  vgenSum  -= vgenBuf[idx];
  speedSum -= speedBuf[idx];

  vmotBuf[idx]  = vmotMV;
  vgenBuf[idx]  = vgenMV;
  speedBuf[idx] = speedHz;

  vmotSum  += vmotMV;
  vgenSum  += vgenMV;
  speedSum += speedHz;

  if (filled < MA_WINDOW) filled++;
  idx = (idx + 1) % MA_WINDOW;

  const float   vmotAvg  = vmotSum  / filled;
  const float   vgenAvg  = vgenSum  / filled;
  const int16_t speedAvg = static_cast<int16_t>(speedSum / filled);

  portENTER_CRITICAL(&g_mux);
  g_vmot      = vmotAvg;
  g_vgen      = vgenAvg;
  g_speed     = speedAvg;
  g_newSample = true;
  portEXIT_CRITICAL(&g_mux);
}

// ── Public API (called from esp32slave.ino) ───────────────────────────────────

void initSampling100Hz() {
  // ADC: 12-bit, 11 dB attenuation → ~0–3.9 V input range
  analogReadResolution(12);
  analogSetPinAttenuation(VMOT_PIN, ADC_11db);
  analogSetPinAttenuation(VGEN_PIN, ADC_11db);

  // GPIO14: digital pulse input (encoder / tachometer / hall sensor)
  pinMode(SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), onSpeedPulse, RISING);

  // esp_timer — IDF-level, works on ESP32-C6 with Arduino core 3.x / IDF 5.x
  esp_timer_create_args_t args = {};
  args.callback              = onSampleTimer;
  args.arg                   = nullptr;
  args.dispatch_method       = ESP_TIMER_TASK;
  args.name                  = "adc_sample";
  args.skip_unhandled_events = true;

  ESP_ERROR_CHECK(esp_timer_create(&args, &g_sampleTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(g_sampleTimer, SAMPLE_US));
}

// Returns true exactly once per 10 ms tick so esp32slave.ino can gate
// serial debug prints on new data without busy-polling.
bool updateMeasurementsFromTimer() {
  portENTER_CRITICAL(&g_mux);
  const bool ready = g_newSample;
  g_newSample = false;
  portEXIT_CRITICAL(&g_mux);
  return ready;
}

void getLatestMeasurements(int16_t &speedOut, float &vmotOut, float &vgenOut) {
  portENTER_CRITICAL(&g_mux);
  speedOut = g_speed;
  vmotOut  = g_vmot;
  vgenOut  = g_vgen;
  portEXIT_CRITICAL(&g_mux);
}
