// ════════════════════════════════════════════════════════════════════
//  ESP32-C6  —  Motor Voltage & Speed Reader  (100 Hz, standalone)
//  GPIO4  : vmot  (ADC1_CH4, 11 dB attenuation → ~0–3.9 V)
//  GPIO5  : vgen  (ADC1_CH5, 11 dB attenuation → ~0–3.9 V)
//  GPIO14 : speed (digital pulse input, rising-edge counting)
//  GPIO20 : simulator output — 1000 Hz square wave → 3000 RPM
//           *** connect GPIO20 to GPIO14 with a jumper to test ***
//
//  Voltage mapping (piecewise linear around zero-cross):
//    1.45 V → -12 V motor voltage
//    1.65 V →   0 V (zero-cross)
//    3.80 V → +12 V motor voltage
//
//  Speed unit: pulses/second (Hz)
//  Encoder: 20 pulses/revolution → RPM = speed_Hz * 60 / 20
// ════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include "esp_timer.h"

// ── Pin & timing constants ────────────────────────────────────────────────────
constexpr uint8_t  VMOT_PIN  = 4;
constexpr uint8_t  VGEN_PIN  = 5;
constexpr uint8_t  SPEED_PIN = 14;

constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;   // 10 000 µs

// ── Voltage mapping constants ─────────────────────────────────────────────────
constexpr float   ZERO_CROSS_V = 1.65f;
constexpr float   MAX_SENSOR_V = 3.80f;
constexpr float   MIN_SENSOR_V = 1.45f;
constexpr float   MOTOR_V_MAX  = 12.0f;
constexpr uint8_t MA_WINDOW    = 8;

// ── Encoder ───────────────────────────────────────────────────────────────────
constexpr uint8_t  PULSES_PER_REV = 20;    // encoder resolution

// ── Pulse simulator (connect GPIO20 → GPIO14 with a jumper to test) ──────────
constexpr uint8_t  SIM_PIN        = 20;    // simulator output pin
constexpr uint32_t SIM_PULSE_HZ   = 1000; // 1000 pulse/s = 3000 RPM

// ── Shared measurement state — written by timer task, read by loop() ──────────
static portMUX_TYPE g_mux      = portMUX_INITIALIZER_UNLOCKED;
volatile float   g_vmot        = 0.0f;
volatile float   g_vgen        = 0.0f;
volatile int16_t g_speed       = 0;     // pulses / second
volatile float   g_rpm         = 0.0f; // revolutions / minute
volatile bool    g_newSample   = false;

// ── Pulse counter — written by GPIO ISR, consumed by timer task ───────────────
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

  // Read ADC (safe in task context)
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

  // RPM from smoothed speed — encoder has 20 pulses per revolution
  const float rpmAvg = static_cast<float>(speedAvg) * 60.0f / PULSES_PER_REV;

  portENTER_CRITICAL(&g_mux);
  g_vmot      = vmotAvg;
  g_vgen      = vgenAvg;
  g_speed     = speedAvg;
  g_rpm       = rpmAvg;
  g_newSample = true;
  portEXIT_CRITICAL(&g_mux);
}

// ═══════════════════════════════════════════════════════════
//  setup() and loop() — required by Arduino
// ═══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);   // wait for USB serial to open

  // ADC: 12-bit, 11 dB attenuation → ~0–3.9 V input range
  analogReadResolution(12);
  analogSetPinAttenuation(VMOT_PIN, ADC_11db);
  analogSetPinAttenuation(VGEN_PIN, ADC_11db);

  // GPIO14: digital pulse input (encoder / tachometer / hall sensor)
  pinMode(SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), onSpeedPulse, RISING);

  // ── Pulse simulator: 1000 Hz square wave on GPIO20 → 3000 RPM ───────────
  // LEDC (PWM) generates the signal in hardware — zero CPU overhead.
  // Remove or comment out these two lines when a real encoder is connected.
  ledcAttach(SIM_PIN, SIM_PULSE_HZ, 8);  // 1000 Hz, 8-bit resolution
  ledcWrite(SIM_PIN, 128);               // 50% duty cycle

  // Start 100 Hz periodic timer via esp_timer (Arduino core 3.x compatible)
  esp_timer_create_args_t args = {};
  args.callback              = onSampleTimer;
  args.arg                   = nullptr;
  args.dispatch_method       = ESP_TIMER_TASK;
  args.name                  = "adc_sample";
  args.skip_unhandled_events = true;

  ESP_ERROR_CHECK(esp_timer_create(&args, &g_sampleTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(g_sampleTimer, SAMPLE_US));

  Serial.println("Ready — 100 Hz sampling started");
  Serial.println("speed (pulse/s) | rpm (RPM) | vmot (V) | vgen (V)");
}

void loop() {
  // Check if the timer task has written a new sample
  bool newData = false;
  portENTER_CRITICAL(&g_mux);
  newData     = g_newSample;
  g_newSample = false;
  portEXIT_CRITICAL(&g_mux);

  if (!newData) return;   // nothing new yet, come back later

  // Thread-safe snapshot of latest values
  float   vmot, vgen, rpm;
  int16_t speed;
  portENTER_CRITICAL(&g_mux);
  vmot  = g_vmot;
  vgen  = g_vgen;
  speed = g_speed;
  rpm   = g_rpm;
  portEXIT_CRITICAL(&g_mux);

  // Print at 100 Hz (throttle here if too fast for your terminal)
  Serial.print(speed);    Serial.print(" pulse/s | ");
  Serial.print(rpm, 1);   Serial.print(" RPM | ");
  Serial.print(vmot, 3);  Serial.print(" V | ");
  Serial.print(vgen, 3);  Serial.println(" V");
}
