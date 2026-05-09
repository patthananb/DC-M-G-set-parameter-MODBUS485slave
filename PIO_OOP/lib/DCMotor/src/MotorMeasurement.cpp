#include "MotorMeasurement.h"

namespace config { namespace motor {

namespace {
MotorMeasurement* s_instance = nullptr;
}

void IRAM_ATTR MotorMeasurement::speedIsr() {
  if (s_instance) s_instance->onSpeedPulse();
}

void MotorMeasurement::timerCallback(void* arg) {
  static_cast<MotorMeasurement*>(arg)->onSampleTimer();
}

float adcToMotor(float v) {
  using namespace settings;

  float motorV;
  if (v >= ZERO_CROSS_V) {
    motorV = ((v - ZERO_CROSS_V) / (MAX_SENSOR_V - ZERO_CROSS_V)) * MOTOR_V_MAX;
  } else {
    motorV = ((v - ZERO_CROSS_V) / (ZERO_CROSS_V - MIN_SENSOR_V)) * MOTOR_V_MAX;
  }

  if (motorV > MOTOR_V_MAX) motorV = MOTOR_V_MAX;
  if (motorV < -MOTOR_V_MAX) motorV = -MOTOR_V_MAX;
  return motorV;
}

void MotorMeasurement::begin(bool enableSimulator) {
  using namespace settings;

  s_instance = this;

  analogReadResolution(ADC_RESOLUTION_BITS);
  analogSetPinAttenuation(VMOT_PIN, ADC_11db);
  analogSetPinAttenuation(VGEN_PIN, ADC_11db);

  pinMode(SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), &MotorMeasurement::speedIsr, RISING);

  esp_timer_create_args_t args = {};
  args.callback              = &MotorMeasurement::timerCallback;
  args.arg                   = this;
  args.dispatch_method       = ESP_TIMER_TASK;
  args.name                  = "motor_sample";
  args.skip_unhandled_events = true;

  ESP_ERROR_CHECK(esp_timer_create(&args, &timerHandle_));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle_, SAMPLE_US));

  simulatorEnabled_ = enableSimulator;
  if (simulatorEnabled_) startSimulator();
}

bool MotorMeasurement::readIfNew(Measurements& out) {
  portENTER_CRITICAL(&stateMux_);
  const bool fresh = newSample_;
  if (fresh) {
    out = latest_;
    newSample_ = false;
  }
  portEXIT_CRITICAL(&stateMux_);
  return fresh;
}

void MotorMeasurement::readLatest(Measurements& out) {
  portENTER_CRITICAL(&stateMux_);
  out = latest_;
  portEXIT_CRITICAL(&stateMux_);
}

void MotorMeasurement::stopSimulator() {
  if (!simulatorEnabled_) return;

  ledcDetach(settings::SIM_PIN);
  simulatorEnabled_ = false;
}

void MotorMeasurement::startSimulator() {
  ledcAttach(settings::SIM_PIN, settings::SIM_PULSE_HZ, settings::SIM_LEDC_BITS);
  ledcWrite(settings::SIM_PIN, settings::SIM_LEDC_DUTY);
}

void MotorMeasurement::onSpeedPulse() {
  portENTER_CRITICAL_ISR(&pulseMux_);
  pulseCount_ = pulseCount_ + 1;
  portEXIT_CRITICAL_ISR(&pulseMux_);
}

void MotorMeasurement::onSampleTimer() {
  using namespace settings;

  const float vmotV = analogReadMilliVolts(VMOT_PIN) / MILLIVOLTS_PER_VOLT;
  const float vgenV = analogReadMilliVolts(VGEN_PIN) / MILLIVOLTS_PER_VOLT;

  portENTER_CRITICAL(&pulseMux_);
  const uint32_t pulses = pulseCount_;
  pulseCount_ = 0;
  portEXIT_CRITICAL(&pulseMux_);

  const float vmotMV = adcToMotor(vmotV);
  const float vgenMV = adcToMotor(vgenV);

  const uint32_t rawHz = pulses * SAMPLE_HZ;
  const int16_t speedHz = (rawHz > static_cast<uint32_t>(INT16_MAX))
                            ? INT16_MAX
                            : static_cast<int16_t>(rawHz);

  vmotSum_  -= vmotBuf_[idx_];
  vgenSum_  -= vgenBuf_[idx_];
  speedSum_ -= speedBuf_[idx_];

  vmotBuf_[idx_]  = vmotMV;
  vgenBuf_[idx_]  = vgenMV;
  speedBuf_[idx_] = speedHz;

  vmotSum_  += vmotMV;
  vgenSum_  += vgenMV;
  speedSum_ += speedHz;

  if (filled_ < MA_WINDOW) filled_++;
  idx_ = (idx_ + 1) % MA_WINDOW;

  const float   vmotAvg  = vmotSum_ / filled_;
  const float   vgenAvg  = vgenSum_ / filled_;
  const int16_t speedAvg = static_cast<int16_t>(speedSum_ / filled_);
  const float   rpmAvg   = static_cast<float>(speedAvg) * 60.0f / PULSES_PER_REV;

  portENTER_CRITICAL(&stateMux_);
  latest_.vmot    = vmotAvg;
  latest_.vgen    = vgenAvg;
  latest_.vmotRaw = vmotV;
  latest_.vgenRaw = vgenV;
  latest_.speed   = speedAvg;
  latest_.rpm     = rpmAvg;
  newSample_      = true;
  portEXIT_CRITICAL(&stateMux_);
}

}  // namespace motor
}  // namespace config
