#include "Sampler.h"
#include "VoltageMap.h"

namespace dcmotor {

namespace {
Sampler* s_instance = nullptr;
}

void IRAM_ATTR Sampler::isrTrampoline() {
  if (s_instance) s_instance->onPulse();
}

void Sampler::timerTrampoline(void* arg) {
  static_cast<Sampler*>(arg)->onTimer();
}

void Sampler::onPulse() {
  portENTER_CRITICAL_ISR(&pulseMux_);
  pulseCount_ = pulseCount_ + 1;
  portEXIT_CRITICAL_ISR(&pulseMux_);
}

void Sampler::onTimer() {
  using namespace config;

  const float vmotV = analogReadMilliVolts(VMOT_PIN) / 1000.0f;
  const float vgenV = analogReadMilliVolts(VGEN_PIN) / 1000.0f;

  portENTER_CRITICAL(&pulseMux_);
  const uint32_t pulses = pulseCount_;
  pulseCount_ = 0;
  portEXIT_CRITICAL(&pulseMux_);

  const float vmotMV = adcToMotor(vmotV);
  const float vgenMV = adcToMotor(vgenV);

  const uint32_t rawHz   = pulses * SAMPLE_HZ;
  const int16_t  speedHz = (rawHz > (uint32_t)INT16_MAX)
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

  const float   vmotAvg  = vmotSum_  / filled_;
  const float   vgenAvg  = vgenSum_  / filled_;
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

void Sampler::begin() {
  using namespace config;

  s_instance = this;

  analogReadResolution(12);
  analogSetPinAttenuation(VMOT_PIN, ADC_11db);
  analogSetPinAttenuation(VGEN_PIN, ADC_11db);

  pinMode(SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), &Sampler::isrTrampoline, RISING);

  esp_timer_create_args_t args = {};
  args.callback              = &Sampler::timerTrampoline;
  args.arg                   = this;
  args.dispatch_method       = ESP_TIMER_TASK;
  args.name                  = "adc_sample";
  args.skip_unhandled_events = true;

  ESP_ERROR_CHECK(esp_timer_create(&args, &timerHandle_));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timerHandle_, SAMPLE_US));
}

bool Sampler::readIfNew(Measurements& out) {
  portENTER_CRITICAL(&stateMux_);
  const bool fresh = newSample_;
  if (fresh) {
    out        = latest_;
    newSample_ = false;
  }
  portEXIT_CRITICAL(&stateMux_);
  return fresh;
}

void Sampler::readLatest(Measurements& out) {
  portENTER_CRITICAL(&stateMux_);
  out = latest_;
  portEXIT_CRITICAL(&stateMux_);
}

}  // namespace dcmotor
