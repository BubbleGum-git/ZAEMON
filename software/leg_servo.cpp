#include "leg_servo.h"

// LEDC API: core 2.x works with channels, core 3.x works directly with pins
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define SERVO_PWM_ATTACH(pin, ch) ledcAttach((pin), SERVO_PWM_FREQ, SERVO_PWM_BITS)
#define SERVO_PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
#define SERVO_PWM_ATTACH(pin, ch) \
  do { \
    ledcSetup((ch), SERVO_PWM_FREQ, SERVO_PWM_BITS); \
    ledcAttachPin((pin), (ch)); \
  } while (0)
#define SERVO_PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

LegServos legs;

void LegServos::begin() {
  SERVO_PWM_ATTACH(SERVO_L_PIN, SERVO_L_LEDC_CH);
  SERVO_PWM_ATTACH(SERVO_R_PIN, SERVO_R_LEDC_CH);
  _curL = _curR = _tgtL = _tgtR = LEG_EXT_MIN_DEG;  // start fully short
  _off = false;
  _lastUs = micros();
  write();
}

void LegServos::setTarget(float extL, float extR, bool immediate) {
  _tgtL = leg::clampf(extL, LEG_EXT_MIN_DEG, LEG_EXT_JUMP_MAX_DEG);
  _tgtR = leg::clampf(extR, LEG_EXT_MIN_DEG, LEG_EXT_JUMP_MAX_DEG);
  if (immediate) _imm = true;
}

void LegServos::update() {
  unsigned long now = micros();
  float dt = (now - _lastUs) * 1e-6f;
  _lastUs = now;
  if (dt < 0.0f || dt > 0.05f) dt = 0.05f;  // guard against hiccups / first call

  if (_imm) {  // jump moves: go straight to the target at full servo speed
    _curL = _tgtL;
    _curR = _tgtR;
    _imm = false;
  } else {
    float step = LEG_SLEW_DEG_PER_S * dt;
    _curL = leg::slewTo(_curL, _tgtL, step);
    _curR = leg::slewTo(_curR, _tgtR, step);
  }
  if (!_off) write();
}

void LegServos::write() {
  uint32_t dL = leg::usToDuty(leg::angleToUs(leg::leftAngleFromExt(_curL)));
  uint32_t dR = leg::usToDuty(leg::angleToUs(leg::rightAngleFromExt(_curR)));
  if (dL != _lastDutyL) {
    SERVO_PWM_WRITE(SERVO_L_PIN, SERVO_L_LEDC_CH, dL);
    _lastDutyL = dL;
  }
  if (dR != _lastDutyR) {
    SERVO_PWM_WRITE(SERVO_R_PIN, SERVO_R_LEDC_CH, dR);
    _lastDutyR = dR;
  }
}

void LegServos::off() {
  _off = true;
  SERVO_PWM_WRITE(SERVO_L_PIN, SERVO_L_LEDC_CH, 0);
  SERVO_PWM_WRITE(SERVO_R_PIN, SERVO_R_LEDC_CH, 0);
  _lastDutyL = _lastDutyR = 0xFFFFFFFF;
}

void LegServos::on() {
  _off = false;
  _lastUs = micros();
  write();
}
