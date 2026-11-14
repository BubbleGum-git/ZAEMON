#pragma once
/*
 * leg_math.h - pure math for the PWM leg servos (no Arduino dependency, so it can be
 * unit-tested on a PC). Everything is expressed as "extension" in degrees of servo
 * travel from fully short: 0 = short, 40 = max (jump only), 0..20 = normal range.
 */
#include <stdint.h>
#include "hw_config.h"

namespace leg {

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Web UI height slider (32..85) -> normal extension (0..20 deg)
inline float heightToExt(int h) {
  float t = (float)(h - LEG_UI_H_MIN) / (float)(LEG_UI_H_MAX - LEG_UI_H_MIN);
  t = clampf(t, 0.0f, 1.0f);
  return LEG_EXT_MIN_DEG + t * (LEG_EXT_NORMAL_MAX_DEG - LEG_EXT_MIN_DEG);
}

// Move cur toward target by at most maxStep
inline float slewTo(float cur, float target, float maxStep) {
  float d = target - cur;
  if (d > maxStep) return cur + maxStep;
  if (d < -maxStep) return cur - maxStep;
  return target;
}

// Extension (deg) -> servo angle (deg). Right servo: 0..40. Left servo: 180..140.
inline float rightAngleFromExt(float ext) {
  return clampf(ext + LEG_TRIM_R_DEG, LEG_EXT_MIN_DEG, LEG_EXT_JUMP_MAX_DEG);
}
inline float leftAngleFromExt(float ext) {
  return 180.0f - clampf(ext + LEG_TRIM_L_DEG, LEG_EXT_MIN_DEG, LEG_EXT_JUMP_MAX_DEG);
}

// Servo angle (0..180) -> pulse width in microseconds
inline uint32_t angleToUs(float angle) {
  angle = clampf(angle, 0.0f, 180.0f);
  return (uint32_t)(SERVO_US_MIN + (angle / 180.0f) * (float)(SERVO_US_MAX - SERVO_US_MIN) + 0.5f);
}

// Pulse width -> LEDC duty at SERVO_PWM_FREQ / SERVO_PWM_BITS
inline uint32_t usToDuty(uint32_t us) {
  const float period_us = 1000000.0f / (float)SERVO_PWM_FREQ;
  const float maxDuty = (float)((1UL << SERVO_PWM_BITS) - 1UL);
  return (uint32_t)((float)us / period_us * maxDuty + 0.5f);
}

}  // namespace leg
