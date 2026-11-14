#pragma once
#include <Arduino.h>
#include "hw_config.h"
#include "leg_math.h"

/*
 * Leg servo driver for two hobby PWM servos (LEDC, 50 Hz).
 * Replaces the original STS3032 serial-bus servo code.
 */
class LegServos {
public:
  void begin();                                             // attach pins, drive both legs fully short
  void setTarget(float extL, float extR, bool immediate = false);  // extension in degrees
  void update();                                            // call every loop: slews toward target
  void off();                                               // stop sending pulses (servos go limp)
  void on();                                                // resume holding position
  bool isOff() const { return _off; }
  float extL() const { return _curL; }
  float extR() const { return _curR; }

private:
  void write();
  float _tgtL = 0, _tgtR = 0, _curL = 0, _curR = 0;
  bool _imm = false;
  bool _off = false;
  unsigned long _lastUs = 0;
  uint32_t _lastDutyL = 0xFFFFFFFF, _lastDutyR = 0xFFFFFFFF;
};

extern LegServos legs;
