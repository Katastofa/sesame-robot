// SesameServo.h - Software servo driver for ESP32-C3 (Arduino Core 3.x)
// Uses 1 hardware timer + direct GPIO register writes. No LEDC channels needed.
// Supports up to 8 servos at 50Hz with smooth interpolation.
// Optimized for single-core ESP32-C3 (minimal ISR overhead, WiFi-friendly).

#pragma once
#include <Arduino.h>
#include "soc/gpio_struct.h"

#define SOFT_SERVO_MAX       8
#define SERVO_TICK_US        20       // Timer fires every 20us (WiFi-safe on single-core)
#define SERVO_CYCLE_US       20000    // 50Hz = 20ms period
#define SERVO_CYCLE_TICKS    (SERVO_CYCLE_US / SERVO_TICK_US)  // 1000

class SoftServoClass {
public:
  SoftServoClass() {}

  // Initialize hardware timer (timerNo is ignored — auto-selected)
  void useTimer(uint8_t timerNo) {
    (void)timerNo;
    if (_timer) return;
    _timer = timerBegin(1000000);                    // 1MHz = 1us tick
    timerAttachInterrupt(_timer, onTimer);
    timerAlarm(_timer, SERVO_TICK_US, true, 0);      // Fire every SERVO_TICK_US
  }

  // Register a servo on 'pin'. Returns index (0-7) or -1 on failure.
  int8_t setupServo(uint8_t pin, uint16_t minUs = 544, uint16_t maxUs = 2400) {
    if (_count >= SOFT_SERVO_MAX) return -1;
    int8_t i = _count++;
    _pin[i]      = pin;
    _minUs[i]    = minUs;
    _maxUs[i]    = maxUs;
    _rangeUs[i]  = maxUs - minUs;
    uint16_t center = (minUs + maxUs) / 2;
    _pulseUs[i]  = center;
    _targetUs[i] = center;
    _pos[i]      = 90;
    _enabled[i]  = false;          // Start disabled to prevent brownout
    _pinMask[i]  = (1UL << pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return i;
  }

  // Set target position (0-180°) with smooth interpolation.
  // First move after setup is instant (no interpolation).
  bool setPosition(uint8_t i, uint16_t deg) {
    if (i >= _count) return false;
    if (deg > 180) deg = 180;
    _pos[i] = deg;
    _targetUs[i] = _minUs[i] + (uint16_t)(((uint32_t)deg * _rangeUs[i]) / 180);
    if (!_enabled[i]) {
      _enabled[i] = true;
      _pulseUs[i] = _targetUs[i];  // First move is instant
      _rebuildSetMask();
    }
    return true;
  }

  // Set speed for smooth interpolation (degrees per second).
  // Use 0 for instant movement (no interpolation).
  // Typical: 500=smooth, 1000=balanced, 2000=snappy, 0=instant.
  void setSpeed(uint16_t degPerSec) {
    if (degPerSec == 0) {
      _stepUs = 0xFFFF;  // Instant — no interpolation
      return;
    }
    // Convert deg/sec to us of pulse change per 20ms PWM cycle.
    // Uses actual servo range (average ~2197us for 732-2929).
    // stepUs = degPerSec * avgRange * 0.020 / 180
    _stepUs = max((uint16_t)1, (uint16_t)(((uint32_t)degPerSec * 2197UL * 20UL) / 180000UL));
  }

  // Get last set target position in degrees, or -1 on invalid index
  int getPosition(uint8_t i) {
    return (i < _count) ? (int)_pos[i] : -1;
  }

  // Check if any servos are still moving toward their targets
  bool isMoving() {
    for (uint8_t i = 0; i < _count; i++) {
      if (_enabled[i] && _pulseUs[i] != _targetUs[i]) return true;
    }
    return false;
  }

  bool enable(uint8_t i) {
    if (i >= _count) return false;
    _enabled[i] = true;
    _rebuildSetMask();
    return true;
  }

  bool disable(uint8_t i) {
    if (i >= _count) return false;
    _enabled[i] = false;
    _rebuildSetMask();
    digitalWrite(_pin[i], LOW);
    return true;
  }

  void enableAll()  { for (uint8_t i = 0; i < _count; i++) enable(i); }
  void disableAll() { for (uint8_t i = 0; i < _count; i++) disable(i); }

private:
  hw_timer_t* _timer = nullptr;

  static uint8_t          _count;
  static uint8_t          _pin[SOFT_SERVO_MAX];
  static uint16_t         _minUs[SOFT_SERVO_MAX];
  static uint16_t         _maxUs[SOFT_SERVO_MAX];
  static uint16_t         _rangeUs[SOFT_SERVO_MAX];
  static uint32_t         _pinMask[SOFT_SERVO_MAX];
  static volatile uint32_t _setMask;
  static volatile uint16_t _pulseUs[SOFT_SERVO_MAX];
  static volatile uint16_t _targetUs[SOFT_SERVO_MAX];
  static volatile uint16_t _pos[SOFT_SERVO_MAX];
  static volatile bool     _enabled[SOFT_SERVO_MAX];
  static volatile uint16_t _tick;
  static uint16_t          _stepUs;

  void _rebuildSetMask() {
    uint32_t m = 0;
    for (uint8_t i = 0; i < _count; i++) {
      if (_enabled[i]) m |= _pinMask[i];
    }
    _setMask = m;
  }

  // ISR: fires every SERVO_TICK_US microseconds — kept minimal for WiFi coexistence
  static void IRAM_ATTR onTimer() {
    uint16_t t = _tick;

    if (t == 0) {
      // Interpolation at start of each 20ms cycle
      uint16_t step = _stepUs;
      for (uint8_t i = 0; i < _count; i++) {
        uint16_t cur = _pulseUs[i];
        uint16_t tgt = _targetUs[i];
        if (cur != tgt) {
          int16_t diff = (int16_t)tgt - (int16_t)cur;
          if (step < 0xFFFF) {
            if (diff > (int16_t)step) diff = step;
            else if (diff < -(int16_t)step) diff = -step;
          }
          _pulseUs[i] = cur + diff;
        }
      }
      // All enabled pins HIGH
      GPIO.out_w1ts.val = _setMask;
    } else {
      // Turn off pins whose pulse width has elapsed
      uint16_t us = t * SERVO_TICK_US;
      uint32_t clr = 0;
      for (uint8_t i = 0; i < _count; i++) {
        if (_enabled[i] && us >= _pulseUs[i]) {
          clr |= _pinMask[i];
        }
      }
      if (clr) GPIO.out_w1tc.val = clr;
    }

    if (++t >= SERVO_CYCLE_TICKS) t = 0;
    _tick = t;
  }
};

// --- Static member definitions ---
uint8_t          SoftServoClass::_count = 0;
uint8_t          SoftServoClass::_pin[SOFT_SERVO_MAX] = {};
uint16_t         SoftServoClass::_minUs[SOFT_SERVO_MAX] = {};
uint16_t         SoftServoClass::_maxUs[SOFT_SERVO_MAX] = {};
uint16_t         SoftServoClass::_rangeUs[SOFT_SERVO_MAX] = {};
uint32_t         SoftServoClass::_pinMask[SOFT_SERVO_MAX] = {};
volatile uint32_t SoftServoClass::_setMask = 0;
volatile uint16_t SoftServoClass::_pulseUs[SOFT_SERVO_MAX] = {};
volatile uint16_t SoftServoClass::_targetUs[SOFT_SERVO_MAX] = {};
volatile uint16_t SoftServoClass::_pos[SOFT_SERVO_MAX] = {};
volatile bool     SoftServoClass::_enabled[SOFT_SERVO_MAX] = {};
volatile uint16_t SoftServoClass::_tick = 0;
uint16_t          SoftServoClass::_stepUs = 0xFFFF;  // Instant by default

// Global instance
SoftServoClass ESP32_ISR_Servos;
