// SoftServo.h - Software servo driver for ESP32-C3 (Arduino Core 3.x)
// Uses 1 hardware timer + direct GPIO register writes. No LEDC channels needed.
// Supports up to 8 servos at 50Hz with ~1.6° resolution.
// Drop-in API replacement for ESP32_C3_ISR_Servo.

#pragma once
#include <Arduino.h>
#include "soc/gpio_struct.h"

#define SOFT_SERVO_MAX       8
#define SERVO_TICK_US        20       // Timer fires every 20us
#define SERVO_CYCLE_US       20000    // 50Hz = 20ms period
#define SERVO_CYCLE_TICKS    (SERVO_CYCLE_US / SERVO_TICK_US)

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
    _pin[i]     = pin;
    _minUs[i]   = minUs;
    _maxUs[i]   = maxUs;
    _pulseUs[i] = (minUs + maxUs) / 2;   // center position
    _pos[i]     = 90;
    _enabled[i] = false;          // Start disabled to prevent brownout
    _pinMask[i] = (1UL << pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return i;
  }

  // Set servo position in degrees (0-180). Auto-enables if disabled.
  bool setPosition(uint8_t i, uint16_t deg) {
    if (i >= _count) return false;
    if (deg > 180) deg = 180;
    _pos[i]     = deg;
    _pulseUs[i] = map(deg, 0, 180, _minUs[i], _maxUs[i]);
    if (!_enabled[i]) {
      _enabled[i] = true;
      _rebuildSetMask();
    }
    return true;
  }

  // Get last set position in degrees, or -1 on invalid index
  int getPosition(uint8_t i) {
    return (i < _count) ? (int)_pos[i] : -1;
  }

  // Enable servo PWM output
  bool enable(uint8_t i) {
    if (i >= _count) return false;
    _enabled[i] = true;
    _rebuildSetMask();
    return true;
  }

  // Disable servo PWM output (servo goes limp)
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
  static uint32_t         _pinMask[SOFT_SERVO_MAX];
  static volatile uint32_t _setMask;           // OR of all enabled pin masks
  static volatile uint16_t _pulseUs[SOFT_SERVO_MAX];
  static volatile uint16_t _pos[SOFT_SERVO_MAX];
  static volatile bool     _enabled[SOFT_SERVO_MAX];
  static volatile uint16_t _tick;

  void _rebuildSetMask() {
    uint32_t m = 0;
    for (uint8_t i = 0; i < _count; i++) {
      if (_enabled[i]) m |= _pinMask[i];
    }
    _setMask = m;
  }

  // ISR: fires every SERVO_TICK_US microseconds
  static void IRAM_ATTR onTimer() {
    uint16_t t  = _tick;
    uint16_t us = t * SERVO_TICK_US;

    if (t == 0) {
      // Start of 20ms cycle — all enabled pins HIGH
      GPIO.out_w1ts.val = _setMask;
    }

    // Turn off pins whose pulse width has elapsed
    uint32_t clr = 0;
    uint8_t n = _count;
    for (uint8_t i = 0; i < n; i++) {
      if (_enabled[i] && us >= _pulseUs[i]) {
        clr |= _pinMask[i];
      }
    }
    if (clr) GPIO.out_w1tc.val = clr;

    t++;
    if (t >= SERVO_CYCLE_TICKS) t = 0;
    _tick = t;
  }
};

// --- Static member definitions ---
uint8_t          SoftServoClass::_count = 0;
uint8_t          SoftServoClass::_pin[SOFT_SERVO_MAX] = {};
uint16_t         SoftServoClass::_minUs[SOFT_SERVO_MAX] = {};
uint16_t         SoftServoClass::_maxUs[SOFT_SERVO_MAX] = {};
uint32_t         SoftServoClass::_pinMask[SOFT_SERVO_MAX] = {};
volatile uint32_t SoftServoClass::_setMask = 0;
volatile uint16_t SoftServoClass::_pulseUs[SOFT_SERVO_MAX] = {};
volatile uint16_t SoftServoClass::_pos[SOFT_SERVO_MAX] = {};
volatile bool     SoftServoClass::_enabled[SOFT_SERVO_MAX] = {};
volatile uint16_t SoftServoClass::_tick = 0;

// Global instance — same name as ESP32_C3_ISR_Servo for drop-in compatibility
SoftServoClass ESP32_ISR_Servos;
