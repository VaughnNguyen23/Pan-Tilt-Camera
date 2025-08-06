#ifndef ESP32_SERVO_H
#define ESP32_SERVO_H

#include <Arduino.h>
#include "driver/ledc.h"

/**
 * ESP32-C6 PWM Servo Control Library
 *
 * A lightweight servo control library using ESP32's native LEDC PWM peripheral
 * Features:
 * - Multiple servo support (up to 8 servos)
 * - Configurable pulse width ranges
 * - Smooth movement with speed control
 * - Non-blocking sweep operations
 * - Compatible with ESP32-C6 and other ESP32 variants
 */

class ESP32Servo
{
private:
  // PWM Configuration
  static const int PWM_FREQ = 50;        // Standard servo frequency (50Hz)
  static const int PWM_RESOLUTION = 16;  // PWM resolution in bits
  static const int SERVO_PERIOD = 20000; // PWM period (20ms for 50Hz)

  // Instance variables
  int _pin;          // GPIO pin for servo
  int _channel;      // PWM channel (0-7)
  int _minPulse;     // Minimum pulse width (microseconds)
  int _maxPulse;     // Maximum pulse width (microseconds)
  int _currentAngle; // Current servo position
  bool _isAttached;  // Whether servo is initialized

  // Static variables for channel management
  static bool _channelUsed[8];  // Track which PWM channels are in use
  static bool _timerConfigured; // Track if LEDC timer is configured

  // Private methods
  uint32_t angleToDutyCycle(int angle);
  int getNextAvailableChannel();
  void configureTimer();

public:
  // Constructors
  ESP32Servo();
  ~ESP32Servo();

  // Basic servo control finished
  bool attach(int pin);
  bool attach(int pin, int minPulse, int maxPulse);
  void detach();
  void write(int angle);
  int read();
  bool attached();

  // Advanced features not finished
  void writeMicroseconds(int microseconds);
  void smoothWrite(int targetAngle, int stepDelay = 20);
  void slowWrite(int targetAngle, int totalTimeMs);
  void moveSlowly(int startAngle, int endAngle, int stepDelayMs = 50);
  void openSlowly(int openAngle = 180, int stepDelayMs = 30);
  void closeSlowly(int closeAngle = 0, int stepDelayMs = 80);
  void sweep(int minAngle, int maxAngle, int stepSize = 1, int stepDelay = 20);
  void continuousSweep(int minAngle, int maxAngle, int stepSize = 1, int stepDelay = 20);

  // Configuration
  void setPulseRange(int minPulse, int maxPulse);
  void setSpeed(int degreesPerSecond);

  // Utility functions
  static void enableDebug(bool enable = true);
  static void printChannelStatus();
};

// Static sweep helper class for non-blocking operations
class ServoSweep
{
private:
  ESP32Servo *_servo;
  int _minAngle;
  int _maxAngle;
  int _stepSize;
  int _stepDelay;
  int _currentAngle;
  int _direction;
  unsigned long _lastUpdate;
  bool _isActive;
  bool _continuous;

public:
  ServoSweep();
  void begin(ESP32Servo *servo, int minAngle, int maxAngle, int stepSize = 1, int stepDelay = 20);
  void start(bool continuous = false);
  void stop();
  void update(); // Call this in loop() for non-blocking operation
  bool isActive();
  int getCurrentAngle();
};

// Convenience functions for quick servo operations
namespace ServoUtils
{
  // Quick servo test - sweeps servo once
  void testServo(int pin, int minAngle = 0, int maxAngle = 180);

  // Multiple servo sweep
  void sweepMultiple(ESP32Servo servos[], int count, int minAngle = 0, int maxAngle = 180);

  // Servo calibration helper
  void calibrate(int pin);
}

// PWM Servo Control Configuration
extern const int SERVO_PIN;      // GPIO pin for servo signal
extern const int PWM_FREQ;       // Standard servo frequency (50Hz)
extern const int PWM_CHANNEL;    // PWM channel (ESP32 has 8 channels for C6)
extern const int PWM_RESOLUTION; // PWM resolution in bits (16-bit = 65536 levels)

// Servo timing constants (in microseconds) - More conservative range
extern const int SERVO_MIN_PULSE; // Minimum pulse width (0 degrees) - safer range
extern const int SERVO_MAX_PULSE; // Maximum pulse width (180 degrees) - safer range
extern const int SERVO_PERIOD;    // PWM period (20ms for 50Hz)

// Sweep configuration - Slower and smoother
extern const int MIN_ANGLE;
extern const int MAX_ANGLE;
extern const int SWEEP_DELAY; // Increased delay for smoother movement
extern const int STEP_SIZE;   // Larger steps for less jitter

// Function declarations
uint32_t angleToDutyCycle(int angle);
void moveServo(int angle);
void initializeServo();
void servoSweep();

#endif // ESP32_SERVO_H
