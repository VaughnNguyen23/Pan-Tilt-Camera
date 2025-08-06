# ESP32Servo Library - Complete Professional Servo Control

[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://espressif.com/)
[![Framework](https://img.shields.io/badge/framework-Arduino-orange.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## Overview

A comprehensive, feature-rich servo control library specifically designed for ESP32-C6 and other ESP32 variants. This library provides professional-grade servo control with smooth movements, non-blocking operations, multi-servo support, and specialized functions for automation projects like cat feeders.

## Key Features

- **Complete ESP32-C6 Compatibility** - Optimized for latest ESP32 hardware
- **Multi-Servo Support** - Control up to 8 servos simultaneously
- **Non-Blocking Operations** - Background servo control with `ServoSweep` class
- **Cat Feeder Optimized** - Specialized slow movement functions
- **Debug & Monitoring** - Comprehensive debugging and channel status
- **High Performance** - Native LEDC PWM peripheral usage
- **Utility Functions** - Built-in calibration and testing tools

## What's Included

### Core ESP32Servo Class
- Basic servo control (attach, write, read, detach)
- Smooth movement functions (smoothWrite, slowWrite, moveSlowly)
- Cat feeder functions (openSlowly, closeSlowly)
- Sweep operations (sweep, continuousSweep)
- Advanced configuration (setPulseRange, setSpeed)

### ServoSweep Helper Class
- Non-blocking sweep operations
- Continuous and single sweep modes
- Real-time angle tracking
- Background processing

### ServoUtils Namespace
- Quick servo testing functions
- Multiple servo coordination
- Calibration helpers

### Global Utilities
- Pre-configured constants
- Global servo control functions
- Angle conversion utilities

## Quick Start

### Installation
1. Copy `servo.h` and `servo.cpp` to your `lib/servo/` directory
2. Include the library in your project:

```cpp
#include <servo.h>
```

### Basic Usage

```cpp
#include <Arduino.h>
#include <servo.h>

ESP32Servo myServo;

void setup() {
    Serial.begin(115200);
    myServo.attach(2);        // Attach to GPIO pin 2
    myServo.write(90);        // Move to center position
}

void loop() {
    myServo.openSlowly(180, 30);   // Open slowly
    delay(2000);
    myServo.closeSlowly(0, 50);    // Close gently
    delay(2000);
}
```

## Cat Feeder Example

Perfect for automated pet feeding systems:

```cpp
ESP32Servo feederServo;

void setup() {
    Serial.begin(115200);
    feederServo.attach(2);
    feederServo.write(0);     // Start closed
    Serial.println("Cat feeder ready!");
}

void feedCat() {
    Serial.println("Feeding time!");
    
    // Open feeder slowly (quiet operation)
    feederServo.openSlowly(150, 40);  // 150° in ~6 seconds
    
    // Keep open for feeding
    delay(5000);  // 5 seconds
    
    // Close gently (prevents spilling)
    feederServo.closeSlowly(0, 80);   // Close in ~12 seconds
    
    Serial.println("Feeding complete!");
}
```

## Non-Blocking Operations

For complex applications requiring multi-tasking:

```cpp
ESP32Servo servo1, servo2;
ServoSweep sweeper;

void setup() {
    servo1.attach(2);
    servo2.attach(3);
    
    // Setup non-blocking sweep on servo2
    sweeper.begin(&servo2, 0, 180, 2, 25);
    sweeper.start(true);  // Start continuous sweep
}

void loop() {
    // Non-blocking sweep update
    sweeper.update();
    
    // Other servo operations
    static unsigned long lastMove = 0;
    if (millis() - lastMove > 5000) {
        servo1.smoothWrite(random(0, 181), 20);
        lastMove = millis();
    }
    
    // Your other code here...
}
```

## Advanced Features

### Multiple Servo Coordination

```cpp
ESP32Servo servos[4];

void setup() {
    for (int i = 0; i < 4; i++) {
        servos[i].attach(i + 2);  // Pins 2, 3, 4, 5
    }
    
    // Sweep all servos together
    ServoUtils::sweepMultiple(servos, 4, 45, 135);
}
```

### Servo Calibration

```cpp
void setup() {
    // Test and calibrate servo on pin 2
    ServoUtils::calibrate(2);
    
    // Test servo with custom range
    ServoUtils::testServo(3, 30, 150);
}
```

### Debug and Monitoring

```cpp
void setup() {
    // Enable comprehensive debugging
    ESP32Servo::enableDebug(true);
    
    // Monitor PWM channel usage
    ESP32Servo::printChannelStatus();
}
```

## Complete API Reference

### ESP32Servo Class Methods

| Method | Description |
|--------|-------------|
| `attach(pin)` | Attach servo to GPIO pin |
| `attach(pin, minPulse, maxPulse)` | Attach with custom pulse range |
| `write(angle)` | Move to angle (0-180°) |
| `read()` | Get current position |
| `writeMicroseconds(us)` | Direct pulse width control |
| `smoothWrite(angle, stepDelay)` | Gradual movement |
| `slowWrite(angle, totalTime)` | Timed movement |
| `moveSlowly(start, end, stepDelay)` | Move between angles |
| `openSlowly(angle, stepDelay)` | Slow opening motion |
| `closeSlowly(angle, stepDelay)` | Gentle closing motion |
| `sweep(min, max, step, delay)` | Single sweep |
| `continuousSweep(min, max, step, delay)` | Infinite sweep |
| `setPulseRange(min, max)` | Configure pulse width |
| `setSpeed(degreesPerSecond)` | Set movement speed |
| `detach()` | Disconnect servo |
| `attached()` | Check if connected |

### ServoSweep Class Methods

| Method | Description |
|--------|-------------|
| `begin(servo, min, max, step, delay)` | Initialize sweep |
| `start(continuous)` | Start sweep operation |
| `stop()` | Stop sweep |
| `update()` | Update sweep (call in loop) |
| `isActive()` | Check if running |
| `getCurrentAngle()` | Get current position |

### ServoUtils Functions

| Function | Description |
|----------|-------------|
| `testServo(pin, min, max)` | Quick servo test |
| `sweepMultiple(servos[], count, min, max)` | Multi-servo sweep |
| `calibrate(pin)` | Calibration helper |

## Configuration

### Speed Reference Guide

| Step Delay | 0° to 180° Time | Use Case |
|------------|-----------------|----------|
| 10ms | 1.8 seconds | Very fast movement |
| 20ms | 3.6 seconds | Quick operations |
| 30ms | 5.4 seconds | Standard opening |
| 50ms | 9.0 seconds | Moderate speed |
| 80ms | 14.4 seconds | Gentle closing |
| 100ms | 18.0 seconds | Very gentle |
| 200ms | 36.0 seconds | Ultra-quiet |

### Hardware Configuration

```cpp
// Default configuration (can be customized)
const int PWM_FREQ = 50;          // 50Hz standard
const int PWM_RESOLUTION = 16;    // 16-bit resolution
const int SERVO_MIN_PULSE = 1000; // 1ms min pulse
const int SERVO_MAX_PULSE = 2000; // 2ms max pulse
```

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Servo doesn't move | Check power supply and pin connection |
| Jittery movement | Increase step delay or check power |
| Channel allocation error | Use `printChannelStatus()` to debug |
| Compilation error | Ensure both .h and .cpp files are included |

### Debug Commands

```cpp
ESP32Servo::enableDebug(true);        // Enable debug output
ESP32Servo::printChannelStatus();     // Show channel usage
servo.attached();                     // Check if servo is connected
```

## Performance Specifications

- **Maximum Servos**: 8 simultaneous
- **PWM Frequency**: 50Hz (standard servo)
- **Resolution**: 16-bit (65536 levels)
- **Angle Range**: 0-180 degrees
- **Pulse Width Range**: 500-2500μs (configurable)
- **Memory Usage**: ~2KB flash, minimal RAM

## Hardware Support

- ESP32-C6 (Primary target)
- ESP32-S3
- ESP32-S2
- ESP32 (Classic)
- ESP32-C3

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Support

For support and questions:
- Check the documentation and examples
- Review the troubleshooting section
- Open an issue for bugs or feature requests

---

**Made with care for the ESP32 community and automated pet care enthusiasts!**
