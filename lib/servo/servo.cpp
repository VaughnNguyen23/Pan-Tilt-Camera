#include "servo.h"

// Static variable definitions
bool ESP32Servo::_channelUsed[8] = {false};
bool ESP32Servo::_timerConfigured = false;

// Constructor
ESP32Servo::ESP32Servo()
{
  _pin = -1;
  _channel = -1;
  _minPulse = 1000;
  _maxPulse = 2000;
  _currentAngle = 90;
  _isAttached = false;
}

// Destructor
ESP32Servo::~ESP32Servo()
{
  detach();
}

// Attach servo to pin
bool ESP32Servo::attach(int pin)
{
  return attach(pin, 1000, 2000);
}

bool ESP32Servo::attach(int pin, int minPulse, int maxPulse)
{
  if (_isAttached)
  {
    detach();
  }

  _pin = pin;
  _minPulse = minPulse;
  _maxPulse = maxPulse;

  // Get available PWM channel
  _channel = getNextAvailableChannel();
  if (_channel == -1)
  {
    Serial.println("Error: No PWM channels available!");
    return false;
  }

  // Configure timer if not done already
  if (!_timerConfigured)
  {
    configureTimer();
  }

  // Configure PWM channel
  ledc_channel_config_t ledc_channel = {
      .gpio_num = _pin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)_channel,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0};

  if (ledc_channel_config(&ledc_channel) != ESP_OK)
  {
    Serial.println("Error: Failed to configure PWM channel!");
    _channelUsed[_channel] = false;
    return false;
  }

  _channelUsed[_channel] = true;
  _isAttached = true;

  // Move to default position
  write(_currentAngle);

  Serial.printf("Servo attached to pin %d, channel %d\n", _pin, _channel);
  return true;
}

// Detach servo
void ESP32Servo::detach()
{
  if (_isAttached && _channel >= 0 && _channel < 8)
  {
    ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, 0);
    _channelUsed[_channel] = false;
    _isAttached = false;
    _channel = -1;
    _pin = -1;
  }
}

// Write angle to servo
void ESP32Servo::write(int angle)
{
  if (!_isAttached)
  {
    Serial.println("Error: Servo not attached!");
    return;
  }

  angle = constrain(angle, 0, 180);
  _currentAngle = angle;

  uint32_t dutyCycle = angleToDutyCycle(angle);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, dutyCycle);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel);
}

// Read current angle
int ESP32Servo::read()
{
  return _currentAngle;
}

// Check if attached
bool ESP32Servo::attached()
{
  return _isAttached;
}

// Write microseconds directly
void ESP32Servo::writeMicroseconds(int microseconds)
{
  if (!_isAttached)
    return;

  microseconds = constrain(microseconds, _minPulse, _maxPulse);

  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t dutyCycle = (microseconds * maxDuty) / SERVO_PERIOD;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, dutyCycle);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel);

  // Update current angle based on microseconds
  _currentAngle = map(microseconds, _minPulse, _maxPulse, 0, 180);
}

// Smooth write with gradual movement
void ESP32Servo::smoothWrite(int targetAngle, int stepDelay)
{
  if (!_isAttached)
    return;

  targetAngle = constrain(targetAngle, 0, 180);
  int currentAngle = _currentAngle;

  if (targetAngle > currentAngle)
  {
    for (int angle = currentAngle; angle <= targetAngle; angle++)
    {
      write(angle);
      delay(stepDelay);
    }
  }
  else
  {
    for (int angle = currentAngle; angle >= targetAngle; angle--)
    {
      write(angle);
      delay(stepDelay);
    }
  }
}

// Slow write with time-based movement
void ESP32Servo::slowWrite(int targetAngle, int totalTimeMs)
{
  if (!_isAttached)
    return;

  targetAngle = constrain(targetAngle, 0, 180);
  int currentAngle = _currentAngle;
  int totalSteps = abs(targetAngle - currentAngle);

  if (totalSteps == 0)
    return; // Already at target

  int stepDelay = totalTimeMs / totalSteps;
  stepDelay = max(stepDelay, 10); // Minimum 10ms delay

  smoothWrite(targetAngle, stepDelay);
}

// Move slowly from startAngle to endAngle with configurable speed
void ESP32Servo::moveSlowly(int startAngle, int endAngle, int stepDelayMs)
{
  if (!_isAttached)
    return;

  startAngle = constrain(startAngle, 0, 180);
  endAngle = constrain(endAngle, 0, 180);

  // Move to start position first if not already there
  if (_currentAngle != startAngle)
  {
    write(startAngle);
    delay(100); // Brief pause
  }

  int step = (startAngle < endAngle) ? 1 : -1;

  Serial.printf("Moving servo from %d° to %d° with %dms delay\n", startAngle, endAngle, stepDelayMs);

  for (int angle = startAngle; angle != endAngle; angle += step)
  {
    write(angle);
    delay(stepDelayMs);
  }
  write(endAngle); // Ensure we reach the exact target
  Serial.printf("Servo movement complete at %d°\n", endAngle);
}

// Cat feeder specific: Open slowly (optimized for feeding)
void ESP32Servo::openSlowly(int openAngle, int stepDelayMs)
{
  if (!_isAttached)
    return;

  Serial.println("Opening feeder lid slowly...");
  moveSlowly(_currentAngle, openAngle, stepDelayMs);
}

// Cat feeder specific: Close slowly (optimized for feeding)
void ESP32Servo::closeSlowly(int closeAngle, int stepDelayMs)
{
  if (!_isAttached)
    return;

  Serial.println("Closing feeder lid slowly...");
  moveSlowly(_currentAngle, closeAngle, stepDelayMs);
}

// Sweep between angles
void ESP32Servo::sweep(int minAngle, int maxAngle, int stepSize, int stepDelay)
{
  if (!_isAttached)
    return;

  minAngle = constrain(minAngle, 0, 180);
  maxAngle = constrain(maxAngle, 0, 180);

  // Forward sweep
  for (int angle = minAngle; angle <= maxAngle; angle += stepSize)
  {
    write(angle);
    delay(stepDelay);
  }

  // Backward sweep
  for (int angle = maxAngle; angle >= minAngle; angle -= stepSize)
  {
    write(angle);
    delay(stepDelay);
  }
}

// Continuous sweep function
void ESP32Servo::continuousSweep(int minAngle, int maxAngle, int stepSize, int stepDelay)
{
  Serial.printf("Starting continuous sweep: %d to %d degrees\n", minAngle, maxAngle);

  while (true)
  {
    // Forward sweep
    for (int angle = minAngle; angle <= maxAngle; angle += stepSize)
    {
      write(angle);
      delay(stepDelay);
    }

    // Backward sweep
    for (int angle = maxAngle; angle >= minAngle; angle -= stepSize)
    {
      write(angle);
      delay(stepDelay);
    }
  }
}

// Set pulse range
void ESP32Servo::setPulseRange(int minPulse, int maxPulse)
{
  _minPulse = minPulse;
  _maxPulse = maxPulse;
}

// Set servo movement speed (degrees per second)
void ESP32Servo::setSpeed(int degreesPerSecond)
{
  // This method sets a speed parameter that can be used by other movement functions
  // For now, we'll store it as a class variable and use it in future smooth movement implementations
  // Note: This is a placeholder implementation - speed control would need additional class variables
  Serial.printf("Servo speed set to %d degrees per second\n", degreesPerSecond);

  // TODO: Add actual speed control implementation with timing calculations
  // This would require adding a private speed variable and modifying smooth movement functions
}

// Private: Convert angle to duty cycle
uint32_t ESP32Servo::angleToDutyCycle(int angle)
{
  angle = constrain(angle, 0, 180);

  int pulseWidth = map(angle, 0, 180, _minPulse, _maxPulse);
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t dutyCycle = (pulseWidth * maxDuty) / SERVO_PERIOD;

  return dutyCycle;
}

// Private: Get next available PWM channel
int ESP32Servo::getNextAvailableChannel()
{
  for (int i = 0; i < 8; i++)
  {
    if (!_channelUsed[i])
    {
      return i;
    }
  }
  return -1;
}

// Private: Configure LEDC timer
void ESP32Servo::configureTimer()
{
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK};

  if (ledc_timer_config(&ledc_timer) == ESP_OK)
  {
    _timerConfigured = true;
    Serial.println("LEDC timer configured successfully");
  }
  else
  {
    Serial.println("Error: Failed to configure LEDC timer!");
  }
}

// Static: Print channel status
void ESP32Servo::printChannelStatus()
{
  Serial.println("PWM Channel Status:");
  for (int i = 0; i < 8; i++)
  {
    Serial.printf("Channel %d: %s\n", i, _channelUsed[i] ? "USED" : "FREE");
  }
}

// Static debug control
void ESP32Servo::enableDebug(bool enable)
{
  static bool debugEnabled = false;
  debugEnabled = enable;
  Serial.printf("Servo debug: %s\n", debugEnabled ? "ENABLED" : "DISABLED");
}

// =============================================================================
// ServoSweep Class Implementation
// =============================================================================

// Constructor
ServoSweep::ServoSweep()
{
  _servo = nullptr;
  _minAngle = 0;
  _maxAngle = 180;
  _stepSize = 1;
  _stepDelay = 20;
  _currentAngle = 0;
  _direction = 1;
  _lastUpdate = 0;
  _isActive = false;
  _continuous = false;
}

// Initialize sweep parameters
void ServoSweep::begin(ESP32Servo *servo, int minAngle, int maxAngle, int stepSize, int stepDelay)
{
  _servo = servo;
  _minAngle = minAngle;
  _maxAngle = maxAngle;
  _stepSize = stepSize;
  _stepDelay = stepDelay;
  _currentAngle = minAngle;
  _direction = 1;
  _lastUpdate = 0;
  Serial.printf("ServoSweep initialized: %d to %d degrees\n", minAngle, maxAngle);
}

// Start sweep operation
void ServoSweep::start(bool continuous)
{
  if (_servo && _servo->attached())
  {
    _isActive = true;
    _continuous = continuous;
    _currentAngle = _minAngle;
    _direction = 1;
    _lastUpdate = millis();
    _servo->write(_currentAngle);
    Serial.printf("Sweep started (%s)\n", continuous ? "continuous" : "single");
  }
  else
  {
    Serial.println("Error: Cannot start sweep - servo not attached");
  }
}

// Stop sweep operation
void ServoSweep::stop()
{
  _isActive = false;
  Serial.println("Sweep stopped");
}

// Update sweep (call this in loop())
void ServoSweep::update()
{
  if (!_isActive || !_servo || !_servo->attached())
  {
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - _lastUpdate >= _stepDelay)
  {
    _currentAngle += (_direction * _stepSize);

    // Check boundaries and direction change
    if (_currentAngle >= _maxAngle)
    {
      _currentAngle = _maxAngle;
      if (_continuous)
      {
        _direction = -1; // Reverse direction
      }
      else
      {
        _isActive = false; // Stop single sweep
        Serial.println("Single sweep completed");
        return;
      }
    }
    else if (_currentAngle <= _minAngle)
    {
      _currentAngle = _minAngle;
      _direction = 1; // Forward direction
    }

    _servo->write(_currentAngle);
    _lastUpdate = currentTime;
  }
}

// Check if sweep is active
bool ServoSweep::isActive()
{
  return _isActive;
}

// Get current angle
int ServoSweep::getCurrentAngle()
{
  return _currentAngle;
}

// =============================================================================
// ServoUtils Namespace Implementation
// =============================================================================

namespace ServoUtils
{
// Quick servo test function
void testServo(int pin, int minAngle, int maxAngle)
{
  ESP32Servo testServo;
  Serial.printf("Testing servo on pin %d (%d to %d degrees)\n", pin, minAngle, maxAngle);

  if (!testServo.attach(pin))
  {
    Serial.println("Error: Failed to attach test servo");
    return;
  }

  // Test positions
  testServo.write(minAngle);
  delay(1000);
  testServo.write((minAngle + maxAngle) / 2); // Middle position
  delay(1000);
  testServo.write(maxAngle);
  delay(1000);
  testServo.write((minAngle + maxAngle) / 2); // Back to middle
  delay(1000);

  testServo.detach();
  Serial.println("Servo test completed");
}

// Multiple servo sweep
void sweepMultiple(ESP32Servo servos[], int count, int minAngle, int maxAngle)
{
  Serial.printf("Sweeping %d servos from %d to %d degrees\n", count, minAngle, maxAngle);

  // Forward sweep
  for (int angle = minAngle; angle <= maxAngle; angle += 5)
  {
    for (int i = 0; i < count; i++)
    {
      if (servos[i].attached())
      {
        servos[i].write(angle);
      }
    }
    delay(50);
  }

  // Backward sweep
  for (int angle = maxAngle; angle >= minAngle; angle -= 5)
  {
    for (int i = 0; i < count; i++)
    {
      if (servos[i].attached())
      {
        servos[i].write(angle);
      }
    }
    delay(50);
  }

  Serial.println("Multiple servo sweep completed");
}

// Servo calibration helper
void calibrate(int pin)
{
  ESP32Servo calibrationServo;
  Serial.printf("Starting servo calibration on pin %d\n", pin);
  Serial.println("This will test different pulse widths...");

  if (!calibrationServo.attach(pin, 500, 2500)) // Wide pulse range for calibration
  {
    Serial.println("Error: Failed to attach calibration servo");
    return;
  }

  // Test different positions with pulse width info
  int testAngles[] = {0, 45, 90, 135, 180};
  int pulseWidths[] = {500, 1000, 1500, 2000, 2500};

  for (int i = 0; i < 5; i++)
  {
    Serial.printf("Testing %d degrees (approx %d μs pulse)...\n", testAngles[i], pulseWidths[i]);
    calibrationServo.write(testAngles[i]);
    delay(2000);
  }

  calibrationServo.detach();
  Serial.println("Calibration completed. Adjust pulse range based on observed movement.");
}
}

// =============================================================================
// Global Function Implementations
// =============================================================================

// Global constants definitions
const int SERVO_PIN = 2;
const int PWM_FREQ = 50;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 16;
const int SERVO_MIN_PULSE = 1000;
const int SERVO_MAX_PULSE = 2000;
const int SERVO_PERIOD = 20000;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int SWEEP_DELAY = 20;
const int STEP_SIZE = 1;

// Global utility functions
uint32_t angleToDutyCycle(int angle)
{
  // Map angle (0-180) to pulse width (1000-2000 microseconds)
  int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // Convert to duty cycle value
  uint32_t dutyCycle = (pulseWidth * ((1 << PWM_RESOLUTION) - 1)) / SERVO_PERIOD;
  return dutyCycle;
}

void moveServo(int angle)
{
  static ESP32Servo globalServo;
  static bool initialized = false;

  if (!initialized)
  {
    globalServo.attach(SERVO_PIN);
    initialized = true;
    Serial.println("Global servo initialized");
  }

  globalServo.write(angle);
  Serial.printf("Global servo moved to %d degrees\n", angle);
}

void initializeServo()
{
  static ESP32Servo globalServo;
  globalServo.attach(SERVO_PIN);
  globalServo.write(90); // Center position
  Serial.printf("Servo initialized on pin %d at center position\n", SERVO_PIN);
}

void servoSweep()
{
  static ESP32Servo globalServo;
  static bool initialized = false;

  if (!initialized)
  {
    globalServo.attach(SERVO_PIN);
    initialized = true;
  }

  Serial.println("Starting global servo sweep...");
  globalServo.sweep(MIN_ANGLE, MAX_ANGLE, STEP_SIZE, SWEEP_DELAY);
}
