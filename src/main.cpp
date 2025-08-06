#include <Arduino.h>
#include <servo.h>

// Pin definitions for pan and tilt servos
#define PAN_SERVO_PIN 2   // Horizontal movement (X-axis)
#define TILT_SERVO_PIN 1  // Vertical movement (Y-axis)

// Movement parameters
#define PAN_MIN_ANGLE 0
#define PAN_MAX_ANGLE 180
#define TILT_MIN_ANGLE 0
#define TILT_MAX_ANGLE 180
#define CENTER_POSITION 90
#define MOVE_DELAY 20      // Delay between movements for smooth operation

class CameraDirectionManager {
private:
  ESP32Servo panServo;   // Horizontal movement servo
  ESP32Servo tiltServo;  // Vertical movement servo
  int currentPanAngle;
  int currentTiltAngle;
  
public:
  CameraDirectionManager() : currentPanAngle(CENTER_POSITION), currentTiltAngle(CENTER_POSITION) {}
  
  void initialize() {
    Serial.println("Initializing Pan-Tilt Camera System...");
    
    // Print channel status before attaching
    ESP32Servo::printChannelStatus();
    
    // Attach servos to their pins with a small delay between attachments
    Serial.println("Attaching pan servo...");
    if (!panServo.attach(PAN_SERVO_PIN)) {
      Serial.println("Error: Failed to attach pan servo!");
      return;
    }
    delay(100); // Small delay between servo attachments
    
    Serial.println("Attaching tilt servo...");
    if (!tiltServo.attach(TILT_SERVO_PIN)) {
      Serial.println("Error: Failed to attach tilt servo!");
      return;
    }
    delay(100);
    
    // Print channel status after attaching
    ESP32Servo::printChannelStatus();
    
    // Test each servo individually first
    Serial.println("Testing servos individually...");
    testIndividualServos();
    
    // Move to center position
    moveToCenter();
    
    Serial.println("Pan-Tilt Camera System initialized successfully!");
    Serial.printf("Pan servo on pin %d, Tilt servo on pin %d\n", PAN_SERVO_PIN, TILT_SERVO_PIN);
  }
  
  void testIndividualServos() {
    Serial.println("Testing pan servo...");
    panServo.write(45);
    delay(1000);
    panServo.write(135);
    delay(1000);
    panServo.write(90);
    delay(500);
    
    Serial.println("Testing tilt servo...");
    tiltServo.write(45);
    delay(1000);
    tiltServo.write(135);
    delay(1000);
    tiltServo.write(90);
    delay(500);
    
    Serial.println("Individual servo tests completed.");
  }
  
  void moveToCenter() {
    Serial.println("Moving camera to center position...");
    panServo.smoothWrite(CENTER_POSITION, MOVE_DELAY);
    tiltServo.smoothWrite(CENTER_POSITION, MOVE_DELAY);
    currentPanAngle = CENTER_POSITION;
    currentTiltAngle = CENTER_POSITION;
    delay(500);
  }
  
  void panLeft(int degrees = 30) {
    int targetAngle = constrain(currentPanAngle - degrees, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    Serial.printf("Panning left to %d degrees\n", targetAngle);
    panServo.smoothWrite(targetAngle, MOVE_DELAY);
    currentPanAngle = targetAngle;
  }
  
  void panRight(int degrees = 30) {
    int targetAngle = constrain(currentPanAngle + degrees, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    Serial.printf("Panning right to %d degrees\n", targetAngle);
    panServo.smoothWrite(targetAngle, MOVE_DELAY);
    currentPanAngle = targetAngle;
  }
  
  void tiltUp(int degrees = 30) {
    int targetAngle = constrain(currentTiltAngle + degrees, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    Serial.printf("Tilting up to %d degrees\n", targetAngle);
    tiltServo.smoothWrite(targetAngle, MOVE_DELAY);
    currentTiltAngle = targetAngle;
  }
  
  void tiltDown(int degrees = 30) {
    int targetAngle = constrain(currentTiltAngle - degrees, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    Serial.printf("Tilting down to %d degrees\n", targetAngle);
    tiltServo.smoothWrite(targetAngle, MOVE_DELAY);
    currentTiltAngle = targetAngle;
  }
  
  void panToAngle(int angle) {
    angle = constrain(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    Serial.printf("Panning to absolute angle: %d degrees\n", angle);
    panServo.smoothWrite(angle, MOVE_DELAY);
    currentPanAngle = angle;
  }
  
  void tiltToAngle(int angle) {
    angle = constrain(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    Serial.printf("Tilting to absolute angle: %d degrees\n", angle);
    tiltServo.smoothWrite(angle, MOVE_DELAY);
    currentTiltAngle = angle;
  }
  
  void moveToPosition(int panAngle, int tiltAngle) {
    panAngle = constrain(panAngle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    tiltAngle = constrain(tiltAngle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    
    Serial.printf("Moving to position: Pan=%d°, Tilt=%d°\n", panAngle, tiltAngle);
    
    // Option 1: Sequential movement (more reliable)
    Serial.println("Moving sequentially for reliability...");
    panServo.smoothWrite(panAngle, MOVE_DELAY);
    delay(100); // Small delay between servo commands
    tiltServo.smoothWrite(tiltAngle, MOVE_DELAY);
    
    currentPanAngle = panAngle;
    currentTiltAngle = tiltAngle;
    
    delay(1000); // Allow time for movement to complete
  }
  
  void moveToPositionSimultaneous(int panAngle, int tiltAngle) {
    panAngle = constrain(panAngle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    tiltAngle = constrain(tiltAngle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    
    Serial.printf("Moving simultaneously to position: Pan=%d°, Tilt=%d°\n", panAngle, tiltAngle);
    
    // Option 2: Simultaneous movement (faster but potentially problematic)
    panServo.write(panAngle);
    delayMicroseconds(100); // Very small delay between commands
    tiltServo.write(tiltAngle);
    
    currentPanAngle = panAngle;
    currentTiltAngle = tiltAngle;
    
    delay(1000); // Allow time for movement to complete
  }
  
  void performScan() {
    Serial.println("Performing camera scan pattern...");
    
    // Scan pattern: left to right, then up, repeat
    int scanPositions[][2] = {
      {45, 60},   // Left-up
      {90, 60},   // Center-up
      {135, 60},  // Right-up
      {135, 90},  // Right-center
      {90, 90},   // Center-center
      {45, 90},   // Left-center
      {45, 120},  // Left-down
      {90, 120},  // Center-down
      {135, 120}  // Right-down
    };
    
    for (int i = 0; i < 9; i++) {
      moveToPosition(scanPositions[i][0], scanPositions[i][1]);
      delay(1000); // Pause at each position
    }
    
    // Return to center
    moveToCenter();
    Serial.println("Scan pattern completed!");
  }
  
  void performSweep() {
    Serial.println("Performing horizontal sweep...");
    
    // Horizontal sweep at current tilt position
    for (int angle = PAN_MIN_ANGLE; angle <= PAN_MAX_ANGLE; angle += 5) {
      panServo.write(angle);
      currentPanAngle = angle;
      delay(100);
    }
    
    // Sweep back
    for (int angle = PAN_MAX_ANGLE; angle >= PAN_MIN_ANGLE; angle -= 5) {
      panServo.write(angle);
      currentPanAngle = angle;
      delay(100);
    }
    
    moveToCenter();
    Serial.println("Sweep completed!");
  }
  
  void getCurrentPosition() {
    Serial.printf("Current position - Pan: %d°, Tilt: %d°\n", currentPanAngle, currentTiltAngle);
    Serial.printf("Pan servo attached: %s\n", panServo.attached() ? "YES" : "NO");
    Serial.printf("Tilt servo attached: %s\n", tiltServo.attached() ? "YES" : "NO");
  }
  
  void diagnosticTest() {
    Serial.println("=== DIAGNOSTIC TEST ===");
    
    // Check servo attachment status
    Serial.println("Checking servo attachment...");
    Serial.printf("Pan servo (pin %d): %s\n", PAN_SERVO_PIN, panServo.attached() ? "ATTACHED" : "NOT ATTACHED");
    Serial.printf("Tilt servo (pin %d): %s\n", TILT_SERVO_PIN, tiltServo.attached() ? "ATTACHED" : "NOT ATTACHED");
    
    // Print PWM channel status
    ESP32Servo::printChannelStatus();
    
    // Test each servo individually with different angles
    Serial.println("\nTesting pan servo only...");
    tiltServo.detach(); // Temporarily detach tilt servo
    delay(100);
    
    panServo.write(0);
    delay(1000);
    panServo.write(90);
    delay(1000);
    panServo.write(180);
    delay(1000);
    panServo.write(90);
    delay(500);
    
    // Re-attach tilt servo
    Serial.println("Re-attaching tilt servo...");
    tiltServo.attach(TILT_SERVO_PIN);
    delay(100);
    
    Serial.println("Testing tilt servo only...");
    panServo.detach(); // Temporarily detach pan servo
    delay(100);
    
    tiltServo.write(0);
    delay(1000);
    tiltServo.write(90);
    delay(1000);
    tiltServo.write(180);
    delay(1000);
    tiltServo.write(90);
    delay(500);
    
    // Re-attach pan servo
    Serial.println("Re-attaching pan servo...");
    panServo.attach(PAN_SERVO_PIN);
    delay(100);
    
    Serial.println("Testing both servos with sequential commands...");
    panServo.write(45);
    delay(500);
    tiltServo.write(45);
    delay(1000);
    
    panServo.write(135);
    delay(500);
    tiltServo.write(135);
    delay(1000);
    
    panServo.write(90);
    delay(500);
    tiltServo.write(90);
    delay(1000);
    
    Serial.println("=== DIAGNOSTIC TEST COMPLETE ===");
  }
  
  void detachServos() {
    panServo.detach();
    tiltServo.detach();
    Serial.println("Servos detached");
  }
  
  void reinitialize() {
    Serial.println("Reinitializing servos...");
    detachServos();
    delay(200);
    
    // Reset current position tracking
    currentPanAngle = CENTER_POSITION;
    currentTiltAngle = CENTER_POSITION;
    
    // Re-attach with fresh PWM channels
    if (!panServo.attach(PAN_SERVO_PIN)) {
      Serial.println("Error: Failed to re-attach pan servo!");
      return;
    }
    delay(100);
    
    if (!tiltServo.attach(TILT_SERVO_PIN)) {
      Serial.println("Error: Failed to re-attach tilt servo!");
      return;
    }
    delay(100);
    
    // Move to center
    moveToCenter();
    Serial.println("Servos reinitialized successfully!");
  }
};

// Global camera manager instance
CameraDirectionManager camera;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("==== Pan-Tilt Camera Control System ====");
  Serial.println("ESP32-C6 Servo Control Demo");
  
  // Initialize the camera system
  camera.initialize();
  
  // Print available commands
  Serial.println("\nAvailable Commands:");
  Serial.println("1 - Pan Left");
  Serial.println("2 - Pan Right");
  Serial.println("3 - Tilt Up");
  Serial.println("4 - Tilt Down");
  Serial.println("5 - Move to Center");
  Serial.println("6 - Perform Scan Pattern");
  Serial.println("7 - Perform Horizontal Sweep");
  Serial.println("8 - Show Current Position");
  Serial.println("9 - Demo Sequence");
  Serial.println("0 - Detach Servos");
  Serial.println("d - Diagnostic Test");
  Serial.println("s - Test Simultaneous Movement");
  Serial.println("r - Reinitialize Servos");
  Serial.println("\nSystem ready! Enter command number:");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char command = Serial.read();
    
    switch (command) {
      case '1':
        camera.panLeft();
        break;
        
      case '2':
        camera.panRight();
        break;
        
      case '3':
        camera.tiltUp();
        break;
        
      case '4':
        camera.tiltDown();
        break;
        
      case '5':
        camera.moveToCenter();
        break;
        
      case '6':
        camera.performScan();
        break;
        
      case '7':
        camera.performSweep();
        break;
        
      case '8':
        camera.getCurrentPosition();
        break;
        
      case '9':
        // Demo sequence
        Serial.println("Starting demo sequence...");
        camera.moveToCenter();
        delay(1000);
        camera.panLeft(45);
        delay(1000);
        camera.tiltUp(30);
        delay(1000);
        camera.panRight(90);
        delay(1000);
        camera.tiltDown(60);
        delay(1000);
        camera.moveToCenter();
        Serial.println("Demo sequence completed!");
        break;
        
      case '0':
        camera.detachServos();
        break;
        
      case 'd':
      case 'D':
        camera.diagnosticTest();
        break;
        
      case 's':
      case 'S':
        Serial.println("Testing simultaneous movement...");
        camera.moveToPositionSimultaneous(45, 135);
        delay(2000);
        camera.moveToPositionSimultaneous(135, 45);
        delay(2000);
        camera.moveToCenter();
        break;
        
      case 'r':
      case 'R':
        camera.reinitialize();
        break;
        
      default:
        Serial.println("Invalid command. Please enter 0-9, d, s, or r.");
        break;
    }
    
    // Clear any remaining characters in the serial buffer
    while (Serial.available()) {
      Serial.read();
    }
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}
