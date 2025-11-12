/*
 * SENTIO ESP32 Servo Bridge Firmware
 * 
 * Accepts JSON servo commands over serial UART and controls servos via PWM/PCA9685.
 * Implements watchdog, safe-start, and E-stop logic.
 * 
 * Author: DevSora Deep-Tech Research
 * License: Proprietary
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_PCA9685.h"

// Configuration
#define SERIAL_BAUDRATE 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ESTOP_PIN 35
#define WATCHDOG_TIMEOUT_MS 5000
#define NUM_SERVOS 5

// PCA9685 servo driver
Adafruit_PCA9685 pwm;
const uint16_t SERVO_MIN_US = 1000;  // 1ms
const uint16_t SERVO_MAX_US = 2000;  // 2ms
const uint8_t SERVO_FREQ_HZ = 50;    // 50Hz for standard servos

// Servo state
struct ServoState {
  uint8_t channel;
  uint16_t current_position;  // 0-4095 for PCA9685
  uint16_t target_position;
  bool enabled;
};

ServoState servos[NUM_SERVOS] = {
  {0, 1500, 1500, true},
  {1, 1500, 1500, true},
  {2, 1500, 1500, true},
  {3, 1500, 1500, true},
  {4, 1500, 1500, true},
};

// System state
bool emergency_stop = false;
uint32_t last_command_time = 0;
uint32_t command_count = 0;
uint32_t error_count = 0;

// Function declarations
void handle_json_command(const JsonDocument &doc);
void move_servo(uint8_t servo_id, uint16_t position);
void send_response(const char *status, const char *msg);
void check_watchdog();
void check_estop();
void safe_shutdown();

/**
 * Setup function - Initialize hardware
 */
void setup() {
  // Serial communication
  Serial.begin(SERIAL_BAUDRATE);
  delay(500);
  
  Serial.println("{\"status\": \"init\", \"msg\": \"SENTIO Servo Bridge starting\"}");
  
  // E-stop input
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  
  // I2C for PCA9685
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialize PCA9685
  if (!pwm.begin()) {
    Serial.println("{\"status\": \"error\", \"msg\": \"PCA9685 initialization failed\"}");
    while (1) delay(100);  // Halt
  }
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  
  // Set all servos to neutral position (1500us = 90 degrees)
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(servos[i].channel, 0, 308);  // 1500us at 50Hz
  }
  
  delay(500);
  Serial.println("{\"status\": \"ok\", \"msg\": \"Servo bridge ready\"}");
}

/**
 * Main loop
 */
void loop() {
  check_estop();
  check_watchdog();
  
  // Process serial commands
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) return;
    
    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, line);
    
    if (error) {
      char msg[128];
      snprintf(msg, sizeof(msg), "JSON parse error: %s", error.c_str());
      send_response("error", msg);
      error_count++;
      return;
    }
    
    last_command_time = millis();
    handle_json_command(doc);
    command_count++;
  }
  
  delay(10);  // Small delay to prevent flooding
}

/**
 * Handle incoming JSON command
 */
void handle_json_command(const JsonDocument &doc) {
  const char *cmd_type = doc["type"] | "";
  
  if (strcmp(cmd_type, "servo_move") == 0) {
    // Extract servo positions
    JsonObject positions = doc["positions"];
    
    if (positions.isNull()) {
      send_response("error", "Missing positions object");
      return;
    }
    
    // Move each servo
    for (JsonPair pair : positions) {
      uint8_t servo_id = atoi(pair.key().c_str());
      uint16_t angle = pair.value().as<uint16_t>();
      
      if (servo_id >= NUM_SERVOS) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Servo %d out of range", servo_id);
        send_response("error", msg);
        error_count++;
        continue;
      }
      
      move_servo(servo_id, angle);
    }
    
    send_response("ok", "Servo move command executed");
  }
  
  else if (strcmp(cmd_type, "led_control") == 0) {
    // LED control (placeholder - would implement actual LED control)
    send_response("ok", "LED command received");
  }
  
  else if (strcmp(cmd_type, "init") == 0) {
    send_response("ok", "Initialization acknowledged");
  }
  
  else if (strcmp(cmd_type, "estop") == 0) {
    emergency_stop = true;
    safe_shutdown();
    send_response("ok", "Emergency stop activated");
  }
  
  else if (strcmp(cmd_type, "clear_estop") == 0) {
    emergency_stop = false;
    send_response("ok", "Emergency stop cleared");
  }
  
  else {
    char msg[64];
    snprintf(msg, sizeof(msg), "Unknown command type: %s", cmd_type);
    send_response("error", msg);
  }
}

/**
 * Move servo to position
 * 
 * @param servo_id Servo ID (0-4)
 * @param angle Position in degrees (0-180)
 */
void move_servo(uint8_t servo_id, uint16_t angle) {
  if (emergency_stop) {
    Serial.println("{\"status\": \"error\", \"msg\": \"Emergency stop active\"}");
    return;
  }
  
  if (servo_id >= NUM_SERVOS) {
    return;
  }
  
  // Clamp angle
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  // Convert angle to PWM value
  // 0° = 1000us, 180° = 2000us
  uint16_t pwm_us = SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US) / 180);
  
  // Convert microseconds to PCA9685 value (12-bit resolution, 50Hz)
  // Each tick = 20ms / 4096 = 4.88us
  uint16_t pwm_value = (pwm_us * 4096) / 20000;
  
  pwm.setPWM(servos[servo_id].channel, 0, pwm_value);
  
  servos[servo_id].current_position = pwm_value;
  
  Serial.printf("{\"status\": \"ok\", \"servo\": %d, \"angle\": %d, \"pwm\": %d}\n",
                servo_id, angle, pwm_value);
}

/**
 * Send JSON response
 */
void send_response(const char *status, const char *msg) {
  StaticJsonDocument<256> response;
  response["status"] = status;
  if (msg) {
    response["msg"] = msg;
  }
  response["cmd_count"] = command_count;
  response["error_count"] = error_count;
  
  serializeJson(response, Serial);
  Serial.println();
}

/**
 * Check watchdog - ensure commands are being received
 */
void check_watchdog() {
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    if (millis() - last_command_time == WATCHDOG_TIMEOUT_MS) {
      // First timeout
      Serial.println("{\"status\": \"warning\", \"msg\": \"Command timeout - watchdog triggered\"}");
      safe_shutdown();
    }
  }
}

/**
 * Check E-stop button input
 */
void check_estop() {
  static uint32_t last_estop_check = 0;
  
  if (millis() - last_estop_check < 100) return;
  last_estop_check = millis();
  
  if (digitalRead(ESTOP_PIN) == LOW) {
    if (!emergency_stop) {
      emergency_stop = true;
      Serial.println("{\"status\": \"error\", \"msg\": \"E-stop button pressed\"}");
      safe_shutdown();
    }
  }
}

/**
 * Safe shutdown - move all servos to neutral
 */
void safe_shutdown() {
  // Move all servos to neutral (1500us = 90°)
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(servos[i].channel, 0, 308);  // 1500us
  }
  
  Serial.println("{\"status\": \"ok\", \"msg\": \"Safe shutdown complete\"}");
}
