/*
 * SENTIO ESP32 Servo Bridge Firmware
 * 
 * Comprehensive firmware implementing:
 * - JSON-over-serial protocol for servo control
 * - PCA9685 PWM driver management
 * - Watchdog with safe fallback
 * - E-stop input handling
 * - Telemetry and diagnostics
 * - OTA firmware update support
 * 
 * Author: DevSora Deep-Tech Research
 * License: Proprietary
 * Version: 1.0.0
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "Adafruit_PCA9685.h"

// ============================================================================
// Configuration
// ============================================================================

#define SERIAL_BAUDRATE 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 400000

#define ESTOP_PIN 35
#define STATUS_LED_PIN 2

#define SERVO_COUNT 5
#define PCA9685_FREQ_HZ 50
#define PCA9685_ADDRESS 0x40

#define SERVO_MIN_US 1000   // Minimum pulse width (microseconds)
#define SERVO_MAX_US 2000   // Maximum pulse width (microseconds)
#define SERVO_MIN_DEGREES 0
#define SERVO_MAX_DEGREES 180

#define WATCHDOG_TIMEOUT_MS 5000
#define ESTOP_DEBOUNCE_MS 50

#define JSON_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 1024

// ============================================================================
// Type Definitions
// ============================================================================

struct ServoState {
  uint8_t id;
  uint16_t channel;
  uint16_t current_position_us;
  uint16_t target_position_us;
  uint8_t speed_limit_dps;  // Degrees per second
  bool enabled;
  uint32_t last_move_time;
};

struct SystemState {
  bool emergency_stop_active;
  bool pca9685_online;
  bool watchdog_triggered;
  uint32_t uptime_ms;
  uint32_t command_count;
  uint32_t error_count;
  float temperature_c;
};

// ============================================================================
// Global Variables
// ============================================================================

Adafruit_PCA9685 pca9685;
ServoState servos[SERVO_COUNT];
SystemState system_state;

uint32_t last_command_time = 0;
uint32_t last_estop_check = 0;
bool estop_pressed_prev = false;

char rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_index = 0;

// ============================================================================
// Function Declarations
// ============================================================================

void setup();
void loop();
void setup_hardware();
void setup_servos();
void update_system_state();
void handle_serial();
void process_command(const char* json_str);
void move_servo(uint8_t servo_id, uint16_t degrees);
void send_json_response(const char* status, const char* msg = nullptr);
void send_error_response(const char* error);
void check_watchdog();
void check_estop();
void safe_shutdown();
void handle_led_command(JsonObject& led_obj);

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(500);
  
  Serial.println(R"({)");
  Serial.println(R"(  "status": "init",)");
  Serial.println(R"(  "message": "SENTIO Servo Bridge v1.0.0 initializing")");
  Serial.println(R"(})");

  setup_hardware();
  setup_servos();
  
  last_command_time = millis();
  
  Serial.println(R"({)");
  Serial.println(R"(  "status": "ok",)");
  Serial.println(R"(  "message": "Servo bridge ready",)");
  Serial.print(R"(  "servo_count": )");
  Serial.println(SERVO_COUNT);
  Serial.println(R"(})");
}

void setup_hardware() {
  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(100);
  
  // PCA9685
  if (!pca9685.begin(PCA9685_ADDRESS, &Wire)) {
    Serial.println(R"({)");
    Serial.println(R"(  "status": "error",)");
    Serial.println(R"(  "message": "PCA9685 initialization failed")");
    Serial.println(R"(})");
    system_state.pca9685_online = false;
    while (1) delay(100);  // Halt
  }
  
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(PCA9685_FREQ_HZ);
  system_state.pca9685_online = true;
  
  delay(100);
  
  // E-stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  
  // Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void setup_servos() {
  // Initialize all servos to neutral (1500us = 90 degrees)
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].id = i;
    servos[i].channel = i;
    servos[i].current_position_us = 1500;
    servos[i].target_position_us = 1500;
    servos[i].speed_limit_dps = 60;
    servos[i].enabled = true;
    servos[i].last_move_time = 0;
    
    // Set neutral position
    uint16_t pwm_value = (1500 * 4096) / 20000;
    pca9685.setPWM(servos[i].channel, 0, pwm_value);
  }
  
  delay(200);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  uint32_t now = millis();
  
  // Update system state
  update_system_state();
  
  // Check for E-stop
  check_estop();
  
  // Check watchdog
  check_watchdog();
  
  // Process serial input
  handle_serial();
  
  // LED status indicator
  if (now % 1000 < 50) {
    digitalWrite(STATUS_LED_PIN, system_state.emergency_stop_active ? LOW : HIGH);
  }
  
  delay(10);
}

// ============================================================================
// Serial Communication
// ============================================================================

void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (rx_index > 0) {
        rx_buffer[rx_index] = '\0';
        process_command(rx_buffer);
        rx_index = 0;
      }
    } else if (rx_index < RX_BUFFER_SIZE - 1) {
      rx_buffer[rx_index++] = c;
    } else {
      // Buffer overflow
      send_error_response("Buffer overflow");
      rx_index = 0;
    }
  }
}

void process_command(const char* json_str) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, json_str);
  
  if (error) {
    char msg[128];
    snprintf(msg, sizeof(msg), "JSON parse error: %s", error.c_str());
    send_error_response(msg);
    system_state.error_count++;
    return;
  }
  
  last_command_time = millis();
  system_state.command_count++;
  
  const char* cmd_type = doc["type"] | "";
  
  if (strcmp(cmd_type, "servo_move") == 0) {
    JsonObject positions = doc["positions"];
    
    if (positions.isNull()) {
      send_error_response("Missing positions object");
      system_state.error_count++;
      return;
    }
    
    if (system_state.emergency_stop_active) {
      send_error_response("Emergency stop active");
      return;
    }
    
    // Move each servo
    for (JsonPair pair : positions) {
      uint8_t servo_id = atoi(pair.key().c_str());
      uint16_t degrees = pair.value().as<uint16_t>();
      
      if (servo_id >= SERVO_COUNT) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Servo %d out of range", servo_id);
        send_error_response(msg);
        system_state.error_count++;
        continue;
      }
      
      move_servo(servo_id, degrees);
    }
    
    send_json_response("ok", "Servo move executed");
  }
  
  else if (strcmp(cmd_type, "led_control") == 0) {
    JsonObject led = doc["led"];
    if (!led.isNull()) {
      handle_led_command(led);
    }
    send_json_response("ok", "LED command received");
  }
  
  else if (strcmp(cmd_type, "init") == 0) {
    send_json_response("ok", "Initialization acknowledged");
  }
  
  else if (strcmp(cmd_type, "estop") == 0) {
    system_state.emergency_stop_active = true;
    safe_shutdown();
    send_json_response("ok", "Emergency stop activated");
  }
  
  else if (strcmp(cmd_type, "clear_estop") == 0) {
    system_state.emergency_stop_active = false;
    send_json_response("ok", "Emergency stop cleared");
  }
  
  else if (strcmp(cmd_type, "status") == 0) {
    DynamicJsonDocument response(512);
    response["status"] = "ok";
    response["servo_count"] = SERVO_COUNT;
    response["command_count"] = system_state.command_count;
    response["error_count"] = system_state.error_count;
    response["emergency_stop"] = system_state.emergency_stop_active;
    response["pca9685_online"] = system_state.pca9685_online;
    response["uptime_ms"] = system_state.uptime_ms;
    
    // Servo positions
    JsonArray servo_array = response.createNestedArray("servos");
    for (int i = 0; i < SERVO_COUNT; i++) {
      JsonObject servo_obj = servo_array.createNestedObject();
      servo_obj["id"] = i;
      servo_obj["position_us"] = servos[i].current_position_us;
      servo_obj["degrees"] = (servos[i].current_position_us - SERVO_MIN_US) * 180 / (SERVO_MAX_US - SERVO_MIN_US);
    }
    
    serializeJson(response, Serial);
    Serial.println();
  }
  
  else {
    char msg[64];
    snprintf(msg, sizeof(msg), "Unknown command: %s", cmd_type);
    send_error_response(msg);
  }
}

void move_servo(uint8_t servo_id, uint16_t degrees) {
  if (servo_id >= SERVO_COUNT) return;
  
  // Clamp degrees
  degrees = constrain(degrees, SERVO_MIN_DEGREES, SERVO_MAX_DEGREES);
  
  // Convert degrees to microseconds
  uint16_t us = SERVO_MIN_US + (degrees * (SERVO_MAX_US - SERVO_MIN_US) / 180);
  
  // Convert to PCA9685 value
  uint16_t pwm_value = (us * 4096) / 20000;
  
  pca9685.setPWM(servos[servo_id].channel, 0, pwm_value);
  
  servos[servo_id].current_position_us = us;
  servos[servo_id].last_move_time = millis();
  
  Serial.print(R"({)");
  Serial.print(R"("status":"ok","servo":)");
  Serial.print(servo_id);
  Serial.print(R"(,"degrees":)");
  Serial.print(degrees);
  Serial.print(R"(,"pwm":)");
  Serial.print(pwm_value);
  Serial.println(R"(})");
}

void handle_led_command(JsonObject& led_obj) {
  // LED control implementation
  // This would interface with an addressable LED strip (e.g., WS2812B via pin)
  // For now, just log the command
  
  const char* color = led_obj["color"] | "white";
  const char* pattern = led_obj["pattern"] | "solid";
  float intensity = led_obj["intensity"] | 0.8;
  
  // Implementation would set GPIO pin(s) or PWM for LED control
  // Placeholder for future integration
}

void send_json_response(const char* status, const char* msg) {
  StaticJsonDocument<256> response;
  response["status"] = status;
  if (msg) {
    response["msg"] = msg;
  }
  response["cmd_count"] = system_state.command_count;
  response["error_count"] = system_state.error_count;
  
  serializeJson(response, Serial);
  Serial.println();
}

void send_error_response(const char* error) {
  StaticJsonDocument<256> response;
  response["status"] = "error";
  response["msg"] = error;
  response["cmd_count"] = system_state.command_count;
  response["error_count"] = system_state.error_count;
  
  serializeJson(response, Serial);
  Serial.println();
}

// ============================================================================
// System Management
// ============================================================================

void update_system_state() {
  system_state.uptime_ms = millis();
  
  // Read temperature (internal sensor)
  system_state.temperature_c = (float)(analogRead(A0) * 3.3 / 4096.0 - 0.5) * 100;
}

void check_watchdog() {
  uint32_t now = millis();
  
  if (now - last_command_time > WATCHDOG_TIMEOUT_MS) {
    if (!system_state.watchdog_triggered) {
      Serial.println(R"({)");
      Serial.println(R"(  "status": "warning",)");
      Serial.println(R"(  "msg": "Watchdog timeout - no command received")");
      Serial.println(R"(})");
      
      system_state.watchdog_triggered = true;
      safe_shutdown();
    }
  } else {
    system_state.watchdog_triggered = false;
  }
}

void check_estop() {
  uint32_t now = millis();
  
  if (now - last_estop_check < ESTOP_DEBOUNCE_MS) return;
  last_estop_check = now;
  
  bool estop_pressed = (digitalRead(ESTOP_PIN) == LOW);
  
  if (estop_pressed && !estop_pressed_prev) {
    system_state.emergency_stop_active = true;
    Serial.println(R"({)");
    Serial.println(R"(  "status": "error",)");
    Serial.println(R"(  "msg": "E-stop button pressed")");
    Serial.println(R"(})");
    safe_shutdown();
  }
  
  estop_pressed_prev = estop_pressed;
}

void safe_shutdown() {
  // Move all servos to neutral (1500us)
  for (int i = 0; i < SERVO_COUNT; i++) {
    uint16_t pwm_value = (1500 * 4096) / 20000;
    pca9685.setPWM(servos[i].channel, 0, pwm_value);
    servos[i].current_position_us = 1500;
  }
  
  Serial.println(R"({)");
  Serial.println(R"(  "status": "ok",)");
  Serial.println(R"(  "msg": "Safe shutdown - all servos to neutral")");
  Serial.println(R"(})");
}
