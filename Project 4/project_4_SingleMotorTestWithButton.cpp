#include <Arduino.h>

// Pin Definitions
const int MOTOR_PIN = 9;              // PWM pin for the motor
const int SWITCH_PIN = 2;             // Input switch pin

// Timing Variables
unsigned long fsm_interval = 50;      // Delay between FSM steps (ms)
unsigned long last_fsm_time = 0;

// Switch debouncing variables
boolean lastSwitchState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;     // Debounce time in ms

// State variables
boolean motorRunning = false;
boolean lastButtonState = HIGH;       // Assuming pull-up resistor (button not pressed)

// Function to bound a value between min and max
float bound(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  else if (value > max_value) return max_value;
  else return value;
}

// Function to set motor speed (0-64 range)
void set_motor(float val) {
  val = bound(val, 0, 64);
  int duty = (int)(val * 255.0 / 64.0); // Scale to 0–255
  analogWrite(MOTOR_PIN, duty);
  
  Serial.print("Setting motor to ");
  Serial.print(val);
  Serial.print(" (");
  Serial.print((val/64.0)*100);
  Serial.println("%)");
}

// Read switch with debouncing
boolean readSwitch() {
  // Read the current state of the switch
  boolean reading = digitalRead(SWITCH_PIN);
  
  // Check if switch changed from last reading (could be noise)
  if (reading != lastButtonState) {
    // Reset the debounce timer
    lastDebounceTime = millis();
  }
  
  // If button state is stable for longer than debounce delay
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has truly changed
    if (reading != lastSwitchState) {
      lastSwitchState = reading;
      
      if (lastSwitchState == HIGH) {
        return true;  // Button was pressed and released
      }
    }
  }
  
  lastButtonState = reading;
  return false;  // No valid button press detected
}

void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Using internal pull-up resistor
  pinMode(MOTOR_PIN, OUTPUT);
  
  // Initialize motor to off
  analogWrite(MOTOR_PIN, 0);
  
  Serial.begin(115200);
  Serial.println("Motor Control System Initialized");
  Serial.println("Press the switch to turn motor ON at 25% power");
  Serial.println("Press it again to turn motor OFF");
}

void loop() {
  unsigned long current_time = millis();
  
  if (current_time - last_fsm_time > fsm_interval) {
    // Check for switch press
    if (readSwitch()) {
      // Toggle motor state
      motorRunning = !motorRunning;
      
      if (motorRunning) {
        Serial.println("Switch pressed - Motor ON (25%)");
        set_motor(16);  // 25% of 64 = 16
      } else {
        Serial.println("Switch pressed - Motor OFF");
        set_motor(0);   // Turn off motor
      }
    }
    
    last_fsm_time = current_time;
  }
}