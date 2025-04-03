#include <Arduino.h>

// Pin Definitions
const int MOTOR_PWM = 9;      // PWM pin for motor speed
const int INB1 = 27;          // Motor input B (direction)
const int INA1 = 28;          // Motor input A (direction)
const int SWITCH_PIN = 2;     // Input switch pin

// Timing Variables
unsigned long fsm_interval = 50;      
unsigned long last_fsm_time = 0;

// Switch debouncing variables
boolean lastSwitchState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;     

// State variables
boolean motorRunning = false;
boolean lastButtonState = HIGH;

// Bound function
float bound(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  else if (value > max_value) return max_value;
  else return value;
}

// Set motor speed and direction
void set_motor(float val) {
  val = bound(val, -64, 64);
  int duty = (int)(abs(val) * 255.0 / 64.0); // Scale to 0–255

  // Set direction based on sign of val
  if (val > 0) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
  } else if (val < 0) {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
  } else {
    // Optional: brake mode
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
  }

  analogWrite(MOTOR_PWM, duty);

  Serial.print("Setting motor to ");
  Serial.print(val);
  Serial.print(" (");
  Serial.print((abs(val) / 64.0) * 100);
  Serial.println("%)");
}

// Read switch with debouncing
boolean readSwitch() {
  boolean reading = digitalRead(SWITCH_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastSwitchState) {
      lastSwitchState = reading;
      if (lastSwitchState == HIGH) {
        return true;
      }
    }
  }

  lastButtonState = reading;
  return false;
}

void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Using internal pull-up resistor
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);

  // Initialize motor off
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);

  Serial.begin(115200);
  Serial.println("Motor Control System Initialized");
  Serial.println("Press the switch to turn motor ON at 25% power");
  Serial.println("Press again to turn motor OFF");
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_fsm_time > fsm_interval) {
    if (readSwitch()) {
      motorRunning = !motorRunning;

      if (motorRunning) {
        Serial.println("Switch pressed - Motor ON (25%) FORWARD");
        set_motor(30);  // 25% forward
      } else {
        Serial.println("Switch pressed - Motor OFF");
        set_motor(0);
      }
    }

    last_fsm_time = current_time;
  }
}
