// Pin Definitions
const int MOTOR_PINS[3] = {3, 6, 9};  // PWM pins for the three motors
const int SWITCH_PIN = 2;  // Input switch pin

const int ina1 = 28;
const int inb1 = 27;


// FSM Timing Variables
unsigned long fsm_interval = 50;
unsigned long last_fsm_time = 0;
unsigned long counter = 0;

// Event Enumeration
typedef enum {
  PRESSED,
  UNPRESSED,
  COUNTER_THRESHOLD
} Event;

// State Enumeration
typedef enum {
  STATE_START,
  STATE_L25,
  STATE_R25,
  STATE_B25,
  STATE_NEG25,
  STATE_L0,
  STATE_R0,
  STATE_B0
} State;

State current_state = STATE_START;

// Function to bound a value between min and max
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

// Function to set a single motor with bounded value
void set_motor(int motor, float val) {
  // Bound the value between -64 and 64
  val = bound(val, -64, 64);
  
  // Convert to PWM duty cycle (0-255 range)
  int duty;
  if (val >= 0) {
    duty = (int)(val * 255 / 64);
  } else {
    duty = 0; // For negative values, we'll handle direction differently if needed
  }
  
  analogWrite(MOTOR_PINS[motor], duty);
}

// Function to set all three motors
void set_motors(float val[3]) {
  const float negative_gain[3] = {1.0, 1.0, 1.0};
  
  for (int i = 0; i < 3; i++) {
    float adjusted_val = val[i];
    
    // Apply negative gain if the value is negative
    if (adjusted_val < 0) {
      adjusted_val = adjusted_val * negative_gain[i];
    }
    
    // Set each motor with the adjusted value
    set_motor(i, adjusted_val);
  }
}

void setup() {
  pinMode(SWITCH_PIN, INPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  Serial.begin(115200);
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_fsm_time > fsm_interval) {
    fsm_step();
    last_fsm_time = current_time;
  }
}

void fsm_step() {
  bool switch_pressed = digitalRead(SWITCH_PIN);
  counter++;
 
  switch (current_state) {
    case STATE_START:
      // Initial state - waiting for switch press
      set_motors((float[]){0, 0, 0});
      if (switch_pressed) {
        current_state = STATE_L25;
        counter = 0;
      }
      break;
      
    case STATE_L25:
      // Ramp the left motor up to 25% duty cycle
      // 25% of 64 = 16
      set_motors((float[]){16, 0, 0});
      if (counter >= 20) { 
        current_state = STATE_R25; 
        counter = 0; 
      }
      break;
      
    case STATE_R25:
      // Ramp the right motor up to 25% duty cycle
      set_motors((float[]){16, 16, 0});
      if (counter >= 20) { 
        current_state = STATE_B25; 
        counter = 0; 
      }
      break;
      
    case STATE_B25:
      // Ramp the rear motor up to 25% duty cycle
      set_motors((float[]){16, 16, 16});
      if (counter >= 20) { 
        current_state = STATE_NEG25; 
        counter = 0; 
      }
      break;
      
    case STATE_NEG25:
      // Ramp all three motors down to -25% duty cycle simultaneously
      // -25% of 64 = -16
      set_motors((float[]){-16, -16, -16});
      if (counter >= 20) { 
        current_state = STATE_L0; 
        counter = 0; 
      }
      break;
      
    case STATE_L0:
      // Ramp the left motor up to 0% duty cycle
      set_motors((float[]){0, -16, -16});
      if (counter >= 20) { 
        current_state = STATE_R0; 
        counter = 0; 
      }
      break;
      
    case STATE_R0:
      // Ramp the right motor up to 0% duty cycle
      set_motors((float[]){0, 0, -16});
      if (counter >= 20) { 
        current_state = STATE_B0; 
        counter = 0; 
      }
      break;
      
    case STATE_B0:
      // Ramp the rear motor up to 0% duty cycle
      set_motors((float[]){0, 0, 0});
      if (counter >= 20) { 
        current_state = STATE_START; 
        counter = 0; 
      }
      break;
  }
  
  Serial.printf("Current State: %d\n", current_state);
}