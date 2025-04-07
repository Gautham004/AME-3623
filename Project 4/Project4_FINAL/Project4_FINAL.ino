// Motor control pins for left, right, and rear motors.
const int LeftMotor_PWM = 3;
const int LeftMotor_INA = 32;
const int LeftMotor_INB = 31;

const int RightMotor_PWM = 6;      
const int RightMotor_INA = 30;      
const int RightMotor_INB = 29;      

const int RearMotor_PWM = 9;
const int RearMotor_INA = 28;
const int RearMotor_INB = 27;

// Control pin.
const int Switch_Pin = 2;

// Constants for motor control.
const int Idle = 0;
const float Min_Val = -64.0;
const float Max_Val = 64.0;
const int fsm_interval = 50;

// Variables for FSM operation.
int i;
float Power = 0;  
unsigned long current_time;
unsigned long last_fsm_time;

void setup ()
{
  // Initialize serial communication.
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  delay(1000);

  // Configurate the left motor control pins as an output.
  pinMode(LeftMotor_PWM, OUTPUT);
  pinMode(LeftMotor_INA, OUTPUT);
  pinMode(LeftMotor_INB, OUTPUT);

  // Configurate the right motor control pins as an output.
  pinMode(RightMotor_PWM, OUTPUT);
  pinMode(RightMotor_INA, OUTPUT);
  pinMode(RightMotor_INB, OUTPUT);

  pinMode(RearMotor_PWM, OUTPUT);
  pinMode(RearMotor_INA, OUTPUT);
  pinMode(RearMotor_INB, OUTPUT);

  // Configure the swith pin as an input with a pull-up resistor.
  pinMode(Switch_Pin, INPUT_PULLUP);
}

void loop()
{
  // Sets variable with actual time in milliseconds.
  current_time = millis();
  // Start when enough time (50 milliseconds) have passed.
  if (current_time - last_fsm_time > fsm_interval)
  {
    // Calls put function fsm_step.
    fsm_step();
    // Updates the last fsm time to the current time value.
    last_fsm_time = current_time;
  }
}

// Definition of the finite states of the motor controller.
typedef enum state
{
  STATE_INITIAL,
  STATE_FIRST_POS,
  STATE_SEC_POS,
  STATE_THIRD_POS,
  STATE_NEGATIVE,
  STATE_FIRST_ZERO,
  STATE_SEC_ZERO,
  STATE_THIRD_ZERO,
} State;

// Function Name: fsm_step.
// Input: None.
// Output: None.
// Description: This function updates the FSM based on the current states.
// It will follow the function path according to the given conditions and
// will change the actions of the motors. This function is called in the main loop. 
void fsm_step()
{
  // Initial State.
  static State state = STATE_INITIAL;
  // Array that holds the value of the three motors.
  float motor_values[3] = {0, 0, 0};

  switch (state) 
  {

    // Waits for switch press to start.
    case STATE_INITIAL:
      i = 0;
      // Print starting message.
      Serial.println(" Please press the switch to start.\n");
      // If the switch is pressed, then...
      if (digitalRead(Switch_Pin) == HIGH) 
      {
        i = 1;
        // Print message after pressing switch.
        Serial.printf("Thank you. Starting.\n");
        Serial.printf("Hovercraft may explode. Proceed with care.\n");
      }
      // Turn off all motors.
      motor_values[0] = Idle;
      motor_values[1] = Idle;
      motor_values[2] = Idle;
      set_motors(motor_values);
      // If the switch was pressed, move to the next state.
      if (i == 1) 
      {
        state = STATE_FIRST_POS;
        Power = 0;
      }
      break;


    // Gradually ramp up LEFT motor.
    case STATE_FIRST_POS:
      Serial.println("LEFT motor ramp up");
      // Limits the power.
      Power = bound(Power + 1, Min_Val, Max_Val);
      // Only left motor gets power.
      motor_values[0] = Power;
      set_motors(motor_values);
      // When the power reaches duty cycle, change to the next state.
      if (Power >= Max_Val) 
      {
        state = STATE_SEC_POS;
        // Reset power.
        Power = 0;
      }
      break;

    // Ramp up Right motor.
    case STATE_SEC_POS:
      Serial.println("In state: RIGHT motor ramp up");
      Power = bound(Power + 1, Min_Val, Max_Val);
      // Left motor maximum power.
      motor_values[0] = Max_Val;
      // Right motor ramps up.
      motor_values[1] = Power;
      set_motors(motor_values);
      // Change to the next state.
      if (Power >= Max_Val) 
      {
        state = STATE_THIRD_POS;
        Power = 0;
      }
      break;

    // Ramp up rear motor.
    case STATE_THIRD_POS:
      Serial.println("In state: REAR motor ramp up");
      Power = bound(Power + 1, Min_Val, Max_Val);
      motor_values[0] = Max_Val;
      motor_values[1] = Max_Val;
      motor_values[2] = Power;
      set_motors(motor_values);
      // Change to the next state.
      if (Power >= Max_Val) 
      {
        state = STATE_NEGATIVE;
        Power = Max_Val;
      }
      break;

    // All motors ramp down into reverse direction.
    case STATE_NEGATIVE:
      Serial.println("In state: ALL motors reverse");
      Power = bound(Power - 1, Min_Val, Max_Val);
      motor_values[0] = Power;
      motor_values[1] = Power;
      motor_values[2] = Power;
      set_motors(motor_values);
      if (Power <= Min_Val) 
      {
        state = STATE_FIRST_ZERO;
        Power = Min_Val;
      }
      break;

    // Gradually stop LEFT motor, others remain reversed.
    case STATE_FIRST_ZERO:
      Serial.println("In state: LEFT motor off");
      Power = bound(Power + 1, Min_Val, Max_Val);
      motor_values[0] = Idle;
      motor_values[1] = Power;
      motor_values[2] = Power;
      set_motors(motor_values);
      if (Power >= Idle)
      {
        state = STATE_SEC_ZERO;
        Power = Min_Val;
      }
      break;

    // Gradually stop RIGHT motor, REAR remains reversed.
    case STATE_SEC_ZERO:
      Serial.println("In state: RIGHT motor off");
      Power = bound(Power + 1, Min_Val, Max_Val);
      motor_values[0] = Idle;
      motor_values[1] = Idle;
      motor_values[2] = Power;
      set_motors(motor_values);
      if (Power >= Idle) 
      {
        state = STATE_THIRD_ZERO;
        Power = Min_Val;
      }
      break;

    // Gradually stop REAR motor to complete cycle.
    case STATE_THIRD_ZERO:
      Serial.println("In state: REAR motor off");
      Power = bound(Power + 1, Min_Val, Max_Val);
      motor_values[0] = Idle;
      motor_values[1] = Idle;
      motor_values[2] = Idle;
      set_motors(motor_values);
      // Restart cycle.
      if (Power >= Idle) 
      {
        state = STATE_INITIAL;
        Power = 0;
      }
      break;
  }
}

// Function Name: bound
// Input: val, Value to bound; min_val, Minimum allowed value; max_val, Maximum allowed value
// Output: Value between min_val and max_val.
// Description: Limits the floating point value between the range of a minimum and maximum
// value. Uses if statements.
float bound(float val, float min_val, float max_val)
{
  if (abs(val) < 0.5) 
  {
  return 0;
  }
  if (val < min_val)
  {
    return min_val;
  }
  else if (val > max_val)
  {
    return max_val;
  }
  return val;
}

// Function Name: set_motor.
// Input: motor, Motor to control (1-3); val, Power value.
// Output: None.
// Description: This function controls the speed and direction of a selected motor
// by using pwm for power and the digital signals for direction. Uses if and else if
// statements.
void set_motor(int motor, float val)
{
  int abs_val = abs(val);
  // If true, forward. If false, reverse.
  bool direction = val >= 0;

  // Left motor.
  if (motor == 1)
  {
    analogWrite(LeftMotor_PWM, abs_val);
    digitalWrite(LeftMotor_INA, direction);
    digitalWrite(LeftMotor_INB, !direction);
  }
  // Right motor.
  else if (motor == 2)
  {
    analogWrite(RightMotor_PWM, abs_val);
    digitalWrite(RightMotor_INA, direction);
    digitalWrite(RightMotor_INB, !direction);
  }
  // Rear motor.
  else if (motor == 3)
  {
    analogWrite(RearMotor_PWM, abs_val);
    digitalWrite(RearMotor_INA, direction);
    digitalWrite(RearMotor_INB, !direction);
  }
}

// Function Name: set_motors.
// Input: val[3], Array of three float values for each motor.
// Output: None.
// Description: Sets the magnitude and direction of thrust for the three motors.
void set_motors(float val[3])
{
  const float negative_gain[3] = {1.0, 1.0, 1.0};

  // Process each motor value.
  for (int i = 0; i < 3; i++)
  {
    float motor_val = val[i];

    // Applies a negative gain if the value is negative.
    if (motor_val < 0)
    {
      motor_val *= negative_gain[i];
    }

    // Bound value.
    motor_val = bound(motor_val, Min_Val, Max_Val);

    // Set the corresponding motor.
    set_motor(i + 1, motor_val);
  }
}