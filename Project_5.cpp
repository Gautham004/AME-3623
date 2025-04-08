//************************************************************************************
// Project 5: Inverse Kinematics                                                     *
// Team 3   : Neils Larsen, Juan Oulon, Gautham Chandra, Andrew Martinez, Deepak Jha *
// Date     : 04-03-2025                                                             *
//************************************************************************************

#include <PWMServo.h> // Include the Servo library


PWMServo fan; // create servo object to control the fan
 const int CENTRAL_FAN_PWM = 23; // Select correct pin for central fan

// Defining pins for lateral motors & switch (select the correct pins)
const int LeftMotor_PWM =  3;
const int LeftMotor_INA =  32;
const int LeftMotor_INB =  31;

const int RightMotor_PWM = 6;      
const int RightMotor_INA = 30;      
const int RightMotor_INB = 29;      

const int RearMotor_PWM =  9;
const int RearMotor_INA =  28;
const int RearMotor_INB =  27;

const int SWITCH_PIN = 2;

// Defining constants for hovercraft control (idle, power limits in each direction, and fsm interval)
const int Idle = 0;
const float Min_Val = -102.0;
const float Max_Val = 102.0;
const int fsm_interval = 50;

// Variables for FSM operation.
int i;
float Power = 0;  
unsigned long current_time;
unsigned long last_fsm_time;

void setup () {
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

    // Configurate the rear motor control pins as an output.
    pinMode(RearMotor_PWM, OUTPUT);
    pinMode(RearMotor_INA, OUTPUT);
    pinMode(RearMotor_INB, OUTPUT);

    // Configure the swith pin as an input with a pull-up resistor.
    pinMode(Switch_Pin, INPUT_PULLUP);

    // Configure central fan pin as output
    pinMode(CENTRAL_FAN_PWM, OUTPUT);
}


}

float bound(float value, float min_value, float max_value) {

  /*
  Bound Function

  Inputs: 
  - value, min_value, max_value

  Outputs:
  - value, min_value, OR max_value

  Description: 
  This function receives input from the set_motor function to know the duty cycle each motor is operating at. The function then checks to make sure the input
  value is within the min and max range, and if the value is outside of this range, the function returns the respective min or max value so that the set_motor 
  function can correct itself to make sure the motors aren't running too fast. 
  
  */
  
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void set_motor(int pwmPin, int inAPin, int inBPin, float val) {

  /*
  set_motor function

  Inputs: 
  - pwmPin (int)
  - inAPin (int)
  - inBPin (int)
  - val (float)

  Outputs:
  - none

  Description:
  This function controls the speed and direction of a motor by adjusting its PWM signal and direction pins. 
  */

    // Calling the bound function to correct for any outlying motor speed values that may be present.
    val = bound(val, -Max_Val, Max_Val);

    // if val > 0,make the fan run forward
    if (val > 0) {
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
    } 

    // if val < 0, make the fan run backward
    else if (val < 0) {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, HIGH);
    } 

    // if val = 0, turn the fan off
    else {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, LOW);
    }

    // send the absolute value of val to the pwmPin
    analogWrite(pwmPin, abs(val));

 // Setup function for central fan
 void fan_setup() {
    fan.attach(CENTRAL_FAN_PWM); // attaches the fan to specified
    // Arduino pin to the object
    delay(100);
    fan.write(20); // write low throttle
    delay(3000);
 }

void set_hovercraft_forces(float fx, float fy, float torque){
    // Set thrust levels for the hovercraft
    int R = 5.5;
 
    // using inverse kinematics matrix to define the motor speeds
    motor_values[0] = (fx/(2*cos(30))) - (fy/(2+2*sin(30))) - (torque/(2*R*(1+sin(30))));
    motor_values[1] = (fx/(2*cos(30))) + (fy/(2+2*sin(30))) + (torque/(2*R*(1+sin(30))));
    motor_values[2] = (-fy/(1+sin(30))) + (torque * sin(30)/(R*(1 + sin(30))));
    set_motors(motor_values);
}

void set_motors(float val[3]) {
    const float negative_gain[3] = {0.8, 1.0, 1.0};
    for (int i = 0; i < 3; i++) {
        if (val[i] < 0) val[i] *= negative_gain[i];
    }
    set_motor(LeftMotor_PWM, LeftMotor_INA, LeftMotor_INB, val[0]);
    set_motor(RightMotor_PWM, RightMotor_INA, RightMotor_INB, val[1]);
    set_motor(RearMotor_PWM, RearMotor_INA, RearMotor_INB, val[2]);
}

typedef enum state{
    STATE_WAITING_FOR_SWITCH,
    STATE_CENTRAL_FAN_ON,
    STATE_CENTRAL_FAN_OFF,
    STATE_WAIT_15,
    STATE_WAIT_10,
    STATE_POS_TORQUE,
    STATE_NEG_TORQUE,
    STATE_FORWARD_FORCE,
    STATE_BACKWARD_FORCE,
    STATE_RIGHT_FORCE,
    STATE_LEFT_FORCE,
} State;


void fsm_step(){

    static int state = STATE_WAITING_FOR_SWITCH; // Initial state

    switch (state) {

        case STATE_WAITING_FOR_SWITCH: // Wait for switch to be pressed
            Serial.println("Waiting for switch to be pressed...\n");
            if (digitalRead(SWITCH_PIN) == HIGH) {
                serial.println("Switch pressed, starting sequence...\n");
                state = STATE_CENTRAL_FAN_ON;
                current_time = millis();
            }
            break;

        case State STATE_CENTRAL_FAN_ON: // Set central fan to 25-40% duty cycle
            fan.write(64); // Example value for 25% duty cycle
            Serial.println("Central fan on...\n");
            state = STATE_WAIT_15;
            current_time = millis();
            break;

        case STATE_WAIT_15: // Wait for 15 seconds
            if (millis() - current_time >= 15000) {
                state = STATE_POS_TORQUE;
            }
            break;

        case STATE_POS_TORQUE: // Generate positive torque for 10 seconds
            set_hovercraft_forces(0, 0, 1.0); // Example positive torque
            serial.println("Generating positive torque...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop torque
                state = STATE_NEG_TORQUE;
                current_time = millis();
            }
            break;

        case STATE_NEG_TORQUE: // Generate negative torque for 10 seconds
            set_hovercraft_forces(0, 0, -1.0); // Example negative torque
            serial.println("Generating negative torque...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop torque
                state = STATE_CENTRAL_FAN_OFF;
                current_time = millis();
            }
            break;

        case STATE_CENTRAL_FAN_OFF: // Turn off the central fan
            fan.write(0); // Turn off fan
            Serial.println("Central fan off...\n");
            state = STATE_WAIT_15;
            current_time = millis();
            break;

        case STATE_WAIT_15: // Wait for 15 seconds
            serial.println("Waiting for 15 seconds...\n");
            if (millis() - current_time >= 15000) {
                state = STATE_CENTRAL_FAN_ON;
            }
            break;

        case STATE_CENTRAL_FAN_ON: // Turn on the central fan
            fan.write(64); // Example value for 25% duty cycle
            Serial.println("Central fan on...\n");
            state = STATE_WAIT_10;
            current_time = millis();
            break;

        case STATE_WAIT_10: // Wait for 10 seconds
            serial.println("Waiting for 10 seconds...\n");
            if (millis() - current_time >= 10000) {
                state = STATE_FORWARD_FORCE;
            }
            break;

        case STATE_FORWARD_FORCE: // Generate forward force for 10 seconds
            set_hovercraft_forces(1.0, 0, 0); // Example forward force
            serial.println("Generating forward force...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop forward force
                state = STATE_BACKWARD_FORCE;
                current_time = millis();
            }
            break;

        case STATE_BACKWARD_FORCE: // Generate backward force for 10 seconds
            set_hovercraft_forces(-1.0, 0, 0); // Example backward force
            serial.println("Generating backward force...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop backward force
                state = STATE_CENTRAL_FAN_OFF;
                current_time = millis();
            }
            break;

        case STATE_CENTRAL_FAN_OFF: // Turn off the central fan
            fan.write(0); // Turn off fan
            Serial.println("Central fan off...\n");
            state = STATE_WAIT_15;
            current_time = millis();
            break;

        case STATE_WAIT_15: // Wait for 15 seconds
            serial.println("Waiting for 15 seconds...\n");
            if (millis() - current_time >= 15000) {
                state = STATE_CENTRAL_FAN_OFF;
            }
            break;

        case STATE_CENTRAL_FAN_ON: // Turn on the central fan
            fan.write(64); // Example value for 25% duty cycle
            Serial.println("Central fan on...\n");
            state = STATE_WAIT_10;
            current_time = millis();
            break;

        case STATE_WAIT_10: // Wait for 10 seconds
            if (millis() - current_time >= 10000) {
                state = STATE_RIGHT_FORCE;
            }
            break;

        case STATE_RIGHT_FORCE: // Generate rightward force for 10 seconds
            set_hovercraft_forces(0, 1.0, 0); // Example rightward force
            serial.println("Generating rightward force...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop rightward force
                state = STATE_LEFT_FORCE;
                current_time = millis();
            }
            break;

        case STATE_LEFT_FORCE: // Generate leftward force for 10 seconds
            set_hovercraft_forces(0, -1.0, 0); // Example leftward force
            serial.println("Generating leftward force...\n");
            if (millis() - current_time >= 10000) {
                set_hovercraft_forces(0, 0, 0); // Stop leftward force
                state = STATE_CENTRAL_FAN_OFF;
                current_time = millis();
            }
            break;

        case STATE_CENTRAL_FAN_OFF: // Turn off the central fan
            fan.write(0); // Turn off fan
            state = STATE_WAITING_FOR_SWITCH; // Return to waiting for the switch
            break;
    }
}

void setup() {
  pinMode(SWITCH_PIN, INPUT); // Set the switch pin as input
  fan_setup(); // Initialize the fan
}

void loop() {
  fsm_step(); // Call the FSM step function defined above
}

