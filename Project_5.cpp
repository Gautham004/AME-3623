//************************************************************************************
// Project 5: Inverse Kinematics                                                     *
// Team 3   : Neils Larsen, Juan Oulon, Gautham Chandra, Andrew Martinez, Deepak Jha *
// Date     : 04-03-2025                                                             *
//************************************************************************************

#include <Servo.h> // Include the Servo library


PWMServo fan; // create servo object to control the fan
 const int CENTRAL_FAN_PWM = ???; // Select correct pin for central fan

 // Setup function for central fan
 void fan_setup() {
    fan.attach(CENTRAL_FAN_PWM); // attaches the fan to specified
    // Arduino pin to the object
    delay(100);
    fan.write(20); // write low throttle
    delay(3000);
 }

// Defining pins for lateral motors & switch (select the correct pins)
const int LeftMotor_PWM =  ???;
const int LeftMotor_INA =  ???;
const int LeftMotor_INB =  ???;

const int RightMotor_PWM = ???;      
const int RightMotor_INA = ???;      
const int RightMotor_INB = ???;      

const int RearMotor_PWM =  ???;
const int RearMotor_INA =  ???;
const int RearMotor_INB =  ???;

const int SWITCH_PIN = ???;

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
}

void set_hovercraft_forces(float fx, float fy, float torque){
//     // Set thrust levels for the hovercraft
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
    static int state = 0; // Current state
    static unsigned long start_time = 0; // Time when the state started

    switch (state) {
        case 0: // Wait for switch to be pressed
            if (digitalRead(SWITCH_PIN) == HIGH) { // Replace SWITCH_PIN with actual pin
                state = 1;
                stateStartTime = millis();
            }
            break;

        case 1: // Set central fan to 25-40% duty cycle
            fan.write(64); // Example value for 25% duty cycle
            state = 2;
            stateStartTime = millis();
            break;

        case 2: // Wait for 15 seconds
            if (millis() - stateStartTime >= 15000) {
                state = 3;
            }
            break;

        case 3: // Generate positive torque for 10 seconds
            set_hovercraft_forces(0, 0, 1.0); // Example positive torque
            state = 4;
            stateStartTime = millis();
            break;

        case 4: // Generate negative torque for 10 seconds
            if (millis() - stateStartTime >= 10000) {
                set_hovercraft_forces(0, 0, -1.0); // Example negative torque
                state = 5;
                stateStartTime = millis();
            }
            break;

        case 5: // Turn off the central fan
            fan.write(0); // Turn off fan
            state = 6;
            stateStartTime = millis();
            break;

        case 6: // Wait for 15 seconds
            if (millis() - stateStartTime >= 15000) {
                state = 7;
            }
            break;

        case 7: // Turn on the central fan
            fan.write(64); // Example value for 25% duty cycle
            state = 8;
            stateStartTime = millis();
            break;

        case 8: // Wait for 10 seconds
            if (millis() - stateStartTime >= 10000) {
                state = 9;
            }
            break;

        case 9: // Generate forward force for 10 seconds
            set_hovercraft_forces(1.0, 0, 0); // Example forward force
            state = 10;
            stateStartTime = millis();
            break;

        case 10: // Generate backward force for 10 seconds
            if (millis() - stateStartTime >= 10000) {
                set_hovercraft_forces(-1.0, 0, 0); // Example backward force
                state = 11;
                stateStartTime = millis();
            }
            break;

        case 11: // Turn off the central fan
            fan.write(0); // Turn off fan
            state = 12;
            stateStartTime = millis();
            break;

        case 12: // Wait for 15 seconds
            if (millis() - stateStartTime >= 15000) {
                state = 13;
            }
            break;

        case 13: // Turn on the central fan
            fan.write(64); // Example value for 25% duty cycle
            state = 14;
            stateStartTime = millis();
            break;

        case 14: // Wait for 10 seconds
            if (millis() - stateStartTime >= 10000) {
                state = 15;
            }
            break;

        case 15: // Generate rightward force for 10 seconds
            set_hovercraft_forces(0, 1.0, 0); // Example rightward force
            state = 16;
            stateStartTime = millis();
            break;

        case 16: // Generate leftward force for 10 seconds
            if (millis() - stateStartTime >= 10000) {
                set_hovercraft_forces(0, -1.0, 0); // Example leftward force
                state = 17;
                stateStartTime = millis();
            }
            break;

        case 17: // Turn off the central fan
            fan.write(0); // Turn off fan
            state = 0; // Return to waiting for the switch
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

