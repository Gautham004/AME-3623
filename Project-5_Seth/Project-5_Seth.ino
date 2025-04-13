/* 
Project 5
Date: 3/22/2024
Group 12
Group Members: Brayden Laird, Craig Holt, Gabriel Mendes
*/

#include <PWMServo.h>
#include <math.h>

const int rightMotor_PWM = A18;     // pwm pin for right motor
const int rightMotor_IN_A = 34;     // A pin for motor
const int rightMotor_IN_B = 35;     // B pin for motor

const int leftMotor_PWM = A9;
const int leftMotor_IN_A = 31;
const int leftMotor_IN_B = 32;

const int rearMotor_PWM = A0;
const int rearMotor_IN_A = 39;
const int rearMotor_IN_B = 38;

const int switchPin = 28;

const int idle = 0;
int motor_state = 0;
int i;
int j=0;
int k=0;
int m=0;

float power = 0.0;
const float min_value = -64.0;
const float max_value = 64.0;
const float f1_powergain = 0.9;

unsigned long current_time;
unsigned long last_action_time;
unsigned long state_started = 0;

const int fsm_interval = 50;
const float R = 5.5;

const float duty_cycle = 64; 
const float hover_cycle = 70;

// Setup for the central fan
PWMServo fan;
const int CENTRAL_FAN_PWM = 30;
void fan_setup(){
    fan.attach(CENTRAL_FAN_PWM);
    delay(100);
    fan.write(20);
    delay(3000);
}

void setup() {
    // Initializing the Serial port
    Serial.begin(115200);
    Serial.setTimeout(1000000000);
    delay(1000);
    fan_setup();        // this will call our function fan_setup() to initialize the central hovering fan

    // Setting the pins connected to IN_A & IN_B to output
    pinMode(34, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(39, OUTPUT);
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);

    pinMode(28, INPUT_PULLUP);
}

// holds the different finite states we have
typedef enum state{
    STATE_START,
    STATE_TURNON1,
    STATE_POSTORQ,
    STATE_NEGTORQ,
    STATE_ALLOFF1,
    STATE_TURNON2,
    STATE_FORWARD,
    STATE_BACKWARD,
    STATE_ALLOFF2,
    STATE_TURNON3,
    STATE_RIGHT,
    STATE_LEFT,
    STATE_ALLOFF3,
} State;

// Handles the timer that calls the fsm_step after a set amount of time
void loop() {
    current_time = millis();
    if (current_time - last_action_time > fsm_interval) {
        fsm_step();
        last_action_time = current_time;
    }
}

// Function Description: Applies a boundary to the power given to the motors,
//      if its higher than max, just returns max_value, lower than min_value just returns min_value
// Input Parameters: float value, float min_value, float max_value; value is the actual power level we are setting the motor to
// Return Variable: returns an int value of power that we want to give to the motor thats within the bounds
float bound(float value, float min_value, float max_value){
    if (value < min_value){
        value = min_value;
    }
    if (value > max_value){
        value = max_value;
    }
    return (value);
}

// Function Description: Sets the pins connected to each respective motor to a certain level
//      with respect to what state we are in
// Input Parameters: int motor_state; this is the state of the machine we are in
//                   float power; this is the returned value from float bound function
// Return Value: N/A
void set_motor(int motor_state, float power){
    // STATE_START: ALL IDLE/OFF
    // STATE_ALLOFF1
    if (motor_state == 0){
        analogWrite(leftMotor_PWM, idle);
        digitalWrite(leftMotor_IN_A, LOW);
        digitalWrite(leftMotor_IN_B, LOW);

        analogWrite(rightMotor_PWM, idle);
        digitalWrite(rightMotor_IN_A, LOW);
        digitalWrite(rightMotor_IN_B, LOW);
        
        analogWrite(rearMotor_PWM, idle);
        digitalWrite(rearMotor_IN_A, LOW);
        digitalWrite(rearMotor_IN_B, LOW);
    
      
        fan.write(idle);
    }

    // STATE_TURNON1: CENTRAL FAN ON
    // STATE_TURNON2
    if (motor_state == 1){
        analogWrite(leftMotor_PWM, idle);
        digitalWrite(leftMotor_IN_A, LOW);
        digitalWrite(leftMotor_IN_A, LOW);
    
        analogWrite(rightMotor_PWM, idle);
        digitalWrite(rightMotor_IN_A, LOW);
        digitalWrite(rightMotor_IN_B, LOW);
    
        analogWrite(rearMotor_PWM, idle);
        digitalWrite(rearMotor_IN_A, LOW);
        digitalWrite(rearMotor_IN_B, LOW);
        fan.write(power);
    }

    // STATE_POSTORQ: turning in +Torque direction for 10 seconds
    if (motor_state == 2){
        float F1, F2, F3;

        // this will send 0,0,1 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(0, 0, 1, F1, F2, F3);
        Serial.printf("+T set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("+T set_motor F value's after power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);

        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);    // we send (1,0,0) for F1 being negative to get the gain
            Serial.printf("+T Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            // here we use a powergain because of power differences between left and right motor
            analogWrite(leftMotor_PWM, (F1*f1_powergain)); 
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 *= reverse_gain(0,1,0);   // we send (0,1,0) for F2 being negative to get the gain
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 *= reverse_gain(0,0,1);  // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }

    // STATE_NEGTORQ: turning in -Torq direction for 10 seconds
    if(motor_state == 3){
        float F1, F2, F3;

        // this will send 0,0,1 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(0, 0, 1, F1, F2, F3);
        Serial.printf("-T set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("-T set_motor F value's after power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        
        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);    // we send (1,0,0) for F1 being negative to get the gain
            Serial.printf("-T Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(leftMotor_PWM, F1);
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 = reverse_gain(0,1,0);   // we send (0,1,0) for F2 being negative to get the gain
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 = reverse_gain(0,0,1);   // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }

    // STATE_FORWARD: going in +x direction for 10 seconds
    if(motor_state == 4){
        float F1, F2, F3;

        // this will send 1,0,0 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(1, 0, 0, F1, F2, F3);
        Serial.printf("+x set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("+x set_motor F value's after *= power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        
        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);    // we send (1,0,0) for F1 being negative to get the gain
            Serial.printf("+x Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            // here we use a powergain because of power differences between left and right motor
            analogWrite(leftMotor_PWM, (F1*f1_powergain));
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 *= reverse_gain(0,1,0);  // we send (0,1,0) for F2 being negative to get the gain
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 *= reverse_gain(0,0,1);  // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }

    // STATE_BACKWARD: going in -x direction for 10 seconds
    if(motor_state == 5){
        float F1, F2, F3;

        // this will send 1,0,0 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(1, 0, 0, F1, F2, F3);
        Serial.printf("-x set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("-x set_motor F value's after *= power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        
        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);    // we send (1,0,0) for F1 being negative to get the gain
            Serial.printf("-x Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            // here we use a powergain because of power differences between left and right motor
            analogWrite(leftMotor_PWM, (F1*f1_powergain));
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 *= reverse_gain(0,1,0);  // we send (0,1,0) for F2 being negative to get the gain
            Serial.printf("-x Heres what abs F2 is after correction: %.4f\n", abs(F2));
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 *= reverse_gain(0,0,1);  // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }

    // STATE_RIGHT: we are going in the -y direction (right)
    if(motor_state == 6){
        float F1, F2, F3;

        // this will send 0,1,0 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(0, 1, 0, F1, F2, F3);
        Serial.printf("-y set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("-y set_motor F value's after *= power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        
        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);
            Serial.printf("-y Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            // here we use a powergain because of power differences between left and right motor
            analogWrite(leftMotor_PWM, (F1*f1_powergain));
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 *= reverse_gain(0,1,0);  // we send (0,1,0) for F2 being negative to get the gain
            Serial.printf("-y Heres what abs F2 is after correction: %.4f\n", abs(F2));
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 *= reverse_gain(0,0,1);  // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }


    if(motor_state == 7){
        float F1, F2, F3;

        // this will send 0,1,0 to the function and the values will be
        // returned and set as F1, F2, F3 in this function
        set_hovercraft_forces(0, 1, 0, F1, F2, F3);
        Serial.printf("+y set_motor F value's b4 power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        // Multiplying each fan force we got from set_hovercraft_forces to properly
        //  set each fan to their respective power level we are wanting at the moment
        F1 *= power;
        F2 *= power;
        F3 *= power;
        Serial.printf("+y set_motor F value's after *= power:\n");
        Serial.printf("F1: %.4f, F2: %.4f, F3: %.4f\n", F1, F2, F3);
        
        // If the F1 value is negative we have to account for efficiency loss for motor in reverse
        if(F1 < 0){
            F1 *= reverse_gain(1, 0, 0);    // we send (1,0,0) for F1 being negative to get the gain
            Serial.printf("+y Heres what abs F1 is after correction: %.4f\n", abs(F1));
            analogWrite(leftMotor_PWM, abs(F1));
            digitalWrite(leftMotor_IN_A, LOW);
            digitalWrite(leftMotor_IN_B, HIGH);
        }
        else
        {
            // here we use a powergain because of power differences between left and right motor
            analogWrite(leftMotor_PWM, (F1*f1_powergain));
            digitalWrite(leftMotor_IN_A, HIGH);
            digitalWrite(leftMotor_IN_B, LOW);
        }

        // If the F2 value is negative we have to account for efficiency loss for motor in reverse
        if(F2 < 0){
            F2 *= reverse_gain(0,1,0);  // we send (0,1,0) for F2 being negative to get the gain
            Serial.printf("+y Heres what abs F2 is after correction: %.4f\n", abs(F2));
            analogWrite(rightMotor_PWM, abs(F2));
            digitalWrite(rightMotor_IN_A, LOW);
            digitalWrite(rightMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rightMotor_PWM, F2);
            digitalWrite(rightMotor_IN_A, HIGH);
            digitalWrite(rightMotor_IN_B, LOW);
        }

        // If the F3 value is negative we have to account for efficiency loss for motor in reverse
        if(F3 < 0){
            F3 *= reverse_gain(0,0,1);  // we send (0,0,1) for F3 being negative to get the gain
            analogWrite(rearMotor_PWM, abs(F3));
            digitalWrite(rearMotor_IN_A, LOW);
            digitalWrite(rearMotor_IN_B, HIGH);
        }
        else
        {
            analogWrite(rearMotor_PWM, F3);
            digitalWrite(rearMotor_IN_A, HIGH);
            digitalWrite(rearMotor_IN_B, LOW);
        }

        fan.write(95);
    }
}

// Function description: Handles the different possible states we can have
void fsm_step(){
    static State state = STATE_START; // This is the beginning state we want to start in

    switch(state){

        // STATE_START: Sits at idle and waits for switch to be pressed to start motors
        case STATE_START:
            i = 0;
            if(j == 0){
                Serial.printf("Starting state\n");
                Serial.printf("Press the switch to start...\n");
                j++;
            }
            motor_state = 0;
            power = idle;
            set_motor(motor_state, power);
            
            if(digitalRead(switchPin)){
                i = 1;
                Serial.printf("Switch Pressed! Starting...\n");
            }

            if (i == 1){
                state = STATE_TURNON1;
                i++;
                state_started = millis();
            }
        break;



        // Starting Central Fan | 2. & 3.
        case STATE_TURNON1: 
            if(k==0){
                Serial.printf("Starting!\n");

                k++;
            }
            motor_state = 1;
            set_motor(motor_state, power);
            power++;
            bound(power, 0, hover_cycle);

            // wait in state for 15 seconds
            if(current_time - state_started > 15000){
                Serial.printf("Longer than 15sec\n");
                state = STATE_POSTORQ;
                power = 0;
                state_started = millis();
            } 
        break;


        // Turn outer motors on to turn +Torq direction 10 seconds | 4.
        case STATE_POSTORQ:
            if (m == 0){
                Serial.printf("Turning +Torque Direction!!\n");
                m++;
            }
            motor_state = 2;
            power = 60.0;
            Serial.printf("Power: %f\n", power);
            set_motor(motor_state, (power*120));    // We have to do power*120 to get a power that will turn side fans on
            power++;
            bound(power, min_value, max_value);

            // wait in state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_NEGTORQ;
                power = 0;
                j = 0;
                state_started = millis();
            }

        break;

        // Turn outer motors in opposite direction for -Torq direction 10 seconds | 5.
        case STATE_NEGTORQ:
            if (j == 0){
                Serial.printf("Turning -Torque Direction!!\n");
                j++;
            }
            motor_state = 3;     
            power = -60.0;
            set_motor(motor_state, (power*120));    // We have to do power*120 to get a power that will turn side fans on
            power--;
            bound(power, min_value, max_value);

            // wait in state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_ALLOFF1;
                power = 0;
                k = 0;
                state_started = millis();
            }
        break;


        // Turn all fans off and wait 15 seconds | 6. & 7.
        case STATE_ALLOFF1:
            if (k == 0){
                Serial.printf("Turning off central fan for 15 seconds!\n");
                k++;
            }
            motor_state = 0;
            power = idle;
            set_motor(motor_state, power);

            // wait in this state for 15 seconds, then move on
            if(current_time - state_started > 15000){
                state = STATE_TURNON2;
                power = 0;
                m = 0;
                state_started = millis();
            }
        break;

        // Starting Central Fan | 8. & 9.
        case STATE_TURNON2:
            if (m == 0){
                Serial.printf("Turning on Central Fan, waiting 10 seconds!\n");
                m++;
            }
            motor_state = 1;
            set_motor(motor_state, power);
            power++;
            bound(power, 0, hover_cycle);

            if(current_time - state_started > 10000){
                state = STATE_FORWARD;
                power = 0;
                j=0;
                state_started = millis();
            }
        break;

        // Turn outer motors on to go forward direction (+x) 10 seconds | 10.
        case STATE_FORWARD:
            if (j == 0){
                Serial.printf("We going forward for 10 seconds!\n");
                j++;
            }
            motor_state = 4;
            power = 60;
            set_motor(motor_state, (power*15)); // We have to do power*15 to get a power that will turn side fans on
            power++;
            bound(power, min_value, max_value);

            // wait in state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_BACKWARD;
                power = 0;
                k = 0;
                state_started = millis();
            }
        break;


        // Turn outer motors opposite to last to go backward direction (-x) 10 seconds | 11.
        case STATE_BACKWARD:
            if(k == 0){
                Serial.printf("We going backward for 10 seconds!\n");
                k++;
            }
            motor_state = 5;
            power = -60;
            set_motor(motor_state, (power*15)); // We have to do power*15 to get a power that will turn side fans on
            power--;
            bound(power, min_value, max_value);

            // wait in state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_ALLOFF2;
                power = 0;
                m = 0;
                state_started = millis();
            }
        break;

        
        case STATE_ALLOFF2:
            if(m == 0){
                Serial.printf("We shutting down for 15 seconds!\n");
                m++;
            }
            motor_state = 0;
            power = idle;
            set_motor(motor_state, power);

            // wait in this state for 15 seconds, then move on
            if(current_time - state_started > 15000){
                state = STATE_TURNON3;
                power = 0;
                j = 0;
                state_started = millis();
            }
        break;


        case STATE_TURNON3:
            if (j == 0){
                Serial.printf("Turning on Central Fan, waiting 10 seconds!\n");
                j++;
            }
            motor_state = 1;
            set_motor(motor_state, power);
            power++;
            bound(power, 0, hover_cycle);

            // wait in this state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_RIGHT;
                power = 0;
                k=0;
                state_started = millis();
            }
        break;

        // Turn outer motors on so we going in -y direction (right) for 10 seconds
        case STATE_RIGHT:
            if (k == 0){
                Serial.printf("Going in the RIGHT direction, haha get it?\n");
                k++;
            }

            motor_state = 6;
            power = -60.0;
            set_motor(motor_state, (power*15)); // We have to do power*15 to get a power that will turn side fans on
            power--;
            bound(power, min_value, max_value);

            // wait in this state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_LEFT;
                power = 0;
                m=0;
                state_started = millis();
            }
        break;

        // Turn outer motors on so we going in +y direction (left) for 10 seconds
        case STATE_LEFT:
            if (m == 0){
                Serial.printf("Going in the LEFT direction, yeah this one doesnt work as well...\n");
                m++;
            }
            motor_state = 7;
            power = 60.0;
            set_motor(motor_state, (power*15)); // We have to do power*15 to get a power that will turn side fans on
            power++;
            bound(power, min_value, max_value);

            // wait in this state for 10 seconds, then move on
            if(current_time - state_started > 10000){
                state = STATE_ALLOFF3;
                power = 0;
                j=0;
                state_started = millis();
            }
        break;


        case STATE_ALLOFF3:
            if(j == 0){
                Serial.printf("We are all done! Shutting down...\n");
                j++;
            }

            motor_state = 0;
            power = idle;
            set_motor(motor_state, power);
            
            if(current_time - state_started > 5000){
                state = STATE_START;
                power = 0;
                j=0;
                state_started = 0;
            }
        break;
    }
}

// Function Description: Calculates the forces for each fan depending on direction we are wanting the hovercraft to go in
// Input Parameters: float fx, float fy, float torque: Depending on what we are wanting the hovercraft to do we send different numbers
//                   i.e. if we are wanting to go in x direction we would send 1, 0, 0 so fx=1 and the rest is 0.
//                   float& F1, float& F2, float& F3: pointers to respective variables in set_motor that will be returned to be used after calculated.
// Return Value: Pointers of F1, F2, F3 that the value calculated here will be sent back to be used.
void set_hovercraft_forces(float fx, float fy, float torque, float& F1, float& F2, float& F3){
    // Right F1 = ((sqrt(3)*R*fx) - (R*fy - torque)) / (3R)
    // Rear F2 = ((sqrt(3)*R*fx) + (R*fy + torque)) / (3R)
    // Left F3 = (-2R*fy + torque) / (3R)
    F1 = ((sqrt(3)*R*fx) - (R*fy) - (torque)) / (3*R);
    F2 = ((sqrt(3)*R*fx) + (R*fy) + (torque)) / (3*R);
    F3 = (((-2)*R*fy) + (torque)) / (3*R);
}

// Function Description: Returns a gain value to adjust the power to a fan running in reverse to account for inefficiencies running in reverse.
// Input Parameters: float f1, float f2, float f3: Depending on what is negative we send a 1 to get the gain value returned for F1
//                  ex: F1 < 0 -> (1,0,0)
//                  ex: F2 < 0 -> (0,1,0)
//                  ex: F3 < 0 -> (0,0,1)
// Return Variables: Fan gain for the respective fan that is running in reverse.
float reverse_gain(float f1, float f2, float f3){
    const float f1_gain = 1.0;
    const float f2_gain = 1.2;
    const float f3_gain = 1.2;

    if(f1 == 1){
        return f1_gain;
    }
    if(f2 == 1){
        return f2_gain;
    }
    if(f3 == 1){
        return f3_gain;
    }
    return 0;
}