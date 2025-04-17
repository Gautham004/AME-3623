//************************************************************************************
// Project 5: Inverse Kinematics                                                     *
// Team 3   : Niels Larsen, Juan Oulon, Gautham Chandra, Andrew Martinez, Deepak Jha *
// Date     : 04-03-2025                                                             *
//************************************************************************************

#include <Servo.h>

Servo fan; // create servo object to control the fan
const int CENTRAL_FAN_PWM = 23; // Initializing the Teensy PWM pin connected to the central fan (pin 23)


// defining the pwm, inA, and inB pins for Fan 1
#define FAN1_PWM 9
#define FAN1_INA 28
#define FAN1_INB 27

// defining the pwm, inA, and inB pins for Fan 2
#define FAN2_PWM 6 // stays the same
#define FAN2_INA 30
#define FAN2_INB 29

// defining the pwm, inA, and inB pins for Fan 3
#define FAN3_PWM 3 //reversing with fan 1
#define FAN3_INA 32
#define FAN3_INB 31

// defining switch pin, duty cycle, and FSM interval globally
#define SWITCH_PIN 2 // Define the pin for the switch
#define DUTY_CYCLE 64 // 25% of 255
#define FSM_INTERVAL 50 // Interval for FSM updates

// global declaration
float motor_values[3] = {0, 0, 0};

// State machine states
typedef enum {
    STATE_START,
	STATE_LIFT_UP,
	STATE_POSITIVE_TORQUE,
	STATE_NEGATIVE_TORQUE,
	STATE_FANS_OFF_1,
	STATE_LIFT_UP_2,
	STATE_FORWARD_FORCE,
	STATE_BACKWARD_FORCE,
	STATE_FANS_OFF_2,
	STATE_LIFT_UP_3,
	STATE_RIGHT_FORCE,
	STATE_LEFT_FORCE,
	STATE_FANS_OFF_3,
} State;

// setting the last fsm time to zero
unsigned long last_fsm_time = 0;

// setting the counter to zero
int counter = 0;

// starting the hovercraft in the start state
State currentState = STATE_START;

void setup() {

  /* 
  Setup Function

  Inputs:
  -none

  Outputs:
  -none
 
  Description:
  This function initializes any necessary settings or configurations before the main loop begins.
  It is called once at the beginning of the program.
  
  */

// configuring the pins attached to the fans as outputs
    pinMode(FAN1_PWM, OUTPUT);
    pinMode(FAN1_INA, OUTPUT);
    pinMode(FAN1_INB, OUTPUT);
    pinMode(FAN2_PWM, OUTPUT);
    pinMode(FAN2_INA, OUTPUT);
    pinMode(FAN2_INB, OUTPUT);
    pinMode(FAN3_PWM, OUTPUT);
    pinMode(FAN3_INA, OUTPUT);
    pinMode(FAN3_INB, OUTPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);
	pinMode(CENTRAL_FAN_PWM, OUTPUT);
    fan_setup();

// setting the sampling rate
    Serial.begin(115200);
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
    val = bound(val, -DUTY_CYCLE, DUTY_CYCLE);

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
}


void fan_setup() {

  /*
  fan_setup function

  Inputs: 
  - none

  Outputs:
  - none

  Description:
  This function sets up the central fan of the hovercraft, then gives low throttle. 
  */
  
  fan.attach(CENTRAL_FAN_PWM); // attaches the fan to specified Arduino pin to the object
  delay(100);
  fan.write(20); // write low throttle
  delay(3000);
}

void set_hovercraft_forces(float fx, float fy, float torque) {

  /*
  set_hovercraft_forces function

  Inputs: 
  - fx
  - fy
  - torque

  Outputs:
  - motor_values[0]
  - motor_values[1]
  - motor_values[2]

  Description:
  This function uses inverse kinematics to determine how much each motor needs to work to
  achieve the desired forces and torques.
  */

  // setting the radius of the hovercraft to 5.5 inches
  int R = 5.5;
 
  // using inverse kinematics matrix to define the motor speeds
  motor_values[0] = ((fx/(2*cos(30))) - (fy/(2+2*sin(30))) - (torque/(2*R*(1+sin(30)))));
  motor_values[1] = ((fx/(2*cos(30))) + (fy/(2+2*sin(30))) + (torque/(2*R*(1+sin(30)))))*.5;
  motor_values[2] = ((-fy/(1+sin(30))) + (torque * sin(30)/(R*(1 + sin(30)))));
  set_motors(motor_values);
}



void set_hovercraft_forces2(float fx, float fy, float torque) { //second hovercraft forces function for the sake of the directional movement

  /*
  set_hovercraft_forces function

  Inputs: 
  - fx
  - fy
  - torque

  Outputs:
  - motor_values[0]
  - motor_values[1]
  - motor_values[2]

  Description:
  This function uses inverse kinematics to determine how much each motor needs to work to
  achieve the desired forces and torques.
  */

  // setting the radius of the hovercraft to 5.5 inches
  int R = 5.5;
 
  // using inverse kinematics matrix to define the motor speeds
  motor_values[0] = ((fx/(2*cos(30))) - (fy/(2+2*sin(30))) - (torque/(2*R*(1+sin(30))))) * .75; //scalar multiplier to nerf overpowered fan
  motor_values[1] = ((fx/(2*cos(30))) + (fy/(2+2*sin(30))) + (torque/(2*R*(1+sin(30))))) * .75; //scalar multiplier to nerf overpowered fan
  motor_values[2] = ((-fy/(1+sin(30))) + (torque * sin(30)/(R*(1 + sin(30))))); //no multiplier here - this should remain the same as to ensure the blue fan has the highest output
  set_motors(motor_values);
}

void set_motors(float val[3]) {

  /*
  set_motors function
   
   Inputs:
   - val[3] (float array)

   Outputs:
   - none

   Description:
   This function updates the speed and direction of the three motors based on the values in the val array.
  */

// declaring the negative gain array
    const float negative_gain[3] = {1.0, 1.0, 1.0};

// for loop that starts at zero and counts up by intervals of 1 until it reaches 3. Serves to check the speed of each motor and provide a correction if necessary
    for (int i = 0; i < 3; i++) {
        if (val[i] < 0) val[i] *= negative_gain[i];
    }

// calling the set_motor function to control the speed and direction of each fan
    set_motor(FAN1_PWM, FAN1_INA, FAN1_INB, val[0]);
    set_motor(FAN2_PWM, FAN2_INA, FAN2_INB, val[1]);
    set_motor(FAN3_PWM, FAN3_INA, FAN3_INB, val[2]);
}

void fsm_step() {

  /* 
  fsm_step function

   Inputs:
   - none

   Outputs:
   - none

   Description:
   This function is part of the finite state machine that controls the behavior of the fans. This function manages the system's state transitions based on
   time (counter) and the state of the switch (SWICH_PIN)
  */


// switch statement to handle different states of the fsm
    switch (currentState) {


// STATE_START: waiting for the switch to be pressed
        case STATE_START:


// if the switch is pressed (LOW), transition to STATE_LIFT_UP
            if (digitalRead(SWITCH_PIN) == HIGH) {
                currentState = STATE_LIFT_UP;
            }
            break;



// STATE_LIFT_UP: set central fan to a 25-40% duty cycle (255 = 100% duty cycle)
        case STATE_LIFT_UP:
            fan.write(102);

// after 300 cycles (15 seconds), transition to the next state (STATE_POSITIVE_TORQUE) and reset the counter
            if (++counter >= 300) {
                currentState = STATE_POSITIVE_TORQUE;
                counter = 0;
            }
            break;



// STATE_POSITIVE_TORQUE: ramp up motors 0, 1, and 2 to the duty cycle speed to create a positive torque
        case STATE_POSITIVE_TORQUE:
            set_hovercraft_forces(0, 0, 60);

// after 300 cycles (15 seconds), transition to the next state (STATE_NEGATIVE_TORQUE) and reset the counter
            if (++counter >= 300) {
                currentState = STATE_NEGATIVE_TORQUE;
                counter = 0;
            }
            break;



// STATE_NEGATIVE_TORQUE: ramp down motors 0, 1, and 2 to the negative value of duty cycle
        case STATE_NEGATIVE_TORQUE:
           set_hovercraft_forces(0, 0, -10);

// after 300 cycles (15 seconds), transition to the next state (STATE_FANS_OFF_1) and reset counter
            if (++counter >= 300) {
                currentState = STATE_FANS_OFF_1;
                counter = 0;
            }
            break;



// STATE_FANS_OFF_1: turn off the central fan and wait for 15 seconds
        case STATE_FANS_OFF_1:
			set_hovercraft_forces(0, 0, 0);
            fan.write(0);

// after 300 cycles (15 seconds), transition to the next state (STATE_LIFT_UP_2) and reset counter
            if (++counter >= 300) {
                currentState = STATE_LIFT_UP_2;
                counter = 0;
            }
            break;



// STATE_LIFT_UP_2: turn on central fan and wait for 10 seconds
        case STATE_LIFT_UP_2:
            fan.write(102);

// after 200 cycles (10 seconds), transition to the next state (STATE_FORWARD_FORCE) and reset counter
            if (++counter >= 200) {
                currentState = STATE_FORWARD_FORCE;
                counter = 0;
            }
            break;



// STATE_FORWARD_FORCE: Move forwards (+x)
        case STATE_FORWARD_FORCE:
		set_hovercraft_forces(-64, 0, 0);
            
// after 300 cycles (15 seconds), transition to the next state (STATE_BACKWARD_FORCE) and reset counter
            if (++counter >= 300) {
                currentState = STATE_BACKWARD_FORCE;
                counter = 0;
            }
            break;



// STATE_BACKWARD_FORCE: generate backward force by calling set_hovercraft_forces function
        case STATE_BACKWARD_FORCE:
        set_hovercraft_forces(64, 0, 0);

// after 300 cycles (15 seconds), transition to the next state (STATE_FANS_OFF_2)
            if (++counter >= 300) {
                currentState = STATE_FANS_OFF_2;
                counter = 0;
            }
            break;


    
// STATE_FANS_OFF_2: turn off all fans
        case STATE_FANS_OFF_2:
            set_hovercraft_forces(0, 0, 0);
            fan.write(0);

// after 300 cycles (15 seconds), transition to the next state (STATE_LIFT_UP_3)
            if (++counter >= 300) {
                currentState = STATE_LIFT_UP_3;
                counter = 0;
            }
            break;
   

   
// STATE_LIFT_UP_3: turn on central fan for 10 seconds
        case STATE_LIFT_UP_3:
            fan.write(102);

// after 200 cycles (10 seconds), transition to the next state (STATE_RIGHT_FORCE)
            if (++counter >= 200) {
                currentState = STATE_RIGHT_FORCE;
                counter = 0;
            }
            break;
    


// STATE_RIGHT_FORCE: generate right force by calling set_hovercraft_forces function
        case STATE_RIGHT_FORCE:
        set_hovercraft_forces2(0, -60, 0);
				
// after 300 cycles (15 seconds), transition to the next state (STATE_LEFT_FORCE)
            if (++counter >= 300) {
                currentState = STATE_LEFT_FORCE;
                counter = 0;
            }
            break;
			
			
			
// STATE_LEFT_FORCE: generate left force by calling set_hovercraft_forces function
        case STATE_LEFT_FORCE:
        set_hovercraft_forces2(0, 60, 0);
		
// after 300 cycles (15 seconds), transition to the next state (STATE_FANS_OFF_3)
            if (++counter >= 300) {
                currentState = STATE_FANS_OFF_3;
                counter = 0;
            }
            break;



// STATE_FANS_OFF_3: turn off all fans and return to start
        case STATE_FANS_OFF_3:
        set_hovercraft_forces(0, 0, 0);
		    fan.write(0);

// after 300 cycles (15 seconds), transition to the next state (STATE_START)
            if (++counter >= 300) {
                currentState = STATE_START;
                counter = 0;
            }
            break;



// print the current state to the serial monitor for debugging purposes
    Serial.printf("Current State: %d\n", currentState);
	}
}

void loop() {
 /* Loop Function

  Inputs:
  -None

  Outputs:
  -none

  Description: 
  This function contains the main code that runs repeatedly. It is called continuously in a loop to perform 
  ongoing tasks or operations. The function checks the time elapsed and runs the fsm_step function at regular intervals.
  */

// get the current time in ms since the program started
    unsigned long current_time = millis();

// check if enough time has passed since the lastfsm update (based on FSM_INTERVAL)
    if (current_time - last_fsm_time > FSM_INTERVAL) {

// if the time interval has passed, call the fsm_step function to update the fsm
        fsm_step();

// update the last fsm time to the current time for the next interval check
        last_fsm_time = current_time;
    }
}