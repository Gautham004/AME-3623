//************************************************************************************
// Project 8: Obstacle avoidance
// Team 3   : Niels Larsen, Juan Oulon, Gautham Chandra, Andrew Martinez, Deepak Jha
// Date     : 04-03-2025
//************************************************************************************

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create servo object for central fan
Servo fan;
const int CENTRAL_FAN_PWM = 23; // Central fan PWM pin

// Motor pin definitions for 3 thrust fans
#define FAN1_PWM   9
#define FAN1_INA  28
#define FAN1_INB  27
#define FAN2_PWM   6
#define FAN2_INA  30
#define FAN2_INB  29
#define FAN3_PWM   3
#define FAN3_INA  32
#define FAN3_INB  31

// User button and timing definitions
#define SWITCH_PIN   2
#define DUTY_CYCLE   64        // max PWM magnitude
#define FSM_INTERVAL 500       // FSM update interval (ms)
#define IMU_INTERVAL 5         // IMU update interval (ms)

// IMU object setup (I2C address 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//define IR sensor pins
#define left_sensor A14
#define right_sensor A16
#define back_sensor A15


typedef enum {
    STATE_START,        // Wait for user input
    STATE_LIFT_UP,      // Spin central fan to lift
    STATE_YAW_Control,     // Run D-only yaw damping
    STATE_FANS_OFF      // Stop everything
} State;

// FSM state definitions

State currentState = STATE_START;
unsigned long last_fsm_time = 0;
unsigned long last_imu_time = 0;
unsigned long yawStartTime = 0;


float motor_values[3] = {0, 0, 0};


// ----------------------------------------------------------------------------
// Utility: bound a value into [min_value, max_value]
// ----------------------------------------------------------------------------
float bound(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}


//------------------------------------------------------------------------------
// provides with error between goal and current heading between 180 and -180 degrees 
//------------------------------------------------------------------------------
float compute_error(float theta, float heading_goal) {
  //compute the error based on the current heading - theta
  //mathematically - a heading that is overly positive should result in a negative action in order to correct
  //the opposite is true - Note that positive is counterclockwise- and needs clockwise motion to correct (negative error)
  float error = heading_goal - theta;
  //now bounding is needed - if over 180 then bound to 180, if less then -180 then bound to -180.
  if (error >= 180) {
    error = 180;
  } else if (error <= -180) {
    error = -180;
  } else {
    Serial.println("Issue with bounding in the compute_error function");
  }
  return error;
}


float db_clip(float error, float deadband, float saturation){
  float s = (error >= 0) ? 1.0f : -1.0f;
  float a = fabsf(error);
  if (a <= deadband)       return 0.0f;
  if (a >= saturation)     return s * (saturation - deadband);
  return s * (a - deadband);
}
void set_motor(int pwmPin, int inAPin, int inBPin, float val) {
    val = bound(val, -DUTY_CYCLE, DUTY_CYCLE);
    if (val > 0) {
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
    } else if (val < 0) {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, HIGH);
    } else {
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, LOW);
    }
    analogWrite(pwmPin, abs(val));
}

// ----------------------------------------------------------------------------
// Spin central fan to preload lift
// ----------------------------------------------------------------------------
void fan_setup() {
    fan.attach(CENTRAL_FAN_PWM);
    delay(100);
    fan.write(20);
    delay(3000);
}


// ----------------------------------------------------------------------------
// Send net forces/torque to hovercraft fans
// ----------------------------------------------------------------------------
void set_hovercraft_forces(float fx, float fy, float torque) {
    const float R = 5.5f;
    motor_values[0] = -((fx/(2*cos(30))) - (fy/(2+2*sin(30))) - (torque/(2*R*(1+sin(30)))));
    motor_values[1] =  ((fx/(2*cos(30))) + (fy/(2+2*sin(30))) + (torque/(2*R*(1+sin(30)))));
    motor_values[2] =  ((-fy/(1+sin(30))) + (torque*sin(30)/(R*(1+sin(30)))));
    set_motors(motor_values);
}
// ----------------------------------------------------------------------------
// Low-level motor driver: direction pins + PWM
// ----------------------------------------------------------------------------
void set_motors(float val[3]) {
    const float negative_gain[3] = {1.0, 1.0, 1.0};
    for (int i = 0; i < 3; i++) {
        if (val[i] < 0) val[i] *= negative_gain[i];
    }
    set_motor(FAN1_PWM, FAN1_INA, FAN1_INB, val[0]);
    set_motor(FAN2_PWM, FAN2_INA, FAN2_INB, val[1]);
    set_motor(FAN3_PWM, FAN3_INA, FAN3_INB, val[2]);
}

// ----------------------------------------------------------------------------
// D-only yaw damping step, driven by IMU rate (deg/s)
// ----------------------------------------------------------------------------
void pd_step() {
    sensors_event_t event;
    bno.getEvent(&event);  // Get current orientation
    float current_heading = event.orientation.z;  // z-axis is yaw

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float yawRate = gyro.z();      // deg/s
    
    // Control parameters
    float fx = 0.0;
    float fy = 0.0;
    const float Kp = 0.25f;
    const float Kd = 0.25f;
    const float deadband = 0.5f;
    const float saturation = 45.0f;  // Fixed float literal
    const float maxTorque = DUTY_CYCLE;
    const float heading_goal = 0.0f;
    
    // Compute errors
    float heading_error = compute_error(current_heading, heading_goal);
    float modified_error = db_clip(heading_error, deadband, saturation);
    
    // Calculate control output
    float torque = Kp * modified_error - Kd * yawRate;
    torque = constrain(torque, -maxTorque, maxTorque);
    
    // Apply forces
    set_hovercraft_forces(fx, fy, torque);
    Serial.printf("PD-Step | Error: %.2f° | Rate: %.2f°/s | Torque: %.2f\n", 
                 heading_error, yawRate, torque);
}
// Add these new global variables
bool calibrationRequested = false;
bool firstButtonPress = false;
unsigned long lastButtonPressTime = 0;
const unsigned long DOUBLE_PRESS_DELAY = 500; // ms between button presses
// ----------------------------------------------------------------------------
// Poll IMU, print orientation, and run damping if active
// ----------------------------------------------------------------------------
void imu_step() {
    sensors_event_t event;
    bno.getEvent(&event);
Serial.printf("IMU Poll | Orientation: x=%.2f y=%.2f z=%.2f\n", event.orientation.x, event.orientation.y, event.orientation.z);
}

// ----------------------------------------------------------------------------
// Finite State Machine: spin up, yaw-damp for fixed time, then shut off
// ----------------------------------------------------------------------------
void fsm_step() {
    static State prevState = STATE_START;
    if (currentState != prevState) {
        Serial.printf("State changed from %d to %d\n", prevState, currentState);
        prevState = currentState;
    }

    switch (currentState) {
        case STATE_START: {
            int buttonState = digitalRead(SWITCH_PIN);
            Serial.printf("Button state: %d\n", buttonState);
            if (buttonState == LOW) {  // LOW when pressed (pullup)
                Serial.println("Button pressed - starting lift");
                currentState = STATE_LIFT_UP;
            }
            break;
        }

        case STATE_LIFT_UP:
            Serial.println("Activating central fan");
            fan.write(102);
            currentState = STATE_YAW_Control;
            yawStartTime = millis();
            break;

        case STATE_YAW_Control:
            pd_step();
             if (yawStartTime >= 800) {
                currentState = STATE_FANS_OFF;
            }
            break;

        case STATE_FANS_OFF:
            Serial.println("Stopping all motors");
            set_hovercraft_forces(0, 0, 0);
            fan.write(0);
            currentState = STATE_START;
            break;
    }
}

void setup() {
    pinMode(FAN1_PWM,   OUTPUT);
    pinMode(FAN1_INA,   OUTPUT);
    pinMode(FAN1_INB,   OUTPUT);
    pinMode(FAN2_PWM,   OUTPUT);
    pinMode(FAN2_INA,   OUTPUT);
    pinMode(FAN2_INB,   OUTPUT);
    pinMode(FAN3_PWM,   OUTPUT);
    pinMode(FAN3_INA,   OUTPUT);
    pinMode(FAN3_INB,   OUTPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(CENTRAL_FAN_PWM, OUTPUT);

    fan_setup();
    Serial.begin(115200);
    Serial.println("System ready. Press button TWICE to start.");

    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055!");
        while (1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    
    // Calibration check - FIXED variable names
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.printf("Calibration- sys:%d gyro:%d accel:%d mag:%d\n", sys, gyro, accel, mag);
    while (mag < 3) {
        Serial.println("Magnetometer not calibrated! Move device in figure-8 pattern");
        delay(1000);
        bno.getCalibration(&sys, &gyro, &accel, &mag);  // Fixed variable names
    }
}


void loop() {
    unsigned long now = millis();

    if (now - last_fsm_time > FSM_INTERVAL) {
        fsm_step();
        last_fsm_time = now;
    }

    if (now - last_imu_time > IMU_INTERVAL) {
        imu_step();
        last_imu_time = now;
    }
}