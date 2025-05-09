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

unsigned long last_range_time = 0;
const unsigned long RANGE_INTERVAL = 50;  // ms

// ——— global IR‐range storage ———
float distance_right = 0.0f;
float distance_left  = 0.0f;
float distance_rear  = 0.0f;

// Create servo object for central fan
Servo fan;
const int CENTRAL_FAN_PWM = 23;  // Central fan PWM pin

// Motor pin definitions for 3 thrust fans
#define FAN1_PWM 9
#define FAN1_INA 28
#define FAN1_INB 27
#define FAN2_PWM 6
#define FAN2_INA 30
#define FAN2_INB 29
#define FAN3_PWM 3
#define FAN3_INA 32
#define FAN3_INB 31

// User button and timing definitions
#define SWITCH_PIN 2
#define DUTY_CYCLE 64     // max PWM magnitude
#define FSM_INTERVAL 500  // FSM update interval (ms)
#define IMU_INTERVAL 5    // IMU update interval (ms)

//define IR sensor pins
#define left_sensor A14
#define right_sensor A16
#define back_sensor A15



// IMU object setup (I2C address 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// FSM state definitions
typedef enum {
  STATE_START,     // Wait for user input
  STATE_LIFT_UP,   // Spin central fan to lift
  STATE_YAW_DAMP,  // Run D-only yaw damping
  STATE_FANS_OFF   // Stop everything
} State;

unsigned long last_fsm_time = 0;
unsigned long last_imu_time = 0;
unsigned long yawStartTime = 0;
State currentState = STATE_START;

float motor_values[3] = { 0, 0, 0 };


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
    error = error;
    //Serial.println("Issue with bounding in the compute_error function");
  }
  return error;
}

float db_clip(float error, float deadband, float saturation) {
  float s = (error >= 0) ? 1.0f : -1.0f;
  float a = fabsf(error);
  if (a <= deadband) return 0.0f;
  if (a >= saturation) return s * (saturation - deadband);
  return s * (a - deadband);
}

  // ----------------------------------------------------------------------------
  // Low-level motor driver: direction pins + PWM
  // ----------------------------------------------------------------------------
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
    analogWrite(pwmPin, abs((int)val));
  }
  float convertToDistance(int sensorValue) {
    return -5 * (sqrt(3)*sqrt(429200*sensorValue - 43938933.0) - 34131.0) / 3219.0;
}

  //void sensor_display
  void sensor_display_task() {
    
    int val1 = analogRead(left_sensor);
    int val2 = analogRead(right_sensor);
    int val3 = analogRead(back_sensor);
    // Function to convert sensor readings to distance using the correct equation

    //convert analogs to usable distances
     distance_right = convertToDistance(val2);
     distance_left = convertToDistance(val1);
     distance_rear = convertToDistance(val3);
    Serial.printf("right sensor: %.2f cm, left sensor: %.2f, rear sensor: %.2f", distance_right, distance_left, distance_rear);

  }

  // ----------------------------------------------------------------------------
  // Send net forces/torque to hovercraft fans
  // ----------------------------------------------------------------------------
  void set_hovercraft_forces(float fx, float fy, float torque) {
    const float R = 5.5f;
    motor_values[0] = -((fx / (2 * cos(30))) - (fy / (2 + 2 * sin(30))) - (torque / (2 * R * (1 + sin(30)))));
    motor_values[1] = ((fx / (2 * cos(30))) + (fy / (2 + 2 * sin(30))) + (torque / (2 * R * (1 + sin(30)))));
    motor_values[2] = ((-fy / (1 + sin(30))) + (torque * sin(30) / (R * (1 + sin(30)))));
    for (int i = 0; i < 3; i++) {
      set_motor((i == 0 ? FAN1_PWM : (i == 1 ? FAN2_PWM : FAN3_PWM)),
                (i == 0 ? FAN1_INA : (i == 1 ? FAN2_INA : FAN3_INA)),
                (i == 0 ? FAN1_INB : (i == 1 ? FAN2_INB : FAN3_INB)),
                motor_values[i]);
    }
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
  // D-only yaw damping step, driven by IMU rate (deg/s)
  // ----------------------------------------------------------------------------
  void pd_step() {
   sensors_event_t event;
    bno.getEvent(&event);  // Get current orientation
    float current_heading = event.orientation.z;  // z-axis is yaw

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float yawRate = gyro.z();      // deg/s
    const float Kd = 0.25f;                       // derivative gain
    const float Kp = 0.25f;                       // needs to be tuned*****
    const float Kpx = 0.75f;                       // needs to be tuned*****
    const float Kpy = 0.75f;                       // needs to be tuned*****
    const float deadband = 0.5f;                  // deg/s threshold
    const float saturation = 45.0f;                 // deg saturation
    const float maxTorque = DUTY_CYCLE;           // clamp to PWM range
    const float heading_goal = 0.0f;              // value to be adjusted depending on where north is??
    float heading_error = compute_error(current_heading, heading_goal);
    float modified_error = db_clip(heading_error, deadband, saturation);
    float torque = Kp * modified_error - Kd * yawRate;
    torque = constrain(torque, -maxTorque, maxTorque);


  // ——— compute “push” strengths ———
  float base_strength_right = 0.0f;
  float base_strength_left  = 0.0f;
  float base_strength_rear  = 0.0f;

if (distance_right >= 80.0f || distance_right <=0){
     base_strength_right = 0;
  }
  else {
    base_strength_right = distance_right;
  }
  if (distance_left  >= 80.0f || distance_left<=0) {
     base_strength_left  = 0;
  }
  else {
    base_strength_left = distance_left;
  }
  
  if (distance_rear >= 80.0f || distance_rear <=0){
     base_strength_rear = 0;
  }
  else {
    base_strength_rear = distance_rear;
  }



    
   
    //translate the strengths into hovercraft forces
    int horizontal_forces = Kpy*((80-((sqrt(3)/2)*base_strength_right)) - (80-(sqrt(3)/2)*base_strength_left));
    int vertical_forces = Kpx*((80-base_strength_rear) - (80-(0.5*base_strength_right)) - (80-(0.5*base_strength_left))); 

  
   
  // only restoring torque, no translation
  set_hovercraft_forces(horizontal_forces, vertical_forces, torque);
  Serial.printf("D-Step | Rate: %.2f deg/s | Torque: %.2f\n", yawRate, torque);

  }
  

    

// ----------------------------------------------------------------------------
// Poll IMU, print orientation, and run damping if active
// ----------------------------------------------------------------------------
void imu_step() {
  sensors_event_t event;
  bno.getEvent(&event);

  if (currentState == STATE_YAW_DAMP) {
    pd_step();
  }

  //Serial.printf("IMU Poll | Ori x=%.2f y=%.2f z=%.2f\n",
   //             event.orientation.x,
   //             event.orientation.y,
   //             event.orientation.z);
}

// ----------------------------------------------------------------------------
// Finite State Machine: spin up, yaw-damp for fixed time, then shut off
// ----------------------------------------------------------------------------
void fsm_step() {
  switch (currentState) {
    case STATE_START:
      if (digitalRead(SWITCH_PIN) == HIGH) {
        currentState = STATE_LIFT_UP;
      }
      break;

    case STATE_LIFT_UP:
      fan.write(102);  // full-speed lift
      currentState = STATE_YAW_DAMP;
      yawStartTime = millis();  // mark start of damping
      break;

    case STATE_YAW_DAMP:
      // exit after 10 seconds
      if (millis() - yawStartTime > 50000) {
        currentState = STATE_FANS_OFF;
      }
      break;

    case STATE_FANS_OFF:
      set_hovercraft_forces(0, 0, 0);
      fan.write(0);
      currentState = STATE_START;
      break;
  }
  Serial.printf("State = %d\n", currentState);
}

void setup() {
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
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  //  calibration status:
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("Calibration- sys:%d gyro:%d accel:%d mag:%d\n", sys, gyro, accel, mag);
  while (mag < 3) {
  Serial.println("Magnetometer not calibrated! Move device in figure-8 pattern");
        delay(1000);
        bno.getCalibration(&sys, &gyro, &accel, &mag);  // 
  }
}


void loop() {
  unsigned long now = millis();

  // ——— every 50 ms, update distances ———
  if (now - last_range_time >= RANGE_INTERVAL) {
    sensor_display_task();           // read and store left_range, right_range, rear_range
    last_range_time = now;        // reset the timer
  }

  // ——— the rest of your tasks ———
  if (now - last_imu_time   >= IMU_INTERVAL) {
    imu_step();
    last_imu_time = now;
  }
  if (now - last_fsm_time   >= FSM_INTERVAL) {
    fsm_step();
    last_fsm_time = now;
  }
}
