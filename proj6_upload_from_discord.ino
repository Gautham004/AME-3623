//************************************************************************************
// Project 6: Yaw Damping                                                     *
// Team 3   : Niels Larsen, Juan Oulon, Gautham Chandra, Andrew Martinez, Deepak Jha *
// Date     : 04-03-2025                                                             *
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
#define FAN1_PWM 9
#define FAN1_INA 28
#define FAN1_INB 27
#define FAN2_PWM 6
#define FAN2_INA 30
#define FAN2_INB 29
#define FAN3_PWM 3
#define FAN3_INA 32
#define FAN3_INB 31

// User button and FSM control definitions
#define SWITCH_PIN 2
#define DUTY_CYCLE 64
#define FSM_INTERVAL 500 // FSM update interval in milliseconds
#define IMU_INTERVAL 5   // New IMU update interval in milliseconds

// IMU object setup (I2C address 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// FSM state definitions
typedef enum {
    STATE_START,        // Wait for user input
    STATE_LIFT_UP,      // Start lifting the hovercraft
    STATE_YAW_DAMP,     // Run PD yaw damping
    STATE_FANS_OFF      // Stop all fans and reset
} State;

unsigned long last_fsm_time = 0; // Timestamp for FSM updates
unsigned long last_imu_time = 0; // Timestamp for IMU polling task
int counter = 0;                  // Generic counter for timing within states
State currentState = STATE_START; // Initial FSM state
float motor_values[3] = {0, 0, 0}; // Output values to each motor

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
        while (1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
}

float bound(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
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

void fan_setup() {
    fan.attach(CENTRAL_FAN_PWM);
    delay(100);
    fan.write(20);
    delay(3000);
}

void set_hovercraft_forces(float fx, float fy, float torque) {
    int R = 5.5;
    motor_values[0] = ((fx / (2 * cos(30))) - (fy / (2 + 2 * sin(30))) - (torque / (2 * R * (1 + sin(30)))));
    motor_values[1] = ((fx / (2 * cos(30))) + (fy / (2 + 2 * sin(30))) + (torque / (2 * R * (1 + sin(30)))));
    motor_values[2] = ((-fy / (1 + sin(30))) + (torque * sin(30) / (R * (1 + sin(30)))));
    set_motors(motor_values);
}

void set_motors(float val[3]) {
    const float negative_gain[3] = {1.0, 1.0, 1.0};
    for (int i = 0; i < 3; i++) {
        if (val[i] < 0) val[i] *= negative_gain[i];
    }
    set_motor(FAN1_PWM, FAN1_INA, FAN1_INB, val[0]);
    set_motor(FAN2_PWM, FAN2_INA, FAN2_INB, val[1]);
    set_motor(FAN3_PWM, FAN3_INA, FAN3_INB, val[2]);
}

void pd_step() {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float fx = 0.0;
    float fy = 0.0;
    float kd = 0.25;
    float threshold = 0.5; // deg/s
    float torque = 0.0;

    if (abs(gyro.z()) > threshold) {
        torque = -kd * gyro.z();
    }

    set_hovercraft_forces(fx, fy, torque);
    Serial.printf("PD Step | Yaw Rate: %.2f deg/s | Torque: %.2f\n", gyro.z(), torque);
}

void imu_step() {
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.printf("IMU Poll | Orientation: x=%.2f y=%.2f z=%.2f\n", event.orientation.x, event.orientation.y, event.orientation.z);
}

void fsm_step() {
    switch (currentState) {
        case STATE_START:
            if (digitalRead(SWITCH_PIN) == HIGH) {
                currentState = STATE_LIFT_UP;
                counter = 0;
            }
            break;

        case STATE_LIFT_UP:
            fan.write(102);
            currentState = STATE_YAW_DAMP;
            counter = 0;
            break;

        case STATE_YAW_DAMP:
            pd_step();
            if (++counter >= 600) {
                currentState = STATE_FANS_OFF;
                counter = 0;
            }
            break;

        case STATE_FANS_OFF:
            set_hovercraft_forces(0, 0, 0);
            fan.write(0);
            currentState = STATE_START;
            counter = 0;
            break;
    }
    Serial.printf("Current State: %d\n", currentState);
}

void loop() {
    unsigned long current_time = millis();
    if (current_time - last_fsm_time > FSM_INTERVAL) {
        fsm_step();
        last_fsm_time = current_time;
    }
    if (current_time - last_imu_time > IMU_INTERVAL) {
        imu_step();
        last_imu_time = current_time;
    }
}
