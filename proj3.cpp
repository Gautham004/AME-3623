#include <Arduino.h>
#include <math.h> 

const int sharpPin = A14;
const int sharpPin2 = A15;
const int sharpPin3 = A16;

unsigned long last_sensor_time = 0;
const unsigned long sensor_interval = 100;

// Function to convert sensor readings to distance using the correct equation
float convertToDistance(int sensorValue) {
    return -5 * (sqrt(3)*sqrt(429200*sensorValue - 43938933.0) - 34131.0) / 3219.0;
}

void setup() {
    Serial.begin(115200);
    pinMode(sharpPin, INPUT);
    pinMode(sharpPin2, INPUT);
    pinMode(sharpPin3, INPUT);
}

void sensor_display_task() {
    int val1 = analogRead(sharpPin);
    int val2 = analogRead(sharpPin2);
    int val3 = analogRead(sharpPin3);

    // Convert sensor readings to distances
    float dist1 = convertToDistance(val1);
    float dist2 = convertToDistance(val2);
    float dist3 = convertToDistance(val3);

    // Print raw values and converted distances
    Serial.printf("Sensor 1: %.2f cm, Sensor 2: %.2f cm, Sensor 3: %.2f cm\n", 
                  dist1, dist2, dist3);
}

void loop() {
    const unsigned long current_time = millis();
  
    if (current_time - last_sensor_time >= sensor_interval) {
        sensor_display_task();
        last_sensor_time = current_time;
    }
}
