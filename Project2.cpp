void setup() {
    Serial.begin(115200);  // Start serial communication
    Serial.setTimeout(100000000);  // Ensure enough time for user input
    delay(1000);  // Allow time for the serial connection
}

const int ledPins[5] = {2, 3, 4, 5, 6};  // Define LED pairs

for (int i = 0; i < 5; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);  // Start with LEDs off
}

void loop() {
    Serial.println("Enter distance: ");
    while (!Serial.available());  // Wait for user input

    float distance = Serial.parseFloat() / 1000.0;  // Convert mm to meters
    display_distance(distance);
}


void display_distance(float dist) {
    int ledPattern = 0;

    if (dist >= 0.1 && dist < 0.2) ledPattern = 0b10000;
    else if (dist >= 0.2 && dist < 0.3) ledPattern = 0b11000;
    else if (dist >= 0.3 && dist < 0.4) ledPattern = 0b01000;
    else if (dist >= 0.4 && dist < 0.5) ledPattern = 0b01100;
    else if (dist >= 0.5 && dist < 0.6) ledPattern = 0b00100;
    else if (dist >= 0.6 && dist < 0.7) ledPattern = 0b00110;
    else if (dist >= 0.7 && dist < 0.8) ledPattern = 0b00010;
    else ledPattern = 0b00001;  // Default case

    // Apply pattern to LEDs
    for (int i = 0; i < 5; i++) {
        digitalWrite(ledPins[i], (ledPattern >> (4 - i)) & 1);
    }
}
