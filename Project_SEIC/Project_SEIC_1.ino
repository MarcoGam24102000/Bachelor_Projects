#include <BlynkSimpleEsp32.h>
#include <Ultrasonic.h>

BlynkTimer timer; // Initialize Blynk timer

byte ledPin = 21; // LED pin
volatile byte state = LOW; // State variable for LED

#define MQ2 15 // MQ2 sensor pin
int sensorValue = 0; // Variable to store MQ2 sensor value

byte ledPin2 = 12; // Another LED pin
volatile byte state2 = LOW; // State variable for second LED

#define INTERVALO_LEITURA 250 // Interval for reading the ultrasonic sensor (ms)

#define PIN_TRIGGER 4 // Trigger pin for ultrasonic sensor
#define PIN_ECHO 5 // Echo pin for ultrasonic sensor

// Initialize ultrasonic sensor with trigger and echo pins
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO);

// Function to get distance from the ultrasonic sensor
int getDistance() {
    int distanciaCM; // Variable to store distance in cm
    long microsec = ultrasonic.timing(); // Get timing from ultrasonic sensor
    distanciaCM = ultrasonic.convert(microsec, Ultrasonic::CM); // Convert timing to distance
    return distanciaCM; // Return distance in cm
}

// Function to verify distance and control LED and buzzer
void verificarDistancia() {
    unsigned int distancia = getDistance(); // Get distance from ultrasonic sensor
    if (distancia < 100) { // If distance is less than 100 cm
        state2 = !state2; // Toggle LED state
        digitalWrite(ledPin2, state2); // Write state to LED
        ledcSetup(1, 2500, 8); // Setup PWM channel with 2500 Hz frequency and 8-bit resolution
        ledcAttachPin(23, 1); // Attach pin 23 to PWM channel 1
        ledcWrite(1, 100); // Write PWM value to buzzer

        delay(200); // Delay for 200 ms

        state2 = !state2; // Toggle LED state back
        digitalWrite(ledPin2, state2); // Write state to LED
        ledcWrite(1, 0); // Turn off buzzer
    }

    // Print distance to Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distancia);
    Serial.println("cm\n");
}

// Function to blink LED a specified number of times
void blinkLED_sevTimes(int timesBlink) {
    for (int x = 0; x < timesBlink; x++) { // Loop for number of times to blink
        state = !state; // Toggle LED state
        digitalWrite(ledPin, state); // Write state to LED
        delay(100); // Delay for 100 ms
        state = !state; // Toggle LED state back
        digitalWrite(ledPin, state); // Write state to LED
        delay(200); // Delay for 200 ms
    }
}

// Function to read MQ2 sensor and perform actions based on its value
void sendUptime() {
    sensorValue = analogRead(MQ2); // Read MQ2 sensor value

    Serial.println(sensorValue); // Print sensor value to Serial Monitor

    if (sensorValue > 600) { // If sensor value is greater than 600
        blinkLED_sevTimes(10); // Blink LED 10 times
        Serial.println("Gas Detected!"); // Print gas detection message
    } else {
        Serial.println("No gas!"); // Print no gas message
    }
}

// Function to run Blynk timer
void readMQ2() {
    timer.run(); // Run Blynk timer
}

// Function to read distance from HCSR04 sensor
void readHCSR04() {
    verificarDistancia(); // Verify distance and control LED and buzzer
    delay(INTERVALO_LEITURA); // Delay for specified interval
}

// Setup function to initialize pins and timer
void setup() {
    Serial.begin(115200); // Initialize Serial Monitor

    pinMode(ledPin, OUTPUT); // Set LED pin as output
    pinMode(ledPin2, OUTPUT); // Set second LED pin as output
    pinMode(MQ2, INPUT); // Set MQ2 sensor pin as input

    timer.setInterval(1000L, sendUptime); // Set timer to call sendUptime every 1000 ms
}

// Loop function to continuously read sensors
void loop() {
    readMQ2(); // Read MQ2 sensor
    readHCSR04(); // Read HCSR04 sensor
}
