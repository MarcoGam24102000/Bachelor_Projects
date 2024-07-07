

#include "DHT.h"

// Pin connected to the DHT sensor
#define DHTPIN 22
// Define the type of DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

byte ledPin = 18; // LED pin
volatile byte state = LOW; // State variable for LED

byte ledPin2 = 19; // Another LED pin
volatile byte state2 = LOW; // State variable for second LED

int value = 0; // Variable to store luminosity sensor value
double threshold_Illuminance = 0; // Variable to store threshold illuminance
int modeSet = 5; // Default light mode

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

// Function to get threshold illuminance based on light mode
double getThreshold_Illuminance_WithLightMode(int mode) {
    double IlluminanceLimitLevel = 0; // Variable to store illuminance limit level
    
    switch (mode) {
        case 1: // Sunlight
            Serial.println("\nSunlight");
            IlluminanceLimitLevel = 107527;
            break;
        case 2: // Full Daylight
            Serial.println("\nFull Daylight");
            IlluminanceLimitLevel = 10752;
            break;
        case 3: // Overcast Day
            Serial.println("\nOvercast Day");
            IlluminanceLimitLevel = 1075;
            break;
        case 4: // Very Dark Day
            Serial.println("Very Dark Day");
            IlluminanceLimitLevel = 107;
            break;
        case 5: // Twilight
            Serial.println("Twilight");
            IlluminanceLimitLevel = 10.8;
            break;
        case 6: // Deep Twilight
            Serial.println("Deep Twilight");
            IlluminanceLimitLevel = 1.08;
            break;
        case 7: // Full Moon
            Serial.println("Full Moon");
            IlluminanceLimitLevel = 0.108; 
            break;
        case 8: // Quarter Moon
            Serial.println("Quarter Moon");
            IlluminanceLimitLevel = 0.0108; 
            break;
        case 9: // Starlight
            Serial.println("Starlight");
            IlluminanceLimitLevel = 0.0011;
            break;
        case 10: // Overcast Night
            Serial.println("Overcast Night");
            IlluminanceLimitLevel = 0.0001;
            break;
        default: 
            Serial.println("Setting illuminance threshold to 1 lux (approx. Deep Twilight)");
    }
    delay(1000); // Delay for 1 second
    return IlluminanceLimitLevel; // Return illuminance limit level
}

// Function to check temperature and humidity
void checkTempAndHumidity(float humidity, float temperature) {
    if ((humidity >= 60 || humidity <= 40) && (temperature < 25 && temperature > 15)) {
        blinkLED_sevTimes(1); // Blink LED 1 time
    } else if ((temperature >= 25 && temperature <= 15) && (humidity < 60 || humidity > 40)) {
        blinkLED_sevTimes(2); // Blink LED 2 times
    } else if ((temperature >= 25 && temperature <= 15) && (humidity >= 60 || humidity <= 40)) {
        blinkLED_sevTimes(3); // Blink LED 3 times
    } else {
        Serial.println("Ideal Conditions"); // Print ideal conditions message
    }
}

// Function to control buzzer
void buzzer_on(boolean stateBuzzer) {
    if (stateBuzzer) {
        ledcSetup(1, 1000, 8); // Setup PWM channel with 1000 Hz frequency and 8-bit resolution
        ledcAttachPin(12, 1); // Attach pin 12 to PWM channel 1
        ledcWrite(1, 100); // Write PWM value to buzzer
    } else {
        ledcWrite(1, 0); // Turn off buzzer
    }
}

// Function to check luminosity
void checkLuminosity(int value) {
    if (value <= threshold_Illuminance) { // If luminosity is less than or equal to threshold
        buzzer_on(true); // Turn on buzzer
        digitalWrite(ledPin2, HIGH); // Turn on LED
        delay(500); // Delay for 500 ms
        buzzer_on(false); // Turn off buzzer
    } else {
        digitalWrite(ledPin2, LOW); // Turn off LED
    }
}

// Function to read DHT11 sensor data
void readDHT11() {
    delay(2000); // Wait a few seconds between measurements

    float h = dht.readHumidity(); // Read humidity
    float t = dht.readTemperature(); // Read temperature in Celsius
    float f = dht.readTemperature(true); // Read temperature in Fahrenheit

    if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println(F("Failed to read from DHT sensor!")); // Print error message if reading fails
        return;
    }

    float hif = dht.computeHeatIndex(f, h); // Compute heat index in Fahrenheit
    float hic = dht.computeHeatIndex(t, h, false); // Compute heat index in Celsius

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));

    checkTempAndHumidity(h, t); // Check temperature and humidity
}

// Function to read LS06S luminosity sensor
void readLS06S() {
    value = analogRead(21); // Read luminosity sensor value
    Serial.println("Luminosity: ");
    Serial.print(value);
    Serial.print("lux");
    Serial.println("\n");
    delay(1000); // Delay for 1 second

    checkLuminosity(value); // Check luminosity
}

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
  
    pinMode(ledPin, OUTPUT); // Set LED pin as output
    pinMode(ledPin2, OUTPUT); // Set second LED pin as output
    pinMode(21, INPUT); // Set LS06-S sensor pin as input

    dht.begin(); // Initialize DHT sensor

    threshold_Illuminance = getThreshold_Illuminance_WithLightMode(modeSet); // Set illuminance threshold
}

void loop() {
    readLS06S(); // Read LS06S sensor
    readDHT11(); // Read DHT11 sensor
}

