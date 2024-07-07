#include "DHT.h"                        // Library for DHT temperature and humidity sensor
#include <Ultrasonic.h>                  // Library for HC-SR04 ultrasonic sensor
#include <Wire.h>                        // I2C communication library
#include <LiquidCrystal_I2C.h>           // Library for I2C LCD

// Sensor pins
#define DHTPIN 4                        // Pin connected to the DHT sensor
#define DHTTYPE DHT11                   // Type of DHT sensor (DHT11 in this case)
#define PIN_TRIGGER 12                  // Pin connected to the HC-SR04 trigger
#define PIN_ECHO 13                     // Pin connected to the HC-SR04 echo
#define LS06Spin 32                     // Pin connected to LS06S light sensor

// Constants
#define INTERVALO_LEITURA 250           // Reading interval in milliseconds

// LCD configuration
#define LCD_COLUMNS 16                  // Number of columns for LCD
#define LCD_ROWS 2                      // Number of rows for LCD
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);  // LCD object with I2C address 0x27

// Struct to store DHT data
typedef struct {
  float temper;                        // Temperature
  float humid;                         // Humidity
} xDataDHT;

// Sensor objects
DHT dht(DHTPIN, DHTTYPE);               // DHT sensor object
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO);  // HC-SR04 ultrasonic sensor object

// Function prototypes
static void DHT11_task(void *pvParameters);
static void LS06S_task(void *pvParameters);
static void buzzer_task(void *pvParameters);
static void led_PWM_task(void *pvParameters);
static void lcd_task(void *pvParameters);
static void adc_task(void *pvParameters);
static void HCSR04_task(void *pvParameters);
static void IRAM_ATTR ExternalInterruptHandler(void);

// FreeRTOS objects
QueueHandle_t xLS06SQueue;              // Queue for LS06S sensor data
QueueHandle_t xBuzzerQueue;             // Queue for buzzer control
QueueHandle_t xADCQueue;                // Queue for ADC data
SemaphoreHandle_t xMutex;               // Mutex semaphore for mutual exclusion
SemaphoreHandle_t xMutexBuzzerTask;     // Mutex semaphore for buzzer task

// LED pin definitions
#define LED_PROX 23                     // Proximity LED pin
#define LED_LUX 25                      // Luminosity LED pin
#define LED_PWM 18                      // PWM LED pin
#define LED_HUM 19                      // Humidity LED pin
#define LED_TEMP 5                      // Temperature LED pin

// Function to scroll text on LCD
void scrollText(int row, String message, int delayTime, int lcdColumns) {
  for (int i = 0; i < lcdColumns; i++) {
    message = " " + message;
  }
  message = message + " ";
  for (int pos = 0; pos < message.length(); pos++) {
    lcd.setCursor(0, row);
    lcd.print(message.substring(pos, pos + lcdColumns));
    delay(delayTime);
  }
}

// Function to control the proximity LED and buzzer
void ledBuzProx() {
  buzzer_on(true, 1000);                // Turn buzzer on with 1kHz frequency
  digitalWrite(LED_PROX, !digitalRead(LED_PROX));  // Toggle proximity LED
  delay(500);
  buzzer_on(false, 1000);               // Turn buzzer off
  digitalWrite(LED_PROX, !digitalRead(LED_PROX));  // Toggle proximity LED
}

// Function to determine number of times based on voltage
int getSelectedNumTimesWithV(double voltage) {
  int numTimes = 0;
  if (voltage >= 0 && voltage < 1.66) {
    numTimes = 1;
  } else if (voltage >= 1.66 && voltage < 3.33) {
    numTimes = 2;
  } else if (voltage >= 3.33 && voltage <= 5) {
    numTimes = 3;
  }
  return numTimes;
}

// Function to present information on LCD based on selected mode
void presentInfLCD(int timesPWM, float temperature, float humidity, int luminosity) {
  byte graus_symb[8] = {                // Custom character for degree symbol
    0b00000,
    0b01110,
    0b01010,
    0b01110,
    0b00000,
    0b00000,
    0b00000,
    0b00000
  };

  lcd.clear();
  lcd.setCursor(0, 0);

  if (timesPWM == 1 && humidity != 0) {
    lcd.print("Humidity: " + String(humidity) + "%");
  } else if (timesPWM == 2 && temperature != 0) {
    lcd.createChar(0, graus_symb);      // Load degree symbol into LCD's memory
    scrollText(0, "Temperature: " + String(temperature), 250, 16);
    lcd.setCursor(0, 0);
    lcd.write(0);                       // Print degree symbol
    lcd.print("C");
  } else if (timesPWM == 3) {
    scrollText(0, "Luminosity: " + String(luminosity) + " lux", 250, 16);
  }
}

// Function to slowly increase and decrease LED brightness
void slowlyLEDSelectedNumTimes(int selectedNumTimes) {
  int brightness = 0;
  int fadeAmount = 8;

  for (int x = 0; x < selectedNumTimes; x++) {
    for (int num = 0; num < 64; num++) {
      ledcWrite(0, brightness);         // Set PWM LED brightness
      brightness = brightness + fadeAmount;
      if (brightness <= 0 || brightness >= 255) {
        fadeAmount = -fadeAmount;        // Reverse fading direction
      }
      delay(30);
    }
  }
}

// Function to get the threshold illuminance level based on mode
double getThreshold_Illuminance_WithLightMode(int mode) {
  double IlluminanceLimitLevel = 0;

  switch (mode) {
    case 1:  // Sunlight
      IlluminanceLimitLevel = 107527;
      break;
    case 2:  // Full Daylight
      IlluminanceLimitLevel = 10752;
      break;
    case 3:  // Overcast Day
      IlluminanceLimitLevel = 1075;
      break;
    case 4:  // Very Dark Day
      IlluminanceLimitLevel = 107;
      break;
    case 5:  // Twilight
      IlluminanceLimitLevel = 10.8;
      break;
    case 6:  // Deep Twilight
      IlluminanceLimitLevel = 1.08;
      break;
    case 7:  // Full Moon
      IlluminanceLimitLevel = 0.108;
      break;
    case 8:  // Quarter Moon
      IlluminanceLimitLevel = 0.0108;
      break;
    case 9:  // Starlight
      IlluminanceLimitLevel = 0.0011;
      break;
    case 10: // Overcast Night
      IlluminanceLimitLevel = 0.0001;
      break;
    default:
      IlluminanceLimitLevel = 1;        // Default to Deep Twilight
  }

  return IlluminanceLimitLevel;
}

// Function to check temperature and humidity and control LEDs accordingly
void checkTempAndHumidity(float humidity, float temperature) {
  if ((humidity >= 60 || humidity <= 40) && (temperature < 25 && temperature > 15)) {
    digitalWrite(LED_HUM, HIGH);        // Turn on humidity LED
    digitalWrite(LED_TEMP, LOW);        // Turn off temperature LED
  } else if ((temperature >= 25 && temperature <= 15) && (humidity < 60 || humidity > 40)) {
    digitalWrite(LED_HUM, LOW);         // Turn off humidity LED
    digitalWrite(LED_TEMP, HIGH);       // Turn on temperature LED
  } else if ((temperature >= 25 && temperature <= 15) && (humidity >= 60 || humidity <= 40)) {
    digitalWrite(LED_HUM, HIGH);        // Turn on humidity LED
    digitalWrite(LED_TEMP, HIGH);       // Turn on temperature LED
  } else {
    digitalWrite(LED_HUM, LOW);         // Turn off humidity LED
    digitalWrite(LED_TEMP, LOW);        // Turn off temperature LED
  }
}

// Function to control the buzzer state and frequency
void buzzer_on(boolean stateBuzzer, int freq) {
  if (stateBuzzer) {
    ledcSetup(1, freq, 8);              // Setup buzzer PWM
    ledcAttachPin(26, 1);               // Attach buzzer pin
    ledcWrite(1, 100);                  // Set buzzer PWM duty cycle
  } else {
    ledcWrite(1, 0);                    // Turn off buzzer
  }
}

// Function to check luminosity and control LEDs and buzzer
void checkLuminosity(int value) {
  if (value <= getThreshold_Illuminance_WithLightMode(MODE_SET)) {
    buzzer_on(true, 2500);              // Turn buzzer on with 2.5kHz frequency
    digitalWrite(LED_LUX, HIGH);        // Turn on luminosity LED
    delay(700);
    buzzer_on(false, 2500);             // Turn buzzer off
  } else {
    digitalWrite(LED_LUX, LOW);         // Turn off luminosity LED
  }
}

// Function to read the LS06S sensor and return luminosity
int readLS06S() {
  int value = analogRead(LS06Spin);     // Read LS06S sensor
  return value;
}

// Function to read DHT11 sensor data into a struct
void readingDHT11(xDataDHT *dataDHT_11) {
  float h = dht.readHumidity();         // Read humidity from DHT sensor
  float t = dht.readTemperature();      // Read temperature from DHT sensor

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));  // Error message if sensor reading failed
    return;
  }

  dataDHT_11->humid = h;                // Store humidity in struct
  dataDHT_11->temper = t;               // Store temperature in struct
}

// Function to get distance measured by HC-SR04 sensor
int getDistance() {
  long microsec = ultrasonic.timing();   // Measure pulse width from HC-SR04
  int distanciaCM = ultrasonic.convert(microsec, Ultrasonic::CM);  // Convert pulse width to distance in cm
  return distanciaCM;
}

// Function to verify and optionally print distance
unsigned int verificarDistancia(bool presentDistance) {
  unsigned int distancia = getDistance();  // Get distance from HC-SR04 sensor

  if (presentDistance) {
    Serial.print("Distance: ");           // Print distance to serial monitor
    Serial.print(distancia);
    Serial.print("cm");
    Serial.println("\n");
  }

  return distancia;
}

// Function to read HC-SR04 sensor and return distance
unsigned int readHCSR04(bool presentDistance) {
  unsigned int distanceUltra = verificarDistancia(presentDistance);  // Verify and get distance

  delay(INTERVALO_LEITURA);             // Delay between readings

  return distanceUltra;
}

// Task to handle DHT11 sensor data
static void DHT11_task(void *pvParameters) {
  xDataDHT dataDHT_11;                  // Data struct for DHT11 sensor
  for (;;) {
    readingDHT11(&dataDHT_11);          // Read DHT11 sensor data
    xQueueSend(xADCQueue, &dataDHT_11, portMAX_DELAY);  // Send data to ADC queue
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle LS06S sensor data
static void LS06S_task(void *pvParameters) {
  int LS06S_Data;                       // Data variable for LS06S sensor
  for (;;) {
    LS06S_Data = readLS06S();           // Read LS06S sensor data
    xQueueSend(xLS06SQueue, &LS06S_Data, portMAX_DELAY);  // Send data to LS06S queue
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle buzzer control
static void buzzer_task(void *pvParameters) {
  int buzzerFrequency;                  // Buzzer frequency variable
  for (;;) {
    xQueueReceive(xBuzzerQueue, &buzzerFrequency, portMAX_DELAY);  // Receive frequency from queue
    buzzer_on(true, buzzerFrequency);   // Activate buzzer with received frequency
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle PWM LED control
static void led_PWM_task(void *pvParameters) {
  int selectedNumTimes;                 // Selected number of times variable
  for (;;) {
    xQueueReceive(xADCQueue, &selectedNumTimes, portMAX_DELAY);  // Receive times from queue
    slowlyLEDSelectedNumTimes(selectedNumTimes);  // Execute PWM LED function
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle LCD display
static void lcd_task(void *pvParameters) {
  xDataDHT dataDHT_11;                  // Data struct for DHT11 sensor
  int LS06S_Data;                       // Data variable for LS06S sensor
  int distanceUltra;                    // Distance variable for HC-SR04 sensor
  for (;;) {
    xQueueReceive(xADCQueue, &dataDHT_11, portMAX_DELAY);  // Receive DHT data from queue
    xQueueReceive(xLS06SQueue, &LS06S_Data, portMAX_DELAY);  // Receive LS06S data from queue
    distanceUltra = readHCSR04(false);  // Read HC-SR04 sensor data
    presentInfLCD(MODE_SET, dataDHT_11.temper, dataDHT_11.humid, LS06S_Data);  // Display data on LCD
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle ADC data
static void adc_task(void *pvParameters) {
  for (;;) {
    analogRead(0);                      // Read ADC
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Task to handle HC-SR04 sensor data
static void HCSR04_task(void *pvParameters) {
  for (;;) {
    readHCSR04(true);                   // Read and print HC-SR04 sensor data
    vTaskDelay(pdMS_TO_TICKS(1000));    // Delay 1 second
  }
}

// Interrupt handler for external interrupts
static void IRAM_ATTR ExternalInterruptHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xMutex, &xHigherPriorityTaskWoken);  // Give mutex semaphore from ISR
  xSemaphoreGiveFromISR(xMutexBuzzerTask, &xHigherPriorityTaskWoken);  // Give buzzer mutex semaphore from ISR
}

// Setup function
void setup() {
  Serial.begin(9600);                   // Start serial communication
  pinMode(LED_PROX, OUTPUT);            // Set proximity LED pin as output
  pinMode(LED_LUX, OUTPUT);             // Set luminosity LED pin as output
  pinMode(LED_PWM, OUTPUT);             // Set PWM LED pin as output
  pinMode(LED_HUM, OUTPUT);             // Set humidity LED pin as output
  pinMode(LED_TEMP, OUTPUT);            // Set temperature LED pin as output
  pinMode(PIN_TRIGGER, OUTPUT);         // Set HC-SR04 trigger pin as output
  pinMode(PIN_ECHO, INPUT);             // Set HC-SR04 echo pin as input
  pinMode(LS06Spin, INPUT);             // Set LS06S sensor pin as input
  dht.begin();                          // Initialize DHT sensor
  lcd.init();                           // Initialize LCD
  lcd.backlight();                      // Turn on LCD backlight
  lcd.home();                           // Set cursor to home position
  ledcSetup(0, 5000, 8);                // Setup PWM channel 0 with 5kHz frequency and 8-bit resolution
  ledcAttachPin(18, 0);                 // Attach PWM pin to channel 0
  attachInterrupt(digitalPinToInterrupt(14), ExternalInterruptHandler, FALLING);  // Attach interrupt handler
  xMutex = xSemaphoreCreateBinary();    // Create mutex semaphore
  xMutexBuzzerTask = xSemaphoreCreateBinary();  // Create buzzer mutex semaphore
  xADCQueue = xQueueCreate(5, sizeof(int));  // Create ADC queue
  xLS06SQueue = xQueueCreate(5, sizeof(int));  // Create LS06S queue
  xBuzzerQueue = xQueueCreate(5, sizeof(int));  // Create buzzer queue
  xTaskCreatePinnedToCore(DHT11_task, "DHT11_task", 2048, NULL, 1, NULL, 0);  // Create DHT11 task on core 0
  xTaskCreatePinnedToCore(LS06S_task, "LS06S_task", 2048, NULL, 1, NULL, 1);  // Create LS06S task on core 1
  xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 2048, NULL, 1, NULL, 0);  // Create buzzer task on core 0
  xTaskCreatePinnedToCore(led_PWM_task, "led_PWM_task", 2048, NULL, 1, NULL, 1);  // Create PWM LED task on core 1
  xTaskCreatePinnedToCore(lcd_task, "lcd_task", 2048, NULL, 1, NULL, 1);  // Create LCD task on core 1
  xTaskCreatePinnedToCore(adc_task, "adc_task", 2048, NULL, 1, NULL, 1);  // Create ADC task on core 1
  xTaskCreatePinnedToCore(HCSR04_task, "HCSR04_task", 2048, NULL, 1, NULL, 1);  // Create HC-SR04 task on core 1
  vTaskStartScheduler();                // Start FreeRTOS scheduler
}

// Loop function
void loop() {
  // Empty as all tasks are managed by FreeRTOS scheduler
}
