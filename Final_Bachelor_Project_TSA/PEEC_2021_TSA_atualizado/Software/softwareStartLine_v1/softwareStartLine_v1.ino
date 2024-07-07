#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "DHT.h"

#define DHTPIN 26    //23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

float humi = 0;
float t = 0;

int LatTimes1000 = 0;
int LongTimes1000 = 0;

bool getExtraDataAndSend = false;

double LAT = 0;
double LONG = 0;

int humInUnits = 0;

int tempInUnits = 0;

uint8_t broadcastAddress[] = {0x9C,0x9C,0x1F,0xCA,0xBF,0x3C};

//uint8_t broadcastAddress[] = {0x24,0x0A,0xC4,0xF9,0x4A,0x9C};

int perBatESP32_times_by_100 = 0;

int t_life_sec = 0;

int NUM_SERIE = 1;

int ack = 0;

int t_1 = 0;
int t_2 = 0;
int DiffTime = 0;

int brightness = 0;
int fadeAmount = 4;


const int analogInPin = 35;
int adc_value = 0;
double voltage_adc = 0;

double Vcc_voltage_esp32 = 3.7;   //3.7V

double per_bat_esp32 = 0;
int maximum_load = 400;
double TimeSpent = 0;

//-------------------------------------------------------------------
#define ESPNOW_CHANNEL 0
//-------------------------------------------------------------------

bool goReceiveACK = false;

typedef struct struct_message {
  byte a[10];
  byte b[6]; 
} struct_message;


typedef struct struct_message_Bat {
  byte Bat[5];  
  byte timeLifeBat[7];
} struct_message_Bat;


typedef struct struct_GetDHT_data {
  byte Humid[2];
  byte Temper[2];
  byte Lat[8];
  byte Long[9];
} struct_dhtData;

struct_message myData;

struct_message_Bat myData_Bat;

struct_dhtData struct_dht_data; 



bool getAndSend = false;

bool sinc_done = false;

bool sincronize = false;

bool espnowFlag = false;

bool timeAdjustFL = false;

bool ser_gps = false;
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"

//#include "soc/rtc_wdt.h"
//#include "esp_int_wdt.h"
//#include "esp_task_wdt.h"

#include "time.h"

#include <sys/time.h>
#define BLINK_GPIO GPIO_NUM_2      

//-----------------------------------------------------//
#define UART_TOUT_THRESH_DEFAULT   (10)
#define UART_EMPTY_THRESH_DEFAULT  (10)
//-----------------------------------------------------//

static const char *TAG = "uart_events";

//-------------------------------------------------------------------//
int8_t hr_dez = 0;
int8_t hr_un = 0;
int8_t min_dez = 0;
int8_t min_un = 0;
int8_t sec_dez = 0;
int8_t sec_un = 0;
int8_t sub_sec_cent = 0;
int8_t sub_sec_dez = 0;
int8_t sub_sec_un = 0;


//------------------------------------------------------------------------------
int8_t lat_milh = 0;
int8_t lat_cent = 0;
int8_t lat_dez = 0;
int8_t lat_un = 0;
int8_t lat_dec = 0;
int8_t lat_centes = 0;
int8_t lat_miles = 0;
int8_t lat_dec_miles = 0;
double lat_calc = 0;
int8_t NS_ind = 0;

int8_t long_dec_milh = 0;
int8_t long_milh = 0;
int8_t long_cent = 0;
int8_t long_dez = 0;
int8_t long_un = 0;
int8_t long_dec = 0;
int8_t long_centes = 0;
int8_t long_miles = 0;
int8_t long_dec_miles = 0;
double long_calc = 0;
int8_t WE_ind = 0;
//------------------------------------------------------------------------------
int8_t info_dataToSend[16];
//------------------------------------------------------------------------------

int8_t sentence[71];

uint8_t rxbuf[256];  //256

bool sent_cplt = false;

int64_t gps_time = 0;

bool s = false;

static intr_handle_t handle_console;

uint16_t i;

bool infoGPS_cap = false;
bool uartINTR = false;

#define EX_UART_NUM UART_NUM_2
#define PATTERN_CHR_NUM    (3)  

#define RXD2 16
#define TXD2 2

#define U2RXD 16
#define U2TXD 2

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
//-------------------------------------------------------------------//

#define ETC_RMT_INTR_SOURCE 0
#define PUSH_BTN_PIN 19
#define ESP_INTR_FLAG_DEFAULT 0

hw_timer_t * timer = NULL;
int ledPin = 12; //13 //33  //4
//int ledVerde = 4;
volatile byte state = LOW;
bool TLtimerFlag = false;
bool tl_is_running = false;
int intr_alloc_flags;
int8_t blink_n = 0;
//int8_t acenderLED = 0;

int countTimesOnGPIO_interrupt = 0;

#define NUM_PROG_SERIES 10

void protectingCodeForButtonActivationOutOfTime(){  
  countTimesOnGPIO_interrupt++;
                
  if((countTimesOnGPIO_interrupt == 1) || (espnowFlag && sinc_done)){
     init_GPIO();
  }
                
  if(NUM_SERIE == NUM_PROG_SERIES){
     countTimesOnGPIO_interrupt = 0;
  }
}


void getDHT_dataAndSend(){

//  while(isnan(humi) || isnan(t)){
//    readingDHT11();
//  
//    delay(2000);
//  }
  
  int digit_m_Humidity = 0;
  int digit_m_Temperature = 0;

  int digit_m_Position = 0;
  byte digitArray_m_Position[8];

  int digit_m_Position_Long = 0;
  byte digitArray_m_Position_Long[9];


  byte digitArray_m_temperature[2];
  byte digitArray_m_humidity[2];
  

for (byte s = 2; s > 0; --s){
    //Serial.println(newNumber);
    //digit = ((originalNumber/(10^segSetup))%10);
    digit_m_Humidity = humInUnits%10;

    digitArray_m_humidity[s-1] = digit_m_Humidity;
    
    humInUnits /= 10;

    Serial.print("digit: ");
    Serial.println(digit_m_Humidity);

    Serial.print("number: ");
    Serial.println(humInUnits);
 }
 
 for (byte s = 2; s > 0; --s){
    //Serial.println(newNumber);
    //digit = ((originalNumber/(10^segSetup))%10);
    digit_m_Temperature = tempInUnits%10;

    digitArray_m_temperature[s-1] = digit_m_Temperature;
    
    tempInUnits /= 10;

    Serial.print("digit: ");
    Serial.println(digit_m_Temperature);

    Serial.print("number: ");
    Serial.println(tempInUnits);
 }
 
 for(byte x=0; x<2; x++){
     struct_dht_data.Humid[x] = digitArray_m_humidity[x];
  }
  
  for(byte x=0; x<2; x++){
     struct_dht_data.Temper[x] = digitArray_m_temperature[x];
  }

  Serial.printf("\n\nLatitude (x100): %d ", LatTimes1000);
  Serial.printf("\n\nLongitude (x100): %d ", LongTimes1000);

  if(LatTimes1000 != 0 && LongTimes1000 != 0){

    for (byte s = 8; s > 0; --s){
      //Serial.println(newNumber);
      //digit = ((originalNumber/(10^segSetup))%10);
      digit_m_Position = LatTimes1000%10;
  
      digitArray_m_Position[s-1] = digit_m_Position;
      
      LatTimes1000 /= 10;
  
      Serial.print("digit: ");
      Serial.println(digit_m_Position);
  
      Serial.print("number: ");
      Serial.println(LatTimes1000);
   }
   
   for (byte s = 9; s > 0; --s){
      //Serial.println(newNumber);
      //digit = ((originalNumber/(10^segSetup))%10);
      digit_m_Position_Long = LongTimes1000%10;
  
      digitArray_m_Position_Long[s-1] = digit_m_Position_Long;
      
      LatTimes1000 /= 10;
  
      Serial.print("digit: ");
      Serial.println(digit_m_Position_Long);
  
      Serial.print("number: ");
      Serial.println(LatTimes1000);
   }
   
   for(byte x=0; x<8; x++){
       struct_dht_data.Lat[x] = digitArray_m_Position[x];
    }
    
    for(byte x=0; x<9; x++){
       struct_dht_data.Long[x] = digitArray_m_Position_Long[x];
    }
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &struct_dht_data, sizeof(myData_Bat));
     
    if (result == ESP_OK) {
      Serial.println("-----------------------------------Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  
}


void readingDHT11(){
  // Wait a few seconds between measurements.
  //delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humi = dht.readHumidity();
  Serial.println("Read Humidity");
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  Serial.println("Read Temperature");
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  humInUnits = int(humi);

  tempInUnits = int(t);

  //delay(2000);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humi) || isnan(t)) {   //|| isnan(f)
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, humi);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, humi, false);

  Serial.print(F("Humidity: "));
  Serial.print(humi);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
}

static void IRAM_ATTR timerISR(){
  //Serial.println("TIMER");
  state = !state;  
  blink_n++;
  digitalWrite(13, state);
  
  ledcWrite(1, 0);

  //endTimer();
  toggleTimelapse();
  
  if(blink_n<11){
    if(blink_n % 2 != 0){   //se blink_n é ímpar
      ledcWrite(1, 0);   //desliga o buzzer
    }else{
      if(blink_n == 10){   //último apito
        ledcSetup(1, 2500, 8);  //2500   //0 ou 1 
        ledcAttachPin(18, 1);  //0 ou 1
        ledcWrite(1, 100);  //128        

        getAndSend = true;  
        
        //Serial.println("espnow");     
         
      }else{
        ledcSetup(1, 1000, 8);  //2500   //0 ou 1 
        ledcAttachPin(18, 1);  //0 ou 1
        ledcWrite(1, 100);  //128
      }
    }

    toggleTimelapse();  
  }
}

static void IRAM_ATTR gpio_isr_handler()  //void* arg     //static
{
  
  //Serial.println("yes"); 
  
  state = !state;
  digitalWrite(13, state);
  blink_n = 0;  

  //Serial.println("yes4"); 
  
  ledcSetup(1, 1000, 8);  //2500   //0 ou 1 
  ledcAttachPin(18, 1);  //0 ou 1
  ledcWrite(1, 100);  //128
  startTimer();
}

void toggleTimelapse() // websocket callback
{

  //Serial.println("toggleTimelapse()");
  if (tl_is_running) {
    //Serial.println("\nendT");
    endTimer(); 
    tl_is_running = false;          
  }
  else {
    //Serial.println("\nstartT");
    startTimer();           
    tl_is_running  = true;         
  }
}

void startTimer() {
      //Serial.println("START");
      /* Use 1st timer of 4 */
      /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
      timer = timerBegin(0, 80, true);
    
      /* Attach onTimer function to our timer */      
      timerAttachInterrupt(timer, timerISR, true);
    
    
      bool TLtimerFlag = false;
    
      /* Set alarm to call onTimer function every second 1 tick is 1us
      => 1 second is 1000000us */
      /* Repeat the alarm (third parameter) */
      timerAlarmWrite(timer, 500000, true);
    
      /* Start an alarm */
      timerAlarmEnable(timer);
      
      tl_is_running = true;
      //Serial.println("start timer");
}  

void endTimer() { // stop interrupt calls to 'timerISR' and no longer set TLtimerflag
  if (timer)      // 'timer' is a pointer, which is non-zero when the timer is running
    timerEnd(timer);
  timer = NULL;   // & NULL when it's not, hence it serves as a 'timer_is_running' flag
  
}


void init_GPIO(){
   if(sinc_done){
    sinc_done = false;
    gpio_pad_select_gpio(GPIO_NUM_19);  
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_19, gpio_isr_t(gpio_isr_handler), NULL);
    gpio_set_intr_type(GPIO_NUM_19, GPIO_INTR_NEGEDGE);
    //Serial.println("Ok");  
   }
}

//-------------------------------

void setup() {
  Serial.begin(115200);

  //pinMode(ledPin, OUTPUT);

  pinMode(13, OUTPUT);

  ledcSetup(0,1000,8);
  ledcAttachPin(ledPin, 0);
  
   
  pinMode(PUSH_BTN_PIN, INPUT_PULLUP);
  //ledcSetup(1, 1000, 8);  //2500   //0 ou 1 
  //ledcAttachPin(12, 1);  //0 ou 1
  
  //camera_fb_t * fb = NULL;                 //OV2640
  
  


  //--------------------------------------------------------------------------------------------------------------------------------------------------

//  wifi_config_t wifi_config = {
//  .sta = {
//  .channel = ESPNOW_CHANNEL,
//  },
//  };
//
//  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

  //--------------------------------------------------------------------------------------------------------------------------------------------------

  //Serial.println("------------------------YA----------------------");

  

  //Serial.println("-----------------------TEM DE APARECER--------------------------");
 
  WiFi.mode(WIFI_STA);
  //WiFi.mode(WIFI_AP_STA);

  //Serial.println("-----------------------A--------------------------");
 
  //Serial.println();
  //Serial.println(WiFi.macAddress());
                                      //0xFF
  //WiFi.softAP(NULL, NULL, 0, 0, 2);   //ssid = NULL, pwd = NULL -> para considerar o MAC address; canal = canal0; 
                                     //ssid_hidden = 0 (serve também de broadcast); max_connection = 2 (Número máximo de clientes conectados)

  //Serial.println("-----------------------B--------------------------");

  //espnow();

  espnowFlag = true;
 
  //init_GPIO();
 
}


void espnow(){

  //Serial.println("espnow");
  //ConfigSerialInterruptForGPS
  
//  if(NUM_SERIE == 1){
    configSerialInterrupt();  
  // }

//  Serial.println("uartINTR: ");
//  Serial.print(uartINTR);
  
//  if(uartINTR){

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
      }
  
      ser_gps = true;
  
    Serial.println("\n");
    //Serial.print(ser_gps);
    Serial.println("\n");
  
    //timeAdjustParsingGPS();
    
    //Serial.end();
//  }

//  if(infoGPS_cap){
//  
//    Serial.println("-----------------------C--------------------------");
//   
//    // register peer
//    esp_now_peer_info_t peerInfo = {};
//     
//    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//    peerInfo.channel = 0;  
//    peerInfo.encrypt = false;
//  
//    Serial.println("-----------------------D--------------------------");
//           
//    if (esp_now_add_peer(&peerInfo) != ESP_OK){
//      Serial.println("Failed to add peer");
//      return;
//    }
//  
//    Serial.println("-----------------------E--------------------------");
//    
//     //send data
////    int x = 3;
////       
////    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &x, sizeof(int));
////
////      if (result == ESP_OK) {
////        Serial.println("Sent with success");
////      }
////      else {
////        Serial.println("Error sending the data");
////      }
////    
//      Serial.println("-----------------------G--------------------------");
//
//    // send data  
//
//    for(int ind=0; ind<16; ind++){
//
//      Serial.println("\nSending");
//
//      Serial.print(info_dataToSend[ind]);
//      
//       esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &info_dataToSend[ind], sizeof(int8_t));
//  
//      Serial.println("-----------------------F--------------------------");
//         
//      if (result == ESP_OK) {
//        Serial.print(ind + 1);
//        Serial.println(" - Sent with success");
//
////        if(ind == 16){
////          Serial.end();
////        }
//      }
//      else {
//        Serial.println("Error sending the data");
//      }
//    
//      Serial.println("-----------------------G--------------------------");
//
//      return;
//    }   
//
//    infoGPS_cap = false;
//   
//  }  //close infoGPS_cap condition
//    //Serial.end();

  //while(isnan(humi) || isnan(t)){
//    readingDHT11();
//  
//    delay(2000);
//
//    while(isnan(humi) || isnan(t)){
//      readingDHT11();
//  
//      delay(2000);
//    }
    
    
  //}
 
}

//void sendingData(){
//     // send data
//    int x = 3;
//       
//    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &x, sizeof(int));
//  
//    Serial.println("-----------------------F--------------------------");
//       
//    if (result == ESP_OK) {
//      Serial.println("Sent with success");
//    }
//    else {
//      Serial.println("Error sending the data");
//    }
//  
//    Serial.println("-----------------------G--------------------------");
//}


//---------------------------------------------------------------------------------------------------------------------------------------------------GPS-----------------//
int secsToTime(int64_t TimeSeconds){

  //Serial.println("\nSecToTime\n");
  int h_p = TimeSeconds/3600;
  int m_p = (TimeSeconds - h_p*3600)/60;
  int s_p = TimeSeconds - h_p*3600 - m_p*60;

  return s_p;
}


static void IRAM_ATTR uart_intr_handle(void *arg)
{
  uint16_t rx_fifo_len, status;   

  //Serial.println("\nUART INTR HANDLE --------"); 

  status = UART2.int_st.val; // read UART interrupt Status

  Serial.print("Status: ");
  Serial.println(status);

  if(status == 8){
    abort();
    delay(200);
    espnowFlag = true;
  }

  
  uartINTR = true;

  //Serial.println("\n");
  //Serial.print(uartINTR);
  Serial.println("\n");
  
  rx_fifo_len = UART2.status.rxfifo_cnt; // read number of bytes in UART buffer

  i = 0;
      
  while(rx_fifo_len){
     int8_t recv_byte = UART2.fifo.rw_byte;
//     if(recv_byte == (char)'$'){
//       //Serial.println("DOLAR--------------------------------------------------------------------------------------\n");
//       i = 0;
//       Serial.println("\n");
//     }  
     rxbuf[i++] = recv_byte; // read all bytes
     rx_fifo_len--;
      
     //Serial.print((char)recv_byte);  
      
  }   

  rxbuf[i] = '\0';

 //Serial.print("Received BUF: ");
 //Serial.print((char*)rxbuf);
 //Serial.print("\n");

        
  // after reading bytes from buffer clear UART interrupt status
  uart_clear_intr_status(EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

  timeAdjustFL = true;

  
}

void sincronizingRTC_esp32_WithGPS(int hr_dez, int hr_un, int min_dez, int min_un, int sec_dez, int sec_un){
  //First Sinchronization//--------------------------------------------------------------//    

  int Year = 2021;
  int Month = 5;
  int Day = 5;


      //if(((sec_dez + sec_un)%5 == 0)){
        //Serial.println("Synchronized");
        s = true;
        struct tm tm;
        tm.tm_year = 2018 - 1900;
        tm.tm_mon = 10;
        tm.tm_mday = 15;
        //Serial.println("\nHour: ");
        //Serial.print(hr_dez + hr_un);
        //Serial.println("\n");
        tm.tm_hour = hr_dez + hr_un;
        tm.tm_min = min_dez + min_un;
        tm.tm_sec = sec_dez + sec_un; 
        tm.tm_isdst = -1;       
        time_t t = mktime(&tm);

        Serial.printf("Setting time: %s", asctime(&tm));

        struct timeval now = { .tv_sec = t };

        settimeofday(&now, NULL);
        
      //}

     //Serial.println("\n-------------------------------------------------------------------------------------------\n");
    

    //First Sinchronization//--------------------------------------------------------------// 

    //Second Sinchronization//--------------------------------------------------------------//

    struct timeval *tv;

    struct timezone *tz = NULL;

    //Serial.println("\nARRIVE1\n");

    gettimeofday(tv, tz);

    //Serial.println("\nARRIVE2\n");

  //Second Sinchronization//--------------------------------------------------------------//

  sinc_done = true;

  protectingCodeForButtonActivationOutOfTime();
  //init_GPIO();
}



void configSerialInterrupt(){

  //Serial.println("configSerialInterrupt");
  
  if(NUM_SERIE == 1){
  
    esp_log_level_set(TAG, ESP_LOG_INFO);
  
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
      .baud_rate = 9600,   //115200
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
  
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
  
  
    uart_intr_config_t intr_conf = {
      .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M,
      //& !UART_RXFIFO_TOUT_INT_ENA_M,
      // & UART_TX_BRK_DONE_INT_ENA_M,   //makes status = 8 always;
      //    & !UART_RXFIFO_OVF_INT_ENA_M
      //    & !UART_TXFIFO_EMPTY_INT_ENA_M
      //    & !UART_RXFIFO_FULL_INT_ENA_M,
        
      .rx_timeout_thresh = 100,  
      .txfifo_empty_intr_thresh = 10,       
      .rxfifo_full_thresh = 50  //1
    };
    
    ESP_ERROR_CHECK(uart_intr_config(EX_UART_NUM, &intr_conf));
  
    //Set UART pins (using UART2 default pins ie no changes.)
    //ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
  
    // register new UART subroutine
    //Serial.println("\nUART subroutine");
    
    //---------------------------------------//
  }

  // release the pre registered UART handler/subroutine
  ESP_ERROR_CHECK(uart_isr_free(EX_UART_NUM));
  
  ESP_ERROR_CHECK(uart_isr_register(EX_UART_NUM,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console)); 
  
  
  // enable RX interrupt
  ESP_ERROR_CHECK(uart_enable_rx_intr(EX_UART_NUM));
}


void timeAdjustParsingGPS(){

  //Serial.println("\nHave to get it");

  for(int j = 0; j<71; j++){      
 
     if(rxbuf[j] != (char)'*'){
        sentence[j] = rxbuf[j];
     }        
  }    
  
  if(((char)sentence[7] >= '0') && ((char)sentence[7] <= '9')){
     hr_dez = 10*(int)((char)sentence[7] - '0');      
  }
  if(((char)sentence[8] >= '0') && ((char)sentence[8] <= '9')){
     hr_un = (int)((char)sentence[8] - '0');      
  }
  if(((char)sentence[9] >= '0') && ((char)sentence[9] <= '9')){
     min_dez = 10*(int)((char)sentence[9] - '0');      
  }
  if(((char)sentence[10] >= '0') && ((char)sentence[10] <= '9')){
     min_un = (int)((char)sentence[10] - '0');      
  }
  if(((char)sentence[11] >= '0') && ((char)sentence[11] <= '9')){
     sec_dez = 10*(int)((char)sentence[11] - '0');     
  }
  if(((char)sentence[12] >= '0') && ((char)sentence[12] <= '9')){
     sec_un = (int)((char)sentence[12] - '0');      
  }
  //
  if(((char)sentence[14] >= '0') && ((char)sentence[14] <= '9')){
     sub_sec_cent = 100*(int)((char)sentence[14] - '0');      
  }
  //
  if(((char)sentence[15] >= '0') && ((char)sentence[15] <= '9')){
     sub_sec_dez = 10*(int)((char)sentence[15] - '0');      
  }
  //
  if(((char)sentence[16] >= '0') && ((char)sentence[16] <= '9')){
     sub_sec_un = (int)((char)sentence[16] - '0');     
  }

  
//
   LAT = ((((int)((char)sentence[18] - '0'))*10 + ((int)((char)sentence[19] - '0')))*3600 + (((int)((char)sentence[20] - '0'))*10 + ((int)((char)sentence[21] - '0')))*60 + 
   (((int)((char)sentence[23] - '0'))*10 + ((int)((char)sentence[24] - '0'))) + (((int)((char)sentence[25] - '0'))*10 + ((int)((char)sentence[26] - '0')))/100)/3600;

   if(LAT != 0){
     if((char)sentence[28] == 'S'){
      Serial.println("South");
        LAT = -LAT;
     }else if((char)sentence[28] != 'N'){
        Serial.println("Neither");
        LAT = 0;
     }  
   }  

   LONG = ((((int)((char)sentence[30] - '0'))*100 + ((int)((char)sentence[31] - '0'))*10 + ((int)((char)sentence[32] - '0')))*3600 + 
   (((int)((char)sentence[33] - '0'))*10 + ((int)((char)sentence[34] - '0')))*60 + 
   (((int)((char)sentence[36] - '0'))*10 + ((int)((char)sentence[37] - '0'))) + (((int)((char)sentence[38] - '0'))*10 + ((int)((char)sentence[39] - '0')))/100)/3600;

   if(LONG != 0){
      if((char)sentence[41] == 'W'){
        LONG = -LONG;
      }else if((char)sentence[41] != 'E'){
        Serial.println("Neither");
        LONG = 0;
      }
   }
   

   LatTimes1000 = LAT*1000;
   LongTimes1000 = LONG*1000;
   
//
//  Serial.println("ARRIVING HERE");
//  Serial.print(hr_dez);
//  Serial.println("\n");
//  Serial.print(hr_un);
//  Serial.println("\n");
//  Serial.print(min_dez);
//  Serial.println("\n");
//  Serial.print(min_un);
//  Serial.println("\n");
//
//  Serial.printf("\n\n\n-------------------Lat And Long: (%u, %u)----------------------\n\n\n", LAT, LONG);


  //----------------------------------------------------------------------------------------------------------------------------------------------//

  
  //----------------------------------------------------------------------------------------------------------------------------------------------//

  
  //
  if(sent_cplt){   //sent_cplt && rightCS
    //rightCS = false;   
    sent_cplt = false;
    i = 0;
    gps_time = ((hr_dez + hr_un) * 3600 + (min_dez + min_un) * 60 + sec_dez + sec_un)*100 + (sub_sec_cent + sub_sec_dez + sub_sec_un)/1000 ;  //

    //Serial.println("\nThere");

    digitalWrite(13, HIGH);

    info_dataToSend[0] = hr_dez + hr_un;
    info_dataToSend[1] = min_dez + min_un;
    info_dataToSend[2] = sec_dez + sec_un;
    info_dataToSend[3] = sub_sec_cent + sub_sec_dez;
    info_dataToSend[4] = sub_sec_un;

    info_dataToSend[5] = lat_milh*10 + lat_cent;
    info_dataToSend[6] = lat_dez*10 + lat_un;
    info_dataToSend[7] = lat_dec*10 + lat_centes;
    info_dataToSend[8] = lat_miles*10 + lat_dec_miles;
    info_dataToSend[9] = NS_ind;

    info_dataToSend[10] = long_dec_milh;
    info_dataToSend[11] = long_milh*10 + long_cent;
    info_dataToSend[12] = long_dez*10 + long_un;
    info_dataToSend[13] = long_dec*10 + long_centes;
    info_dataToSend[14] = long_miles*10 + long_dec_miles;
    info_dataToSend[15] = WE_ind;

    //Serial.println("AFTER THAT");
    
    

         
  
    Serial.printf("\nGPS Time: %u", gps_time); 

    sincronize = true;
  
  
//    hr_dez = 0; 
//    hr_un = 0; 
//    min_dez = 0; 
//    min_un = 0; 
//    sec_dez = 0; 
//    sec_un = 0; 
//    sub_sec_cent = 0; 
//    sub_sec_dez = 0; 
//    sub_sec_un = 0;
  //    //------------------------------------------------------//    
  //    

  //infoGPS_cap = true;
  }else if(rxbuf[0] == '$' && rxbuf[1] == 'G' && (rxbuf[2] == 'P' || rxbuf[2] == 'N') && rxbuf[3] == 'G' && rxbuf[4] == 'G' && rxbuf[5] == 'A' && rxbuf[6] == ','){
    //Serial.println("----------------------SOLUTION NUMBER 1-----------------------------");
    sent_cplt = true;
    i = 0;
  } 

}
//---------------------------------------------------------------------------------------------------------------------------------------------------GPS-----------------//

void getTimeAndSend(){

//  Try 1

//  Serial.println("\ngetTimeAndSend - entering\n");
//  
//  struct timeval *tv;
//
//  struct timezone *tz = NULL;
//
//  Serial.println("\ngetTimeAndSend 1\n");
//
//  gettimeofday(tv, tz);
//
//  Serial.print((suseconds_t)tv->tv_usec);
//
//  Serial.println("\ngetTimeAndSend 2\n");
//  
//  int secs = secsToTime(tv->tv_sec);
//
//  Serial.println("\ngetTimeAndSend 3\n");
//
//  Serial.println("\nSeconds: ");
//  Serial.print(secs);
//
//  Serial.println("\n");

  //Try 2

//  Serial.println("\n--------------------------   1  --------------------\n");
//  time_t now;
//  Serial.println("\n--------------------------   2  --------------------\n");
//  //struct tm * timeinfo;
//  Serial.println("\n--------------------------   3  --------------------\n");
//  time(&now);
//  Serial.println("\n--------------------------   4  --------------------\n");
//  Serial.print(localtime(&now));
//  //timeinfo = localtime(&now);
//  //Serial.println("\n---------------------------Hours:  ");
//  Serial.println((localtime(&now))->tm_hour);
//  delay(1000);

  struct timeval tvTime;
  
  gettimeofday(&tvTime, NULL);
  
  int iTotal_seconds = tvTime.tv_sec;

  Serial.println("\n");
  Serial.printf("Segundos: ");
  Serial.print(tvTime.tv_sec);
  Serial.println("To Char: ");
  Serial.print(char(tvTime.tv_sec));
  Serial.println("\n");
  Serial.printf("Microsegundos: ");
  Serial.print(tvTime.tv_usec);
   
  Serial.println("\n");
  
//  struct tm *ptm = localtime((const time_t *) & iTotal_seconds);
//  
//  int iHour = ptm->tm_hour;;
//  int iMinute = ptm->tm_min;
//  int iSecond = ptm->tm_sec;
//  int iMilliSec = tvTime.tv_usec / 1000;
//  int iMicroSec = tvTime.tv_usec;
//
//  Serial.printf("\nTime: %d:%d:%d - Miliseconds: %d", iHour, iMinute, iSecond, iMilliSec);

  int secondsNumberSince1970 = tvTime.tv_sec;
  int microssecondsOfNumber = tvTime.tv_usec;

  byte digitArray[10];  

  int digit = 0;
  int digit_m = 0;  
  byte digitArray_m[6]; 

  
  for (byte s = 10; s > 0; --s){
    //Serial.println(newNumber);
    //digit = ((originalNumber/(10^segSetup))%10);
    digit = secondsNumberSince1970%10;

    digitArray[s-1] = digit;
    
    secondsNumberSince1970 /= 10;

    Serial.print("digit: ");
    Serial.println(digit);

    Serial.print("number: ");
    Serial.println(secondsNumberSince1970);
  }

  for(byte x=0; x<10; x++){
     myData.a[x] = digitArray[x];
  }

  for (byte s = 6; s > 0; --s){
    //Serial.println(newNumber);
    //digit = ((originalNumber/(10^segSetup))%10);
    digit_m = microssecondsOfNumber%10;

    digitArray_m[s-1] = digit_m;
    
    microssecondsOfNumber /= 10;

    Serial.print("digit: ");
    Serial.println(digit_m);

    Serial.print("number: ");
    Serial.println(microssecondsOfNumber);
  }
  
  for(byte x=0; x<6; x++){
     myData.b[x] = digitArray_m[x];
  }
  
  if(NUM_SERIE == 1){
  
  // register peer
    esp_now_peer_info_t peerInfo = {};
    
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    //Serial.println("-----------------------D--------------------------");
              
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
     
    //Serial.println("-----------------------E--------------------------");
  }
  
  digitalWrite(13, LOW);

  for(int count=0; count<sizeof(myData); count++){
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(100);    
  }

  

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  t_1 = esp_timer_get_time();

  goReceiveACK = true;
  
  
}

void onDataRecvACK(const uint8_t * mac, const uint8_t *incomingData, int len){
 
  
  memcpy(&ack, incomingData, sizeof(int));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("ACK: ");
  Serial.print(ack);  

  digitalWrite(13, LOW);

  for(int count=0; count<len; count++){
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(100);    
  }
}

void receivingACK(){
  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecvACK);
  
}

int adc_reading(){
  adc_value = analogRead(analogInPin);
  // print the readings in the Serial Monitor
  //Serial.print("sensor = ");
  //Serial.print(adc_value);
  
  delay(1000);

  return adc_value;
}

double getVoltageADC(double Vcc_voltage, int Value_ADC){
  double voltage_adc = 0;

  voltage_adc = Value_ADC*(Vcc_voltage/(2^12-1));

  return voltage_adc;
}

double CalculatePerBattery(int delta_t, double value_voltage, int maximum_load){
  double per_bat = 0;
  double per_bat_rem = 0;
  double divider = 0;
  double cur_load = 0;

  divider = 1-delta_t/3600;
  cur_load = value_voltage/divider;

  per_bat_rem = (cur_load/maximum_load)*100;
  per_bat = 100 - per_bat_rem;
  
  return per_bat;
}

double CalculateTimeLife(int c_max, double v_adc, double delta_t){
  double timeLifeInSeconds = 0;
  double c_atual = 0;

  c_atual = v_adc/(1-delta_t/3600);

  timeLifeInSeconds = 3600*(c_max/c_atual);

  return timeLifeInSeconds;
}

void TurnOnOffLEDSlowly_BatteryLife(){    // 256/fadeAmont = 64 -> 64*(ligar + desligar) = 64*2 = 128
  for(int i= 0; i<128; i++){
      //Serial.println("--------------------OKL-----------------------");
      ledcWrite(0, brightness); // set the brightness of the LED
      // change the brightness for next time through the loop:
      //Serial.println("--------------------OKL1-----------------------");
      brightness = brightness + fadeAmount;
      // reverse the direction of the fading at the ends of the fade:
      if (brightness <= 0 || brightness >= 255) {
          fadeAmount = -fadeAmount;
      }
      // wait for 30 milliseconds to see the dimming effect
      delay(50);
  }
} 

void PerBat_show(double per_batToShowByLED){
  if(per_batToShowByLED >=0 && per_batToShowByLED <25){
    for(int countTimes=0; countTimes<4; countTimes++){
      TurnOnOffLEDSlowly_BatteryLife();
    }
  }else if(per_batToShowByLED >=25 && per_batToShowByLED <50){
    for(int countTimes=0; countTimes<3; countTimes++){
      TurnOnOffLEDSlowly_BatteryLife();
    }
  }else if(per_batToShowByLED >=50 && per_batToShowByLED <75){
    for(int countTimes=0; countTimes<2; countTimes++){
      TurnOnOffLEDSlowly_BatteryLife();
    }
  }else if(per_batToShowByLED >=75 && per_batToShowByLED <=100){
    for(int countTimes=0; countTimes<1; countTimes++){
      TurnOnOffLEDSlowly_BatteryLife();
    }
  }
  delay(4000);
}


void calculatingDiffTime(){

  byte digitArray_m_Bat[5];

  int digit_m_Bat = 0;

  byte digitArray_m_Bat_TimeLife[7];

  int digit_m_Bat_TimeLife = 0;


  DiffTime = abs(t_2-t_1);

  Serial.printf("\n\n-----TIME OF SERIE RECEIVED AT SENDING POINT: %d", DiffTime);  

  voltage_adc = getVoltageADC(Vcc_voltage_esp32, adc_reading());

  per_bat_esp32 = CalculatePerBattery(DiffTime, voltage_adc, maximum_load);

  PerBat_show(per_bat_esp32);

  t_life_sec = CalculateTimeLife(maximum_load, voltage_adc, DiffTime);

  Serial.println("\n\n\nA percentagem de bateria do dispositivo presente na linha de partida é de: ");
  Serial.print(per_bat_esp32);
  Serial.println("\n\n");

  perBatESP32_times_by_100 = per_bat_esp32*100;

  for(byte s=5; s>0; s--){
    digit_m_Bat = perBatESP32_times_by_100%10;
    digitArray_m_Bat[s-1] = digit_m_Bat;

    perBatESP32_times_by_100 /= 10;    
  }

  for(byte x=0; x<5; x++){
    myData_Bat.Bat[x] = digitArray_m_Bat[x];
  }

  for(byte s=7; s>0; s--){
    digit_m_Bat_TimeLife = t_life_sec%10;
    digitArray_m_Bat_TimeLife[s-1] = digit_m_Bat_TimeLife;

    t_life_sec /= 10;    
  }

  for(byte x=0; x<7; x++){
    myData_Bat.timeLifeBat[x] = digitArray_m_Bat_TimeLife[x];
  }

  delay(200);

//  WiFi.mode(WIFI_STA);
//
//  if (esp_now_init() != ESP_OK){
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }
//
//  // register peer
//  esp_now_peer_info_t peerInfoBat = {};
//  
//  memcpy(peerInfoBat.peer_addr, broadcastAddress, 6);
//  peerInfoBat.channel = 0;  
//  //peerInfoBat.ifidx = WIFI_IF_STA;
//  peerInfoBat.encrypt = false;
//  Serial.println("-----------------------D--------------------------");
//            
//  if (esp_now_add_peer(&peerInfoBat) != ESP_OK){
//    Serial.println("Sending battery - Failed to add peer");
//    return;
//  }
   
  //Serial.println("-----------------------E--------------------------");

  digitalWrite(13, LOW);

  for(int count=0; count<sizeof(myData_Bat); count++){
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(100);    
  }

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData_Bat, sizeof(myData_Bat));
   
  if (result == ESP_OK) {
    Serial.println("-----------------------------------Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(100);

  //abort();

  //getExtraDataAndSend = true;

  delay(100);  

  espnowFlag = true;
  NUM_SERIE++;

  if(NUM_SERIE < (NUM_PROG_SERIES + 2)){
    Serial.printf("\n\n\nNEXT SERIE !!! - READY FOR %dª serie\n\n\n", NUM_SERIE);
  }else{
    Serial.println("\n\nNUMBER OF SERIES OVERLOADED\n\n");
    espnowFlag = false;
  }
  
}


void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("\nNOT IN");

  //Serial.print(ser_gps);
  
//  if(ser_gps){
//    Serial.println("\nTRUE");
//    timeAdjustParsingGPS();
//    ser_gps = false;
//  }

  if(espnowFlag){
    espnowFlag = false;
    espnow();
  }

  if(timeAdjustFL){
    timeAdjustFL = false;
    timeAdjustParsingGPS();
  }

  if(sincronize){
    sincronize = false;
    sincronizingRTC_esp32_WithGPS(hr_dez, hr_un, min_dez, min_un, sec_dez, sec_un);
  }

//  if(sinc_done){
//    sinc_done = false;
//    init_GPIO();
//  }

  if(getAndSend){
    getAndSend = false;
    getTimeAndSend();
  }

  if(goReceiveACK){
    goReceiveACK = false;
    receivingACK();
  }

  if(ack == 1){
    //Serial.println("\nIt´s all right\n");
    ack = 0;
    t_2 = esp_timer_get_time();
    calculatingDiffTime();
  }
 
  if(getExtraDataAndSend){
    getExtraDataAndSend = false;
    getDHT_dataAndSend();
  }

}
