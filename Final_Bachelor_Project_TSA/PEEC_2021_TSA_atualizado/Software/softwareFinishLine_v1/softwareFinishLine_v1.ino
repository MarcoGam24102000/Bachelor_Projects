#define NUM_PROG_SERIES 10

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#include <U8g2lib.h>

U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(  /* rotation zero */ U8G2_R0, 15 , 14, U8X8_PIN_NONE);

float timesSeries[NUM_PROG_SERIES];

bool BatInfoFunction = false;

bool nextRace = true;

bool id_bat = false;

bool led_dataRecv = false;

int idMsgToPrint = 0;

char timeSerieOLED[2];
char timeSerieOLED_subsec[2];
char temp_v[2];
char hum_v[2];
char dist_v[3];

bool getRestBat = false;

double distH = 0;

int NUM_SERIE = 1;

int LatEndingPoint = 0;
int LongEndingPoint = 0;

double RealdistSL = 0;

char battery[3];

double BatteryOfSystem = 0;
//-----------------------------------------------------------------------//
int R = 6371000; //Earth´s radius
double fi_1, fi_2, lam_1, lam_2;

double delta_fi, delta_lam;

double a, c, d;
//-----------------------------------------------------------------------//


double getRealDistanceStraightLine(double d){
  double d_r = 0;
  
  if(d == 2 || d == 5 || d == 7 || d == 9){
    d_r = 10*(d+1);
  }else if(d == 1 || d == 3 || d == 4 || d == 6 || d == 8 || d == 10 || d == 11){
    d_r = 10*d;
  }

  return d_r;
}

void calculateEstimatedSpeed(int distance, double timeofSerie){
  double speedS = 0;

  speedS = distance/timeofSerie;

  Serial.printf("\n\nRegisting a speed in that serie of: %u\n\n", speedS);  
}

double calculateDistanceBetGPSs_HaversineFormula(int latGPS_startLine, int longGPS_startLine, int latGPS_endLine, int longGPS_endLine){
  fi_1 = latGPS_startLine*PI/180;
  fi_2 = latGPS_endLine*PI/180;

  delta_fi = fi_2-fi_1;

  lam_1 = longGPS_startLine*PI/180;
  lam_2 = longGPS_endLine*PI/180;

  delta_lam = lam_2-lam_1;

  a = (sin(delta_fi/2))*(sin(delta_fi/2)) + cos(fi_1)*cos(fi_2)*(sin(delta_lam/2))*(sin(delta_lam/2));

  c = 2*atan2(sqrt(a), sqrt(1-a));

  d = R*c;

  Serial.println("Distância entre coordenadas: ");
  Serial.print(d);
}

int count = 0;

bool waitForRecvBatAtSendingPoint = false;

bool goToReceiveExtraDataViaESPNOW = false;

int brightness = 0;
int fadeAmount = 4;

volatile bool camera = false;

volatile bool num_serie_changed = false;

const int analogInPin = 13;
int adc_value = 0;
double voltage_adc = 0; 

double Vcc_voltage_esp32cam = 5;   //3.7V

double per_bat_esp32 = 0;
int maximum_load = 400;
double TimeSpent = 0;

double t_life_sec = 0;


int diffSubSeconds = 0;

#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER

#define RXD2 12
#define TXD2 0

bool gga_recv = false;

bool sendACK = false;

int BatteryAtSendingDevice = 0;

int TimeLifeBatAtSendingDevice = 0;

int LatAtSendingPoint = 0;
int LongAtSendingPoint = 0;

double batTransmission = 0;

int milesSecGPS = 0;

int64_t time1 = 0;

bool gpsParsing = false;

double diffOfTime = 0;

int hoursGPS = 0;
int minutesGPS = 0;
int secondsGPS = 0;

extern int64_t time2;

const char DEGREE_SYMBOL[] = { 0xB0, '\0' }; 

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int8_t i = 0;
int8_t per_tr = 0;
int8_t time_utc = 0;
int8_t time_s_utc = 0;
int8_t latx = 0;
int8_t lat_s = 0;
int8_t longx = 0;
int8_t long_s = 0;
bool calc_GPS_time = false;
char inSt_s_sent[3] = {};
int8_t time_vec[6];
int8_t time_s_vec[3];
int64_t h = 0;
int64_t m = 0;
int64_t s = 0;
int64_t sub_s = 0;
int64_t gps_time = 0;
char inChar = NULL;

int8_t lat_vec[4];
int8_t lat_s_vec[4];
int8_t long_vec[5];
int8_t long_s_vec[4];

int latInd = 0;
int longInd = 0;
int latsInd = 0;
int longsInd = 0;

bool sincronized = false;

#include "camera_pins.h"
//#include "ESP32_LogicAnalyzer.h"

int8_t buffB[120*160];
int8_t frame_ant [19200];
volatile bool readytoGo;
volatile bool start_race = false;

bool parsex = false, comp = false, sent_comp = false;

void stream_handler();

bool calcTimeDone = false;


volatile bool checkIfDetected = false;

//-----------------------------------------------------//

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xF0,0x08,0xD1,0xC9,0x24,0xC4};

#define ESPNOW_CHANNEL 0

#include <sys/time.h>

//#include "esp_private/wifi.h"

//DISABLE BOTH Components => Wi-Fi => "WiFi AMPDU TX" & "WiFi AMPDU RX"

//--------------------------------------------//
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels   //64

#define I2C_SDA 14
#define I2C_SCL 15

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//--------------------------------------------//

#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"

bool serieDone = false;

int secsInSendingPoint = 0;

int microsOfSecond = 0;


bool sincronize = false;

bool timeAdjustFL = false;

extern bool wait_and_presentTime;


double TimeSerie = 0;

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

//int64_t gps_time = 0;

const char PERC_SYMBOL[] = { 0x25, '\0' };

const char BAT_SYMBOL[] = { 0x00, '\0' };

//bool s = false;

static intr_handle_t handle_console;

//uint16_t i;

bool infoGPS_cap = false;
bool uartINTR = false;

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)  


#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)



struct timeval tv;

bool go_GPS = false;

int hrs = 0;
int mins = 0;
int sec = 0;
int milisecs = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    byte a[10];
    byte b[6] ;     
} struct_message;

// Create a struct_message called myData
struct_message myData;

typedef struct struct_message_Bat {
  byte Bat[5];
  byte timeLifeBat[7];  
} struct_message_Bat;

struct_message_Bat myData_Bat;

typedef struct struct_GetDHT_data {
  byte Humid[2];
  byte Temper[2];
  byte Lat[8];
  byte Long[9];
} struct_dhtData;

struct_dhtData struct_dht_data;

//-----------------------------------------------------------------//

volatile bool tl_is_running = false;

//camera_state_t *s_statex = NULL;

//const char* ssid = "NOS-F9F0F2"; // AndroidAP
//const char* password = "FACF28A7";  // yxbk0096

const char* ssid = "labs";
const char* password = "robot1cA!ESTG";

// Keep track of number of pictures
unsigned int pictureNumber = 0;

//Stores the camera configuration parameters
camera_config_t config;

void takePhoto(){
  // Take Picture with Camera
  camera_fb_t  * fb = esp_camera_fb_get();

  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
    //return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb); 
}


void startCameraServer(int codeFunctionality);

//-----------------------------------------------------------------//
 //------------------------------------------------------------------------------------------------------------------------------------OLED--------------------//


void show_ExtraInfo(int temp, int hum, int SL_dist){   
     

    itoa(temp, temp_v, 10);
    itoa(hum, hum_v, 10);
    itoa(SL_dist, dist_v, 10);


      for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{ 
            // u8g2.clearBuffer();
          //u8g2.clear();
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(5, 15, temp_v);    
          u8g2.drawUTF8(28, 15, DEGREE_SYMBOL);
          u8g2.drawStr(38, 15, "C");
          //u8g2.sendBuffer();     

        }while(u8g2.nextPage());

         if(!u8g2.nextPage()){
            Serial.println(". . . . . Stop to presenting");
            break;
          
         }

        delay(1000);
      }

      delay(1000);

      for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{      

          //u8g2.clear();
          //u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(5, 15, hum_v);    
          u8g2.drawUTF8(28, 15, PERC_SYMBOL);
          u8g2.drawStr(38, 15, "RH");
          //u8g2.sendBuffer();          

        }while(u8g2.nextPage());

         if(!u8g2.nextPage()){
            Serial.println(". . . . . Stop to presenting");
            break;
          
        }

        delay(1000);
      }

      delay(1000);

      for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{   
          //u8g2.clear();
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(5, 15, dist_v);             
          u8g2.drawStr(40, 15, "M");

          }while(u8g2.nextPage());

          if(!u8g2.nextPage()){
            Serial.println(". . . . . Stop to presenting");
            break;
          
          }
        delay(1000);
      }        
      delay(1000);
}

void show_System_Bat_OLED(double sysBat){  

  BatteryOfSystem = sysBat;

  itoa((int)sysBat, battery, 10); 

  
    for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{ 
          //u8g2.clear();
          Serial.printf("\n\nP[%d]", i);
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(5, 15, "B:"); 
          u8g2.drawStr(28, 15, battery);    
          u8g2.drawUTF8(63, 15, PERC_SYMBOL);
        }while(u8g2.nextPage());
        
      
      if(!u8g2.nextPage()){
          Serial.println(". . . . . Stop to presenting");
          break;
      }
      delay(500);
    }

    delay(500);  
}

bool show_Msg_OLED(float timeOfSerie){

  bool done = true; 

  int timeOfSerieInteger = 0;

  timeOfSerieInteger = int(abs(timeOfSerie));

  itoa(timeOfSerieInteger, timeSerieOLED, 10);   

  itoa((abs(timeOfSerie)-timeOfSerieInteger)*100, timeSerieOLED_subsec, 10); 

  int positionForDot = 0;

  if(abs(timeOfSerie)/10 < 0){
    positionForDot = 60;
  }else if(abs(timeOfSerie)/10 > 0){
    positionForDot = 72;
  }
  

  for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{           
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(5, 15, "AT:"); 
          u8g2.drawStr(40, 15, timeSerieOLED);  
          u8g2.drawStr(positionForDot, 15, ".");  
          u8g2.drawStr(positionForDot+15, 15, timeSerieOLED_subsec);
          u8g2.drawStr(115, 15, "s");
        }while(u8g2.nextPage());
        //u8g2.sendBuffer();
      
       
    if(!u8g2.nextPage()){
      Serial.println(". . . . . Stop to presenting");
      break;
    }

    delay(500);
  }

  return done;

}

  //------------------------------------------------------------------------------------------------------------------------------------OLED--------------------//

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  start_race = true;

  checkIfDetected = true;

  digitalWrite(2, LOW);

  //stream_handler();
  //start_race = true;
  
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Segundos: ");

  

  for(i=0; i<4; i++){
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(100);
  }

  
  
  for(byte indic=0; indic<10; indic++){
    Serial.println(myData.a[indic]);
  }
  Serial.print("Microssegundos: ");

  for(byte indicx=0; indicx<6; indicx++){
    Serial.println(myData.b[indicx]);
  }
  
  Serial.println();

  secsInSendingPoint = myData.a[0]*1000000000 + myData.a[1]*100000000 + myData.a[2]*10000000 + myData.a[3]*1000000 + myData.a[4]*100000 + myData.a[5]*10000 + myData.a[6]*1000 + myData.a[7]*100 + myData.a[8]*10 + myData.a[9];

  microsOfSecond = myData.b[0]*100000 + myData.b[1]*10000 + myData.b[2]*1000 + myData.b[3]*100 + myData.b[4]*10 + myData.b[5];


  time1 = esp_timer_get_time();

   
}


void Camera_Config(){
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;  //YUV422,GRAYSCALE,RGB565,JPEG
    //init with high specs to pre-allocate larger buffers
    if(psramFound()){
      config.frame_size = FRAMESIZE_QQVGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
  
    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
  
    
   
    //------------------------------------------------------------------
    
  
  sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 1);          // 0 = disable , 1 = enable Vflip
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    //drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_QQVGA); // - SF QQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    //initial sensors are flipped vertically and colors are a bit saturated
    //drop down frame size for higher initial frame rate
  
  

}
 
void setup() {
  // Initialize Serial Monitor
 
 Serial.begin(115200);

 Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

 u8g2.begin();
 u8g2.enableUTF8Print();

 pinMode(2, OUTPUT);

 ledcSetup(1, 1000, 8);
 ledcAttachPin(2, 1);

 Camera_Config();

 gpsParsing = true;
 
}


void espnowConfig(){ 
  
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //WiFi.mode(WIFI_AP_STA);  

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv); 
}

void calculateGPS_time(int8_t time_vec[6], int8_t time_s_vec[3]) {
  if(gga_recv){
    Serial.println("\nTHAT\n");
    h = 10 * time_vec[0] + time_vec[1];
    m = 10 * time_vec[2] + time_vec[3];
    s = 10 * time_vec[4] + time_vec[5];
    sub_s = 100 * time_s_vec[0] + 10 * time_s_vec[1] + time_s_vec[2];
  
    gps_time = h * 3600 + m * 60 + s + sub_s / 1000;
  
    //calcTimeDone = true;
  
    Serial.printf("\nTime: %d:%d:%d", hoursGPS, minutesGPS, secondsGPS);

    Serial.printf("\nGPS Time: %u", gps_time);
  
    sincronizingRTC_esp32_WithGPS(hoursGPS, minutesGPS, secondsGPS);

    gga_recv = false;
  }
}

void ESPserialEvent() {

  if(NUM_SERIE > 1){
    //Serial.println("------ESP SERIAL EVENT AGAIN");
  }

  
  // get the new byte:

  
    inChar = (char)Serial2.read();
  
    

  if (inChar == '$') {
    count = 1;
    
  }

  if (count == 2  && inChar != NULL) {
    count = 3;
    inChar = NULL;
  }

  if ((inChar == 'G') && (count == 1)) {
    count = 2;
    inChar = NULL;
  } else if ((inChar == 'G') && (count == 3)) {
    parsex = true;
    count = 0;
  }



  if (parsex) {
    inSt_s_sent[i] = inChar;
    i++;
    
    if (i == 3) {
      i = 0;
      parsex = false;
      comp = true;
    }
  }

  if (comp) {
    //Serial.println("\nOK !!!!!!\n");
    //Serial.println(inSt_s_sent);
    if (String(inSt_s_sent) == "GGA") {
      gga_recv = true;
      sent_comp = true;
      //comp = false;
//      state = !state;
//      digitalWrite(ledPin, state);

      digitalWrite(2, HIGH);
      Serial.println("\nRead !!!!!!\n");
    }
    comp = false;
  }

  if (sent_comp && (inChar == ',')) {
    //Serial.println("PER TR = 1");
    per_tr = 1;
    sent_comp = false;
  }

  //Serial.printf("\nPER_TR: %u\n", per_tr);
  //Serial.printf("\nTIME_UTC: %u\n", time_utc);
  if ((per_tr == 1) && (time_utc < 6) && gga_recv) {
    
    if (((char)inChar >= '0') && ((char)inChar <= '9')) {
      
      time_vec[time_utc] = (int)inChar - 48;  //inChar - '0'
      time_utc = time_utc + 1;
      if (time_utc == 6) {
        per_tr = 2;
        time_utc = 0;
   

        Serial.println("\n");
        

        Serial.print("Hours: ");
        Serial.print(time_vec[0]*10 + time_vec[1]);
        hoursGPS = time_vec[0]*10 + time_vec[1];
        Serial.println("\n");
        Serial.print("Minutos: ");
        Serial.print(time_vec[2]*10 + time_vec[3]);
        minutesGPS = time_vec[2]*10 + time_vec[3];
        Serial.println("\n");
        Serial.print("Segundos: ");
        Serial.print(time_vec[4]*10 + time_vec[5]);
        secondsGPS = time_vec[4]*10 + time_vec[5];

        
        Serial.println("\n");
      }
      
    }
  }

  if ((per_tr == 2) && (inChar == '.')) {
    per_tr = 3;
  }

  if ((per_tr == 3) && (time_s_utc < 3) && gga_recv) {
    if ((inChar >= '0') && (inChar <= '9')) {
      time_s_vec[time_s_utc] = (int)inChar - 48;
      time_s_utc = time_s_utc + 1;
      if (time_s_utc == 3) {
        per_tr = 4;
        time_s_utc = 0;
        Serial.print(".");
        for (int y = 0; y < 3; y++) {
          Serial.printf("%u", time_s_vec[y]);
        }

        milesSecGPS = time_s_vec[0]*100 + time_s_vec[1] + time_s_vec[2];

        Serial.println("\n");
        Serial.print(milesSecGPS);
        Serial.println("\n");
      }
      calc_GPS_time = true;
      per_tr = 0;
    }
  }

  if ((per_tr == 4) && (inChar == ',')) {
    per_tr = 5;
  }

  if ((per_tr == 5) && (latx < 4)) {
    if ((inChar >= '0') && (inChar <= '9')) {
      lat_vec[latInd] = (int)inChar - 48;
      latx = latx + 1;
      latInd = latInd + 1; 
      if(latx == 4){
        per_tr = 6;
        latx = 0;
        latInd = 0;
      }
    }
  }

  if ((per_tr == 6) && (inChar == '.')) {
    per_tr = 7;
  }

  if ((per_tr == 7) && (lat_s < 4)) {
    if ((inChar >= '0') && (inChar <= '9')) {
      lat_s_vec[latsInd] = (int)inChar - 48;
      lat_s = lat_s + 1;      
      latsInd = latsInd + 1; 
      if(lat_s == 4){
        per_tr = 8;
        lat_s = 0;
        latsInd = 0;
      }      
    }
  }

  if ((per_tr == 8) && (inChar == ',')) {
    per_tr = 9;
  }

  if ((per_tr == 9) && ((inChar == 'N') || (inChar == 'S'))) {
    per_tr = 10;
  }

  if ((per_tr == 10) && (inChar == ',')) {
    per_tr = 11;
  }

  if ((per_tr == 11) && (longx < 5)) {
    if ((inChar >= '0') && (inChar <= '9')) {
      long_vec[longInd] = (int)inChar - 48;       
      longInd = longInd + 1; 
      longx = longx + 1;
      if(longx == 5){
        per_tr = 12;
        longx = 0;
        longInd = 0;
      }
    }
  }

  if ((per_tr == 12) && (inChar == '.')) {
    per_tr = 13;
  }

  if ((per_tr == 13) && (long_s < 3)) {
    if ((inChar >= '0') && (inChar <= '9')) {
      long_s_vec[longsInd] = (int)inChar - 48;       
      longsInd = longsInd + 1;       
      long_s = long_s + 1;
      if(long_s == 3){
        per_tr = 14;
        long_s = 0;
        longsInd = 0;
      }
    }
  }

  if ((per_tr == 14) && (inChar == ',')) {
    per_tr = 15;
  }

  if ((per_tr == 15) && ((inChar == 'E') || (inChar == 'W'))) {
    per_tr = 16;
  }

  if ((per_tr == 16) && (inChar == ',')) {
    per_tr = 17;
  }

  if ((per_tr == 17) && ((inChar >= '0') && (inChar <= '2'))) {
    per_tr = 18;
  }

  if ((per_tr == 18) && (inChar == ',')) {
    per_tr = 19;
  }

  if ((per_tr == 19) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 20;
  }

  if ((per_tr == 20) && (inChar == ',')) {
    per_tr = 21;
  }

  if ((per_tr == 21) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 22;
  }

  if ((per_tr == 22) && (inChar == '.')) {
    per_tr = 23;
  }

  if ((per_tr == 23) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 24;
  }

  if ((per_tr == 24) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 25;
  }

  if ((per_tr == 25) && (inChar == ',')) {
    per_tr = 26;
  }

  if ((per_tr == 26) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 27;
  }

  if ((per_tr == 27) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 28;
  }

  if ((per_tr == 28) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 29;
  }

//  if(per_tr == 30){
//    Serial.print(inChar);
//  }
  
  if ((per_tr == 29) && (inChar == '.')) {
    per_tr = 30;
    //calculateGPS_time(time_vec, time_s_vec);
    //per_tr = 0;
    //Serial.print(inChar);
  }

  if ((per_tr == 30) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 31;
  }

  if ((per_tr == 31) && (inChar == ',')) {
    per_tr = 32;
  }

  if ((per_tr == 32) && (inChar == 'M')) {
    per_tr = 33;
  }

  if ((per_tr == 33) && (inChar == ',')) {
    per_tr = 34;
  }

  if ((per_tr == 34) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 35;
  }

  if ((per_tr == 35) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 36;
  }

  if ((per_tr == 36) && (inChar == '.')) {
    per_tr = 37;
  }

  if ((per_tr == 37) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 38;
  }

  if ((per_tr == 38) && (inChar == ',')) {
    per_tr = 39;
  }

  if ((per_tr == 39) && (inChar == 'M')) {
    per_tr = 40;
  }

  if ((per_tr == 40) && (inChar == ',')) {
    per_tr = 41;
  }

  if ((per_tr == 41) && (inChar == ',')) {
    per_tr = 42;
  }

  if ((per_tr == 42) && (inChar == '*')) {
    per_tr = 43;
  }

  if ((per_tr == 43) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 44;
  }

  if ((per_tr == 44) && ((inChar >= '0') && (inChar <= '9'))) {
    per_tr = 45;
  }

  if (per_tr == 45) {
    calc_GPS_time = true;
    per_tr = 0;
  }

  if (calc_GPS_time) {
    calc_GPS_time = false;
    //Serial.println("\nYA\n");
    calculateGPS_time(time_vec, time_s_vec);    
  }

  
}

float humi = 0;
float t = 0;

void sincronizingRTC_esp32_WithGPS(int hours, int minutes, int seconds){
  //First Sinchronization//--------------------------------------------------------------//    

  int Year = 2021;
  int Month = 5;
  int Day = 5;

  

        gpsParsing = false;
        struct tm tm;
        tm.tm_year = 2018 - 1900;
        tm.tm_mon = 10;
        tm.tm_mday = 15;
        Serial.println("\nHour: ");
        Serial.print(hr_dez + hr_un);
        Serial.println("\n");
        tm.tm_hour = hours;
        tm.tm_min = minutes;
        tm.tm_sec = seconds; 
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

//    Serial.print(tv->tv_sec);
//
//    int64_t secondsTo = tv->tv_sec;
//
//    Serial.println("\nsecondsTo\n");
//  
//    int secs = secsToTime(secondsTo);
//    
//    Serial.println("\nARRIVE3\n");
//  
//    int x = (sec_dez + sec_un)*1000 + millis();
//    int y = secs*1000 + millis();
//    int gap = x-y;
//
//    Serial.println("\nARRIVE4\n");
//
////  }
//
//  if (s &&((gap>=1000) || (gap<=-1000))){
//    s = false;
//    struct tm tm;
//        tm.tm_year = 2018 - 1900;
//        tm.tm_mon = 10;
//        tm.tm_mday = 15;
//        tm.tm_hour = hr_dez + hr_un;
//        tm.tm_min = min_dez + min_un;
//        tm.tm_sec = sec_dez + sec_un;
//        tm.tm_isdst = -1; 
//        time_t t_2 = mktime(&tm);
//
//        Serial.printf("Setting time: %s", asctime(&tm));
//
//        struct timeval now_2 = { .tv_sec = t_2 };
//
//        settimeofday(&now_2, NULL);
//  
//  }

  //Second Sinchronization//--------------------------------------------------------------//

  //wait_and_presentTime = true;

  sincronized = true;

  //Serial.println("\ncheckIfDetected: ");
  //Serial.print(checkIfDetected);  
  
}

void sendByBLE_ToAPP(float timeSerie){
  Serial.println("Starting BLE work!");

  BLEDevice::init("TSA");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       ); 

  uint8_t vecToSend[4];
  uint8_t vecToSendShortSerie[3];

  double exampleDiv10 = 0;

  int decNum = 0;
  int decNumDiv = 0;
  
  exampleDiv10 = timeSerie/10;
  
  int lenBuff = 0;
  
  if(exampleDiv10 > 0){
    vecToSend[0] = int(exampleDiv10); 
    vecToSend[1] = int(timeSerie)-vecToSend[0]*10;
    
    decNum = (timeSerie - (vecToSend[0]*10 + vecToSend[1]))*100;
    
    decNumDiv = decNum/10;
    
    vecToSend[2] = int(decNumDiv);
    vecToSend[3] = decNum - vecToSend[2]*10; 
    
    lenBuff = sizeof(vecToSend);

    pCharacteristic->setValue(vecToSend, lenBuff);
        
  }else if(exampleDiv10 < 0){
    vecToSendShortSerie[0] = int(timeSerie);
    decNum = (timeSerie - vecToSendShortSerie[0])*100;
    decNumDiv = decNum/10;
    
    vecToSendShortSerie[1] = int(decNumDiv);
    vecToSendShortSerie[2] = decNum - vecToSendShortSerie[1]*10; 
    
    lenBuff = sizeof(vecToSendShortSerie);
    
    pCharacteristic->setValue(vecToSendShortSerie, lenBuff);
  }
  
  //pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);  
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void waitAndPresentTime(){

  //delay(10000);

  //Serial.println("\nwait\n");
  
  struct timeval tvTime;
  
  gettimeofday(&tvTime, NULL);
  
  int iTotal_seconds = tvTime.tv_sec;
//  struct tm *ptm = localtime((const time_t *) & iTotal_seconds);


 Serial.println("\nSeconds at reception: ");
 Serial.print(iTotal_seconds);

  

  int secsRecep = tvTime.tv_sec;
  int microsecsRecep = tvTime.tv_usec;
  double endingTime = secsRecep + microsecsRecep/1000000;
  double startingTime = secsInSendingPoint + microsOfSecond/1000000;
  Serial.println("\nTime of ending: ");
  Serial.print(endingTime);
  Serial.printf("\n\nAt finishing: %d.%d\n\n", secsRecep, microsecsRecep);
  Serial.println("\nTime of starting: ");
  Serial.print(startingTime);
  Serial.printf("\n\nAt starting: %d.%d\n\n", secsInSendingPoint, microsOfSecond);

  TimeSerie = secsRecep-secsInSendingPoint; 
  

  diffSubSeconds = microsecsRecep-microsOfSecond;  

  Serial.println("Time of Serie: "); 
  
  delay(1000);

  float serieTime = abs(TimeSerie*100 + (diffSubSeconds/10000));

  serieTime = serieTime/100;

  Serial.print(serieTime);

  serieDone = show_Msg_OLED(serieTime); 

  timesSeries[NUM_SERIE - 1] = serieTime;


  sendByBLE_ToAPP(serieTime);

  
  start_race = false;
  

  sendingAcknowledgement();
 
}

void sendingAcknowledgement(){

  if(NUM_SERIE == 1){
    esp_now_peer_info_t peerInfoACK = {};
    
    memcpy(peerInfoACK.peer_addr, broadcastAddress, 6);
    peerInfoACK.channel = 0;  
    peerInfoACK.encrypt = false;
    //Serial.println("-----------------------D--------------------------");
              
    if (esp_now_add_peer(&peerInfoACK) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }   
    
  }
  
  int ack = 1;



 

   for(i=0; i<4; i++){
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(100);
   }

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ack, sizeof(int));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  //waitForRecvBatAtSendingPoint = true;
  receivingBatOnSendingPoint();
}


//---------------------------------------------------------------------------------------------------------------------------------//

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

//---------------------------------------------------------------------------------------------------------------------------------//

void showingTimeLifeBatAtSendingPoint(double timeLifeAtSendingPoint){  
  int daysTimeLife = 0;
  int hoursTimeLife = 0;
  int minutesTimeLife = 0;
  int secondsTimeLife = 0;

  daysTimeLife = (int)((timeLifeAtSendingPoint/24)/3600);

  hoursTimeLife = (int)((timeLifeAtSendingPoint - daysTimeLife*24*3600)/3600);

  minutesTimeLife = (int)((((timeLifeAtSendingPoint - daysTimeLife*24*3600)/3600)-hoursTimeLife)*60);

  secondsTimeLife = (int)((((((timeLifeAtSendingPoint - daysTimeLife*24*3600)/3600)-hoursTimeLife)*60)-minutesTimeLife)*60);

  Serial.printf("\nTime life of battery at sending point: %d:%d:%d:%d dias", daysTimeLife, hoursTimeLife, minutesTimeLife, secondsTimeLife);  
}

void showingTimeLifeBatAtEndPoint(double timeLifeAtEndPoint){
  int daysTimeLife = 0;
  int hoursTimeLife = 0;
  int minutesTimeLife = 0;
  int secondsTimeLife = 0;

  daysTimeLife = (int)((timeLifeAtEndPoint/24)/3600);

  hoursTimeLife = (int)((timeLifeAtEndPoint - daysTimeLife*24*3600)/3600);

  minutesTimeLife = (int)((((timeLifeAtEndPoint - daysTimeLife*24*3600)/3600)-hoursTimeLife)*60);

  secondsTimeLife = (int)((((((timeLifeAtEndPoint - daysTimeLife*24*3600)/3600)-hoursTimeLife)*60)-minutesTimeLife)*60);

  Serial.printf("\nTime life of system battery: %d:%d:%d:%d dias", daysTimeLife, hoursTimeLife, minutesTimeLife, secondsTimeLife); 

  if(timeLifeAtEndPoint < 0){
    Serial.println("\n\n----BAD TIME LIFE OF BATTERY INDICATION --- WRONG SYSTEM BATTERY TIME LIFE\n\n\n");
  }

  //readingDHT11();
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
  ledcWrite(0,0);
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
}

void getBatteryAnalysis(){
  voltage_adc = getVoltageADC(Vcc_voltage_esp32cam, adc_reading());
  per_bat_esp32 = CalculatePerBattery(TimeSerie + diffSubSeconds/1000000, voltage_adc, maximum_load);
  
  //PerBat_show(per_bat_esp32);
  
  t_life_sec = CalculateTimeLife(maximum_load, voltage_adc, TimeSerie + diffSubSeconds/1000000);
}

void getTimelifeSystemBattery(double timeLifeBat_t, double timeLifeBat_r){
  double TimeLifeBatteryOfSystem = 0;

  if(timeLifeBat_t < timeLifeBat_r){
    TimeLifeBatteryOfSystem = timeLifeBat_t;
  }else{
    TimeLifeBatteryOfSystem = timeLifeBat_r;
  }

  showingTimeLifeBatAtEndPoint(TimeLifeBatteryOfSystem);
  
}



void getSystemBattery(double PerBat_t, double PerBat_r){
  double PerBatterySystem = 0;

  if(PerBat_t < PerBat_r){
    PerBatterySystem = PerBat_t;
  }else{
    PerBatterySystem = PerBat_r;
  }

  //delay(1000);

  show_System_Bat_OLED(PerBatterySystem);
  
}

void OnDataRecvExtradata(const uint8_t * mac, const uint8_t *incomingData, int len) { 

  int TemperatureDegreesAtSendingPoint = 0;
  int RelativeHumidityAtSendingPoint = 0;
  
  memcpy(&struct_dht_data, incomingData, sizeof(struct_dht_data));
  Serial.print("Bytes received: ");
  Serial.println(len); 
  
  TemperatureDegreesAtSendingPoint = struct_dht_data.Temper[0]*10 + struct_dht_data.Humid[1]; 
  
  RelativeHumidityAtSendingPoint = struct_dht_data.Humid[0]*10 + struct_dht_data.Humid[1];

  LatAtSendingPoint = struct_dht_data.Lat[0]*10000000 + struct_dht_data.Lat[1]*1000000 + struct_dht_data.Lat[2]*100000 + struct_dht_data.Lat[3]*10000 + struct_dht_data.Lat[4]*1000 + struct_dht_data.Lat[5]*100 + struct_dht_data.Lat[6]*10 + struct_dht_data.Lat[7]; 

  LatAtSendingPoint/=1000;  
  
  LongAtSendingPoint = struct_dht_data.Long[0]*100000000 + struct_dht_data.Long[1]*10000000 + struct_dht_data.Long[2]*1000000 + struct_dht_data.Long[3]*100000 + struct_dht_data.Long[4]*10000 + struct_dht_data.Long[5]*1000 + struct_dht_data.Long[6]*100 + struct_dht_data.Long[7]*10 + struct_dht_data.Long[8];

  LongAtSendingPoint/=1000;
  
  //At Receiving Point
  LatEndingPoint = ((lat_vec[0]*10 + lat_vec[1])*3600 + (lat_vec[2]*10 + lat_vec[3])*60 + (lat_s_vec[0]*10 + lat_s_vec[1]) + (lat_s_vec[2]*10 + lat_s_vec[3])/100)/3600;
  LongEndingPoint = ((long_vec[0]*100 + long_vec[1]*10 + long_vec[2])*3600 + (long_vec[3]*10 + long_vec[4])*60 + (long_s_vec[0]*10 + long_s_vec[1]) + (long_s_vec[2]*10 + long_s_vec[3])/100)/3600;

  distH = calculateDistanceBetGPSs_HaversineFormula(LatAtSendingPoint, LongAtSendingPoint, LatEndingPoint, LongEndingPoint);

  RealdistSL = getRealDistanceStraightLine(distH);

  calculateEstimatedSpeed(RealdistSL, abs(TimeSerie + diffSubSeconds/1000000));

  show_ExtraInfo(TemperatureDegreesAtSendingPoint, RelativeHumidityAtSendingPoint, (int)RealdistSL);

  //--------------------------//
    
    //  abort();  
}

void receivingExtraDataOnSendingPoint(){  
  Serial.println("\nreceivingExtraDataOnSendingPoint\n");
  esp_now_register_recv_cb(OnDataRecvExtradata);

  delay(1000);
}

void OnDataRecvBat(const uint8_t * mac, const uint8_t *incomingData, int len) {  
    
    memcpy(&myData_Bat, incomingData, sizeof(myData_Bat));
    Serial.print("Bytes received: ");
    Serial.println(len);    

//  for(i=0; i<len; i++){
//    digitalWrite(2, HIGH);
//    delay(100);
//    digitalWrite(2, LOW);
//    delay(200);
//  }

  //led_dataRecv = true;

   for(i=0; i<4; i++){
    digitalWrite(2, HIGH);
    delay(50);
    digitalWrite(2, LOW);
    delay(100);
   }
  
  BatteryAtSendingDevice = myData_Bat.Bat[0]*10000 + myData_Bat.Bat[1]*1000 + myData_Bat.Bat[2]*100 + myData_Bat.Bat[3]*10 + myData_Bat.Bat[4];

  TimeLifeBatAtSendingDevice = myData_Bat.timeLifeBat[0]*1000000 + myData_Bat.timeLifeBat[1]*100000 + myData_Bat.timeLifeBat[2]*10000 + myData_Bat.timeLifeBat[3]*1000 + myData_Bat.timeLifeBat[4]*100 + myData_Bat.timeLifeBat[5]*10 + myData_Bat.timeLifeBat[6];
  batTransmission = BatteryAtSendingDevice/100;

  Serial.println("\n\n\n------------------Baterry and sending point: ");
  Serial.print(batTransmission);
  Serial.println("\n\n\n");

  showingTimeLifeBatAtSendingPoint(TimeLifeBatAtSendingDevice);

  camera = false; 


    getBatteryAnalysis(); 

   
    
    getSystemBattery(batTransmission, per_bat_esp32);

   

    getTimelifeSystemBattery(TimeLifeBatAtSendingDevice, t_life_sec);

 
  start_race = false;
 
  serieDone = false;
  gpsParsing = true;

  
  NUM_SERIE++;
  num_serie_changed = true;

  if(NUM_SERIE < (NUM_PROG_SERIES + 2)){
    Serial.printf("\n\n\nNEXT SERIE !!! - READY FOR %dª serie\n\n\n", NUM_SERIE);
    //break;
  }else{
    Serial.printf("\n\nNUMBER OF SERIES OVERLOADED !!!\n\n");
    gpsParsing = false;
  }

 
  
}

void batteryInfoFunction(){
  //Serial.println("\n-----------Info---------------\n");
  getBatteryAnalysis(); 

  delay(2000);

  getSystemBattery(batTransmission, per_bat_esp32);

  getTimelifeSystemBattery(TimeLifeBatAtSendingDevice, t_life_sec);
  delay(2000);

  serieDone = false;

  nextRace = true;  
}

void receivingBatOnSendingPoint(){

  Serial.println("\nReceiving battery");

//  WiFi.mode(WIFI_STA);
//
//   // Init ESP-NOW
//  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecvBat);

  //delay(3000);

  //Serial.println("Telmo 03");

   Serial.print("ESP Board MAC Address:  ");
   Serial.println(WiFi.macAddress());



  // Serial.print(BatInfoFunction);

//  if(BatInfoFunction == true){
//    BatInfoFunction = false;
//    Serial.println("\n\n-------------------OK_BAT---------------\n\n");
//    batteryInfoFunction();
//  }

//  if(nextRace){
//    nextRace = false;
//    //abort();
//    delay(1000);
//    gpsParsing = true;
//  }
  
  //------------------------------------------//
  
}

  
//--------------------------------------------------------------//
 
void loop() {

   BLEDevice::startAdvertising();

  if(start_race){
      camera = true;
      //Serial.println("\nRace Started\n");
      //startCameraServer(5);   //Select Streaming Functionality
      stream_handler();
    
//      Serial.print("Camera Ready! Use 'http://");
//      Serial.print(WiFi.localIP());
//      Serial.println("' to connect");
  }else{

    if(!serieDone){
      //Serial.println("Passou Aqui!!!");
      if(gpsParsing == true){
        if(NUM_SERIE > 1){
          //Serial.println("\n\n---GOING TO PARSING OF GPS DATA !!!\n\n");
        }
        //gpsParsing = false;
        //putGPS_working();
//        if(NUM_SERIE > 1){
//           Serial.println("\n-----------HAVE TO ARRIVE THERE-------------\n");
//        }           
        if(Serial2.available() > 0){
          ESPserialEvent();
        }
      }
    
  //    if(timeAdjustFL == true){
  //      timeAdjustFL = false;
  //      timeAdjustParsingGPS();
  //    }
    
      if(calcTimeDone == true){
        calcTimeDone = false;      
        //sincronizingRTC_esp32_WithGPS(time_vec[0], time_vec[1], time_vec[2], time_vec[3], time_vec[4], time_vec[5]);      
      }
  
      if(sincronized){
        sincronized = false;
        espnowConfig();
      }  
    
      if(wait_and_presentTime){
        wait_and_presentTime = false;
        waitAndPresentTime();    
      }
    }else{
      //Serial.println("\nA\n");
      if(sendACK){
        sendACK = false;
        //Serial.println("\n\n-------------------------Go to send ACK !!!-------------\n\n");
        sendingAcknowledgement();
      }else if(waitForRecvBatAtSendingPoint){
        waitForRecvBatAtSendingPoint = false;
        //Serial.println("-------------------------Go to receive battery------------");
        start_race = false;
        //Serial.println("\nB - \n");
        //Serial.println(goToReceiveExtraDataViaESPNOW);
        receivingBatOnSendingPoint();      
      }else if(led_dataRecv){
        led_dataRecv = false;
       for(i=0; i<8; i++){
        digitalWrite(2, HIGH);
        delay(50);
        digitalWrite(2, LOW);
        delay(100);
       }
      }else if(goToReceiveExtraDataViaESPNOW){
        goToReceiveExtraDataViaESPNOW = false;
        Serial.println("\nC\n");
        receivingExtraDataOnSendingPoint();
      }
    }
  }
  
  

}
