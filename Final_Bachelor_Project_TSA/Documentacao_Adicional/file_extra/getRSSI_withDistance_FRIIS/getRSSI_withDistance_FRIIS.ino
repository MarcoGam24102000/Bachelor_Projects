

#include "Math.h"

float distance = 0;

int logPowerdBm = 30;

float lambda = 0;

int count = 0;

float big_RSSI = -100;

int c = 3e8;

int f = 125000;

int G_t_dB, G_r_dB = 0;

float RSSI = 0;

float RSSI_vec[100];


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);  

}

void loop() {
  // put your main code here, to run repeatedly:


  distance = random(10, 110);

  lambda = c/f;

  RSSI = logPowerdBm + G_t_dB + G_r_dB + 20*log(lambda/(4*PI*distance));

  RSSI_vec[count] = RSSI;

  count++;


  if(count == 100){
    for(int x = 0; x < count; x++){
      if(RSSI_vec[x] > big_RSSI){
        big_RSSI = RSSI_vec[x];
      }

      if(x == count-1){
        Serial.println("Bigger RSSI: \t");
        Serial.print(big_RSSI);
      }
    }
  }
  
}
