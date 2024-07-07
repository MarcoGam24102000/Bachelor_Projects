// RFID_UART.ino

#include <SoftwareSerial.h>
#include <SeeedRFID.h>

//#define RFID_RX_PIN 16    //10
//#define RFID_TX_PIN 17

#define RFID_RX_PIN 17    //10
#define RFID_TX_PIN 16

// #define DEBUG
// #define TEST


int indRFID_reader = 0;

int count = 0;

bool goToParseRFID_data = false;

char RFIDCode[11] = {'\0'};

byte tempByte = 0;

byte checksum = 0;

byte code[6];

SeeedRFID RFID(RFID_RX_PIN, RFID_TX_PIN);
RFIDdata tag;

void setup() {
    Serial.begin(57600);
    Serial.println("RFID Test..");
}

void loop() {
    //if (RFID.isAvailable()) {

    if (RFID.isAvailable()) {
      if(!goToParseRFID_data){
          indRFID_reader++;
          tag = RFID.data();
          Serial.print("RFID card number: ");
          Serial.println(RFID.cardNumber());
          //#ifdef TEST
  
          if(indRFID_reader == 1){
            const int LenString = tag.dataLen;          
          }

          if(tag.valid){
            Serial.print("RFID raw data: ");
            for (int i = 0; i < tag.dataLen; i++) {
                Serial.print(tag.raw[i], HEX);
                Serial.print('\t');
    
                if(i == tag.dataLen - 1){
                  indRFID_reader = 0;
                  char tagString[tag.dataLen];
    
                  for(int indic=0; indic < tag.dataLen; indic++){
                      tagString[indic] = char(tag.raw[indic]);
  
  
                      if((0x0D == tagString[indic]) ||
                         (0x0A == tagString[indic]) ||
                         (0x03 == tagString[indic]) ||
                         (0x02 == tagString[indic])){
                         break; 
                      }
          
                      if(indic < tag.dataLen){
                        RFIDCode[indic] = tagString[indic];
                      }
          
                      if((tagString[indic] >= '0') && (tagString[indic] <= '9')){
                        tagString[indic] -= '0';              
                      }else if((tagString[indic] >= 'A') && (tagString[indic] <= 'F')){
                        tagString[indic] += 10 - 'A';
                      }
          
                      if(indic & 1 == 1){
                        code[indic >> 1] = (tagString[indic] | (tempByte << 4));
          
                        if(indic >> 1 != 5){
                          checksum ^= code[indic >> 1];
                        }              
                      }else{
                        tempByte = tagString[indic];              
                      }
          
                      //bytesRead++;
              
    
                     if(indic == tag.dataLen - 1){
    
                        if((RFID.cardNumber() != 0)  && (tag.raw != 0)){ count++; }
                        
                     }
    
                     if(count == 5){
                        goToParseRFID_data = true;
                     }
  
                     if(indic == tag.dataLen){
                        if(code[5] == checksum){
                          Serial.println("Presence detected !!!");
                        }
                     }
                  }
    
                  //memcpy(tagString, char(tag.raw), )
                  //tagString[] = char(tag.raw[]);
                }
            }
          }
      }
          //#endif

    }
}
