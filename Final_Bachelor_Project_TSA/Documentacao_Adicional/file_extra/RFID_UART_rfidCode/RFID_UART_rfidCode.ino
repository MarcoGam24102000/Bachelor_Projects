// RFID_UART.ino

#include <SoftwareSerial.h>
#include <SeeedRFID.h>

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xF0,0x08,0xD1,0xC9,0x24,0xC4};

//#define RFID_RX_PIN 16    //10
//#define RFID_TX_PIN 17

#define RFID_RX_PIN 17    //10
#define RFID_TX_PIN 16


int confirmation = 0;

// #define DEBUG
// #define TEST


int indRFID_reader = 0;

char RFIDCode[11] = {'\0'};

int count = 0;

char packetType = NULL;
int LenData = 0;
char respCode = NULL;
char devNumber = NULL;

char checksumCalc = NULL;

int lengthDATA = 0;

bool goToParseRFID_data = false;

SeeedRFID RFID(RFID_RX_PIN, RFID_TX_PIN);
RFIDdata tag;

void setup() {
    Serial.begin(57600);
    Serial.println("RFID Test..");

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
}


void parsingRFID_data(char tagInfo[]){
        //Serial.println("-> In RFID data parse function");
        packetType = tagInfo[0];
        LenData = tagInfo[1] - '0';
        respCode = tagInfo[2];
        devNumber = tagInfo[3];
        char respData[LenData-3];        
        for(int indData = 0; indData < LenData-3; indData++){
            respData[indData] = tagInfo[4+indData];
        }
        char checksumInChars = tagInfo[1+LenData];

        //Verifying cheksum

        for(int dta = 0; dta < lengthDATA-1; dta++){
          checksumCalc = checksumCalc xor tagInfo[dta];

          if(dta == lengthDATA-2){
            if(checksumInChars == checksumCalc){
              //Serial.println("Checksum right !!!");
            }else{
              //Serial.println("Checksum NOT right !!!");
            }
          }
        }
        
}


void RFID_read(){
  //if(!goToParseRFID_data){
        indRFID_reader++;
        tag = RFID.data();
        Serial.print("RFID card number: ");
        Serial.println(RFID.cardNumber());
        Serial.print("\t LEN: ");   
        Serial.print(tag.dataLen);

        if(tag.dataLen != 0){
          Serial.println("RFID tag checked");
          confirmation = 1;
        }

//        if(RFID.cardNumber() != 0){
//          Serial.println("Data Length: ");
//          Serial.print(tag.dataLen);
//        }
        //#ifdef TEST
        
//        if(indRFID_reader == 1){
//          const int LenString = tag.dataLen;   
//          lengthDATA = tag.dataLen;       
//        }
//                
//        //Serial.print("RFID raw data: ");
//        for (int i = 0; i < tag.dataLen; i++) {
//            //Serial.print(tag.raw[i], HEX);
//            //Serial.print('\t');
//
//            if(i == tag.dataLen - 1){
//              indRFID_reader = 0;
//              char tagString[tag.dataLen];
//
//              for(int indic=0; indic < tag.dataLen; indic++){
//                tagString[indic] = char(tag.raw[indic]);
//
//                 if(indic == tag.dataLen - 1){
//
//                    if((RFID.cardNumber() != 0)  && (tag.raw != 0)){ count++; }
//                    
//                 }
//
//                 if(count == 1){
//                    goToParseRFID_data = true;
//                    parsingRFID_data(&tagString[tag.dataLen]);
//                 }
//              }
//
//              //memcpy(tagString, char(tag.raw), )
//              //tagString[] = char(tag.raw[]);
//            }
//        }
    //}
    
    //else{
        
        
    //}
        //#endif
}

bool RFIDTagHandled(){
    byte value = 0;
    byte bytesRead = 0;
    byte code[6];
    byte checksum = 0;
    byte tempByte = 0;

    bool handled = false;

    const byte RFID_SIZE = 10;
    const byte CSUM_SIZE = 2;
    
    if (RFID.isAvailable()) {
      detachInterrupt(NULL);

      if(0x02 == (value = Serial.read())){
        while(bytesRead < (RFID_SIZE + CSUM_SIZE)){
          if(RFID.isAvailable()){
            value = Serial.read();

            if((0x0D == value) ||
               (0x0A == value) ||
               (0x03 == value) ||
               (0x02 == value)){
               break; 
            }

            if(bytesRead < RFID_SIZE){
              RFIDCode[bytesRead] = value;
            }

            if((value >= '0') && (value <= '9')){
              value -= '0';              
            }else if((value >= 'A') && (value <= 'F')){
              value += 10 - 'A';
            }

            if(bytesRead & 1 == 1){
              code[bytesRead >> 1] = (value | (tempByte << 4));

              if(bytesRead >> 1 != 5){
                checksum ^= code[bytesRead >> 1];
              }              
            }else{
              tempByte = value;              
            }

            bytesRead++;
          }
        }

          if(bytesRead == (RFID_SIZE + CSUM_SIZE)){
            if(code[5] == checksum){
              handled = true;
            }
          }

          attachInterrupt(NULL, NULL, NULL);
      }
    }

    return handled;
}

void sendByESPNOW_conf(int confirm){ 
  
    esp_now_peer_info_t peerInfoACK = {};
    
    memcpy(peerInfoACK.peer_addr, broadcastAddress, 6);
    peerInfoACK.channel = 0;  
    peerInfoACK.encrypt = false;
   
    if (esp_now_add_peer(&peerInfoACK) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }    

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &confirm, sizeof(int));
   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

}

void loop() {
//    if(RFIDTagHandled()){
//      Serial.println("Presence detected !!!");
//    }


    if(confirmation == 1){
      confirmation = 0;
      sendByESPNOW_conf(1);
    }else{
      RFID_read();
    }
}
