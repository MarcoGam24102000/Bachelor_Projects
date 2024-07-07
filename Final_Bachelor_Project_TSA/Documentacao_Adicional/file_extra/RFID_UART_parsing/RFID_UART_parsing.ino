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

void loop() {
    //if (RFID.isAvailable()) {

    if(!goToParseRFID_data){
        indRFID_reader++;
        tag = RFID.data();
        Serial.print("RFID card number: ");
        Serial.println(RFID.cardNumber());
        Serial.print("\t LEN: ");   
        Serial.print(tag.dataLen);

//        if(RFID.cardNumber() != 0){
//          Serial.println("Data Length: ");
//          Serial.print(tag.dataLen);
//        }
        //#ifdef TEST
        
        if(indRFID_reader == 1){
          const int LenString = tag.dataLen;   
          lengthDATA = tag.dataLen;       
        }
                
        //Serial.print("RFID raw data: ");
        for (int i = 0; i < tag.dataLen; i++) {
            //Serial.print(tag.raw[i], HEX);
            //Serial.print('\t');

            if(i == tag.dataLen - 1){
              indRFID_reader = 0;
              char tagString[tag.dataLen];

              for(int indic=0; indic < tag.dataLen; indic++){
                tagString[indic] = char(tag.raw[indic]);

                 if(indic == tag.dataLen - 1){

                    if((RFID.cardNumber() != 0)  && (tag.raw != 0)){ count++; }
                    
                 }

                 if(count == 1){
                    goToParseRFID_data = true;
                    parsingRFID_data(&tagString[tag.dataLen]);
                 }
              }

              //memcpy(tagString, char(tag.raw), )
              //tagString[] = char(tag.raw[]);
            }
        }
    }
    
    //else{
        
        
    //}
        //#endif
}
