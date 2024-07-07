#include <U8g2lib.h>

U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(  /* rotation zero */ U8G2_R0, 15 , 14, U8X8_PIN_NONE);
float timex = 19.19;

char timeSerieOLED[2];
char timeSerieOLED_subsec[2];

bool show_Msg_OLED(float timeOfSerie){

  bool done = true; 

  int numberAlgarisms = 0;

  //float timeOfSerieInteger = 0;

  int timeOfSerieInteger = 0;

  timeOfSerieInteger = int(timeOfSerie);

  Serial.println(timeOfSerieInteger);

  if(abs(timeOfSerie)/10 < 0){
    numberAlgarisms = 1;
  }else{
    numberAlgarisms = 2;
  }

  //dtostrf(timeOfSerieInteger, numberAlgarisms, 0, timeSerieOLED);

  itoa(abs(timeOfSerieInteger), timeSerieOLED, 10); 

  //itoa(timeOfSerieInteger, timeSerieOLED, 10);   

  itoa((abs(timeOfSerie)-timeOfSerieInteger)*100, timeSerieOLED_subsec, 10); 

  int positionForDot = 0;

  int positionSubSeconds = 0;

  int positionSecSymbol = 0;

  int startNumbers = 0;

  startNumbers = 15;

//  if(abs(timeOfSerie)/10 < 0){
//    positionForDot = 60;
//  }else if(abs(timeOfSerie)/10 > 0){
//    positionForDot = 72;
//  }

  positionForDot = startNumbers + 50;

  positionSubSeconds = positionForDot + 10;
  positionSecSymbol = positionSubSeconds + 23;
  

  for(int i=0;i<20;i++){
        u8g2.firstPage(); 
        
        do{           
          u8g2.setFont(u8g2_font_ncenB14_tf);
          u8g2.drawStr(30, 15, "AT:"); 
          u8g2.drawStr(startNumbers, 35, timeSerieOLED);  //24
//          u8g2.drawStr(positionForDot, 35, ".");  
//          u8g2.drawStr(positionSubSeconds, 35, timeSerieOLED_subsec);
//          u8g2.drawStr(positionSecSymbol, 35, "s");
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  u8g2.begin();
  u8g2.enableUTF8Print();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(show_Msg_OLED(timex)){
    Serial.println("Presenting");
  }
}
