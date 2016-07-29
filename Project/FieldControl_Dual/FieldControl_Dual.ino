//for debug 0:off 1:on
#define debug 0

//General variables
double db_RawCndct = 0.0;
double db_Cndct = 0.0;
double db_RawTemp = 0.0;
double db_Temp = 0.0;
double db_RawDensity = 0.0;
double db_Density = 0.0;
double db_RawPH = 0.0;
double db_PH = 0.0;


//from DHT sensor
double db_ATemp = 0.0;
double db_Hmdty = 0.0;
unsigned int ramp = 0; //ramp count for test

#include <Timer.h>
Timer timer_getDHT, timer_sendJsonData, timer_recevieFromAdk, timer_showDebug, timer_syncAnalogShield;


 

//Temperature,Humidity Sensor
#include <dht.h>
dht DHT;
#define DHT22_PIN 23

void getDHT()
{
  int chk = DHT.read22(DHT22_PIN);
  switch (chk)
  {
    case DHTLIB_OK:  
        //Serial.print("OK,\t");
        db_Hmdty = DHT.humidity;
        db_ATemp = DHT.temperature;
    break;
    case DHTLIB_ERROR_CHECKSUM: 
        //Serial.print("Checksum error,\t"); Error code:997
        db_Hmdty = 997;
        db_ATemp = 997;
    break;
    case DHTLIB_ERROR_TIMEOUT: 
        // Serial.print("Time out error,\n"); Error code:998
        db_Hmdty = 998;
        db_ATemp = 998;
    break;
    default: 
     //Serial.print("Unknown error,\t");  Error code:999
        db_Hmdty = 999;
        db_ATemp = 999;
    break;
  }
  // DISPLAY DATA 
  if (debug==1){
   String tmpDisplay = "Hmdty:"+ D2string(db_Hmdty,2)+" || "+"ATemp:"+ D2string(db_ATemp,2);
   Serial.println(tmpDisplay);
  }
}



// Accessory descriptor. It's how Arduino identifies itself to Android.
#include <adk.h>
char descriptionName[] = "FieldController"; 
char modelName[] = "UDOO_DUAL";           // your Arduino Accessory name (Need to be the same defined in the Android App)
char manufacturerName[] = "Hwaling";     // manufacturer (Need to be the same defined in the Android App)
// Make up anything you want for these
char versionNumber[] = "1.0";            // version (Need to be the same defined in the Android App)
char serialNumber[] = "1";
char url[] = "http://www.hwaling-tech.com";      // If there isn't any compatible app installed, Android suggest to visit this url
USBHost Usb;
ADK adk(&Usb, manufacturerName, modelName, descriptionName, versionNumber, url, serialNumber);
#define RCVSIZE 256
uint8_t buf[RCVSIZE];
//char buffer[RCVSIZE];
uint32_t bytesRead = 0; //serial  buffer length
char adk_buffer[RCVSIZE];
#include <ArduinoJson.h>
double db_ParamA[] = {0.0, 0.0, 0.0};

void sendJsonData(){
  StaticJsonBuffer<RCVSIZE> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ATemp"].set(db_ATemp,2);
  json["Hmdty"].set(db_Hmdty,2);
  json["RawCndct"].set(db_RawCndct,2); 
  json["RawTemp"].set(db_RawTemp,2);
  /*
  json["RawCndct"].set(db_RawCndct,2);
  json["Conductivity"].set(db_Cndct,2);
  json["RawTemp"].set(db_RawTemp,2);
  json["Temperature"].set(db_Temp,2);
  */
  char buffer[RCVSIZE];
  json.printTo(buffer, sizeof(buffer));
  String strbuffer(buffer);
  strbuffer.trim();
  if (debug==1){
  Serial.println(strbuffer);
  }

  sendToAdk(strbuffer);
  //object["key4"].set(3.1415, 4);  // 4 digits "3.1415"
}

//void sendToAdk(char textToSend[]) {   
void sendToAdk(String textToSend) {  
     if (adk.isReady()){ 
     int len = textToSend.length();
     len++;
     textToSend.toCharArray(adk_buffer,len);
     Serial.println(textToSend);
     Serial.println(adk_buffer);
     len--;
     adk.write(len, (uint8_t*)adk_buffer); //sizeof can't work well, Calen change to RCVSIZE 2014/07/27
     } 
     // clear array buffer
     for( int i = 0; i < sizeof(adk_buffer);  ++i ){
      adk_buffer[i] = (char)0;
     }
     
}

void recevieFromAdk() {     
    //write implementation code here
}

void recevieJsonData(){
    if (adk.isReady()) {
      adk.read(&bytesRead, RCVSIZE, buf);// read data into buf variable
      if (bytesRead > 0){
         StaticJsonBuffer<RCVSIZE> jsonBuffer;
         JsonObject& root = jsonBuffer.parseObject((char*)buf);       
         if (root.success())
         {
          // String strTemp ="";
           for (int i=0; i <= 2; i++){
              db_ParamA[i] = root["Param1"][i];
              //strTemp = D2string(db_ParamA[i],2);
              //Serial.println("ArduinoJSON:ParamA" + String((int)i)+":"+ strTemp);
               }
           /*
            long        time      = root["time"];
            double      latitude  = root["data"][0];
            double      longitude = root["data"][1];  
          */     
         }else
         { 
          //Write Error code here       
         }
      }  // End of bytesRead > 0
    } // End of adk.read 
}



//SPI for AnalogShield
#include <SPI.h>	//required for Arduino DUE 
//#include <analogShield.h>   //Include to use analog shield.
  #define adccs   2
  #define adcbusy   3  
  #define syncPin   5
  #define ldacPin   6
  #define DIFF_MODE true
  #define SIGL_MODE false
  int shieldMode;

  //AnalogShield Information
  #define Board_No  D593741
  #define zeroA0   32755 
  #define zeroA1   32780
  #define zeroA2   32746
  #define zeroA3   32672
  #define zeroA01  32747
  #define zeroA23  32846

  int int_zeroA0 = 32767-zeroA0;
  int int_zeroA1 = 32767-zeroA1;
  int int_zeroA2 = 32767-zeroA2;
  int int_zeroA3 = 32767-zeroA3;
  int int_zeroA01 = 32767-zeroA01;
  int int_zeroA23 = 32767-zeroA23;
  
//Moving Average 
#include <AnalogSmooth.h>
#define MA_Num 20
AnalogSmooth asA0 = AnalogSmooth(MA_Num);
AnalogSmooth asA1 = AnalogSmooth(MA_Num);
AnalogSmooth asA2 = AnalogSmooth(MA_Num);
AnalogSmooth asA3 = AnalogSmooth(MA_Num);
AnalogSmooth asA01 = AnalogSmooth(MA_Num);
AnalogSmooth asA23 = AnalogSmooth(MA_Num);
unsigned int uint_ADCvalue0 = 0, uint_ADCvalue1=0, uint_ADCvalue2=0, uint_ADCvalue3=0, uint_ADCvalue01=0, uint_ADCvalue23=0;

void syncAnalogShield(){
  uint_ADCvalue0 = ADCread(0, SIGL_MODE);
  uint_ADCvalue1 = ADCread(1, SIGL_MODE);
  uint_ADCvalue2 = ADCread(2, SIGL_MODE);
  uint_ADCvalue3 = ADCread(3, SIGL_MODE);
  uint_ADCvalue01 = ADCread(0, DIFF_MODE);
  uint_ADCvalue23 = ADCread(2, DIFF_MODE);
  ramp++;
  if (debug==1) {
  Serial.println(ramp);
  String Str1 = String(uint_ADCvalue0) +" || "+ String(uint_ADCvalue1) +" || "+ String(uint_ADCvalue2) +" || "+ String(uint_ADCvalue3);
  Serial.println(Str1);
  String Str2 = String(uint_ADCvalue01) +" || "+ String(uint_ADCvalue23);
  Serial.println(Str2);
  }
  
  float analogA0 = static_cast<float>(uint_ADCvalue0);  //unsigned int to float
  float analogA1 = static_cast<float>(uint_ADCvalue1);
  float analogA2 = static_cast<float>(uint_ADCvalue2);
  float analogA3 = static_cast<float>(uint_ADCvalue3);
  float analogA01 = static_cast<float>(uint_ADCvalue01);
  float analogA23 = static_cast<float>(uint_ADCvalue23);
  
  //float analogSmooth = as.smooth(analog);
  analogA0 = asA0.smooth(analogA0); 
  analogA1 = asA1.smooth(analogA1);
  analogA2 = asA2.smooth(analogA2);
  analogA3 = asA3.smooth(analogA3);
  analogA01 = asA01.smooth(analogA01);
  analogA23 = asA23.smooth(analogA23);

  db_RawCndct = ADConverter(0,1000,analogA0,int_zeroA0);  //analogA0  set to Cndct
  db_RawDensity = ADConverter(0.5,1.5,analogA1,int_zeroA1); //analogA1 set to Density
  db_RawTemp  = ADConverter(0,100,analogA2,int_zeroA2); //analogA2 set to Temp
  db_RawPH = ADConverter(0,1000,analogA3,int_zeroA3); //analogA3 set to PH

  
 // Serial.println(analogSmooth);
 if (debug==1) {
  String Str3 = D2string(analogA0,0) + ";" + D2string(analogA1,0) +";"+ D2string(analogA2,0) + ";" + D2string(analogA3,0); 
  Serial.println(Str3);
  String Str4 = D2string(analogA01,0)+";"+D2string(analogA23,0);
  Serial.println(Str4);  
 }
  unsigned int aa = ramp%65535;
  //if (ramp%100==1){AQwrite(0,aa);}
  //if (ramp%100==51){AQwrite(1,34077);}
  //if (ramp%20==11){AQwrite(2,40000);}
  //if (ramp%20==16){AQwrite(3,50000);}   
}

void showDebug()
{
  String debugLog = D2string(db_RawCndct,2)+" || "+ D2string(db_RawDensity,2)+" || " + D2string(db_RawTemp,2)+" || "+D2string(db_RawPH,2);
  Serial.println(debugLog);
}


void setup()
{
  
  //no setup
  Serial.begin(115200);
  timer_getDHT.every(1000, getDHT);
  timer_sendJsonData.every(1000,sendJsonData);
  timer_showDebug.every(5000,showDebug);
  timer_syncAnalogShield.every(50,syncAnalogShield);
  
   //ADC pins setting
  pinMode(adccs, OUTPUT);
  digitalWrite(adccs, HIGH);
  pinMode(adcbusy, INPUT);
    
    //DAC pins setting
  pinMode(syncPin, OUTPUT);
  pinMode(ldacPin, OUTPUT);
  digitalWrite(syncPin,HIGH);  
    digitalWrite(ldacPin,LOW);     
    shieldMode = 0;
    
   SPI.begin();
  
   //Initial DAC value = 0; -5V~+5V (0~65535)
   DACwrite(0,32767);
   DACwrite(1,32767);
   DACwrite(2,32767);
   DACwrite(3,32767);
}

void loop() {
  Usb.Task(); //call android activity to connect.
  timer_getDHT.update();  //1000
  timer_sendJsonData.update(); //1000
  timer_syncAnalogShield.update(); //50
  timer_showDebug.update(); 
  //recevieJsonData();
}




void setChannelAndModeByte(byte channel, bool mode){
    //control byte
    //S - sentinel, always 1
    //A2 - channel select
    //A1 - channel select
    //A0 - channel select
    //- N/C
    //Single / Diff pair
    //PD1 - power down mode
    //PD0 - power down mode
    byte control = B10000010; //default to channel 1 '001'
    
    //channel mask
    if(channel == 3){
      control = control | B11100000;
    }
    
    else if(channel == 2){
      control = control | B10100000;
    }
    
    else if(channel == 1){
      control = control | B11010000;
    }
    
    else if(channel == 0){
      control = control | B00010000;
    }
    
    //differential mode active
    if(mode){
      control = control & B11111011;
    }
    
    else{
      control = control | B00000100;
    }

     SPI.transfer(control);
     return;
}


  unsigned int ADCread(int channel, bool mode) {
    // initialize SPI:
    if(shieldMode != 2){
        SPI.setDataMode(SPI_MODE3);       
      shieldMode = 2;
    }

    digitalWrite(adccs,LOW);
    setChannelAndModeByte(channel, mode); 
    digitalWrite(adccs,HIGH);
    //wait for busy signal to rise. If it lasts a while, try resending.
    while(digitalRead(adcbusy) == 0); //wait for pin 3 to == 0 , 2016.07.26 Calen Added
    digitalWrite(adccs,LOW);
    //collect data
    byte high = SPI.transfer(0x00);
    byte low = SPI.transfer(0x00);    
    //release chip select
    digitalWrite(adccs,HIGH);
    //compile the result into a 32 bit integer.
    int result;
      result = (int)high<<24;
    result+= low<<16;
    //make into an unsigned int for compatibility with the DAC used on the analog shield.
    if(result < 0)
    {
      result = result >> 16;
      result &= 0x7FFF;
    }
    else
    {
      result = result >> 16;
      result |= 0x8000;
    }
    return result;
  }

void DACwrite(int channel, unsigned int value){
    if(shieldMode != 1)
    {
       // SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(SPI_MODE1);     
      shieldMode = 1;
    }
    byte high = value >> 8;//highByte(value);
    byte low = value & 0x00FF;//lowByte(value);
    // take the SS pin low to select the chip:
    digitalWrite(syncPin,LOW);
    digitalWrite(ldacPin,LOW);
    //  send in the address and value via SPI:
    byte call = 0x10;
    if(channel == 1)
      call = 0x12;
    
    else if(channel == 2)
      call = 0x14;
    
    else if(channel == 3)
      call = 0x16;
    
    //send command byte
    SPI.transfer(call);

    //send data
    SPI.transfer(high);
    SPI.transfer(low);
    // take the SS pin high to de-select the chip:
    digitalWrite(syncPin,HIGH);
}

double ADConverter(double db_StartValue, double db_RangeValue,double db_RawValue, int offset){
  double result = 0.0;
  db_RawValue= db_RawValue + offset;
  result = db_RawValue/65536.0;  //
  result = result*db_RangeValue;
  result = result + db_StartValue;
  return result;
}
    
String D2string(double input,int decimalPlaces){
  String strTemp ="";
  if(decimalPlaces!=0){
    strTemp = String((int)(input*pow(10,decimalPlaces)));
    if(abs(input)<1){
      if(input>0)
      strTemp = "0"+strTemp;
      else if(input<0)
      strTemp = strTemp.substring(0,1)+"0"+strTemp.substring(1);
   }
  return strTemp.substring(0,strTemp.length()-decimalPlaces)+"."+strTemp.substring(strTemp.length()-decimalPlaces);
} 
  else {
    return String((int)input);
  }
}
