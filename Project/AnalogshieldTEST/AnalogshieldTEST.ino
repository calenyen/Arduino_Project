
/************************************************************************/
/*                                                                      */
/*  ramp.ino  --  Ramp Demo                                             */
/*                                                                      */
/************************************************************************/
/*  Author:  William Esposito                                           */
/*  Copyright 2013, Stanford University                                 */
/************************************************************************/
/*
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
/************************************************************************/
/*  Module Description:                                                 */
/*                                                                      */
/*  This example will put out a linear ramp on output 0.  The 16 bit    */
/*  unsigned integer will roll over from +5V back to -5V.               */                                       
/*  On a scope, it should look something like:                          */
/*          -/|/|/|/|/|/|/|/|-                                          */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*  09/01/2013 (W. Esposito): created                                   */
/*  05/28/2014 (MWingerson): Updated for ChipKIT and release            */
/*  06/01/2015 (GCone): Changed order of includes for DUE support       */
/*                                                                      */
/************************************************************************/

/************************************************************************/
/*  Board Support:                                                      */
/*                                                                      */
/*      Arduino UNO                                                     */
/*      Arduino DUE                                                     */
/*      ChipKIT UNO32                                                   */
/*      ChipKit UC32                                                    */
/*                                                                      */
/************************************************************************/

/*
For Arduino DUE using Arduino IDE, SPI needs to be included before 
analogShield.  analogShield calls SPI.begin during constructor 
thus SPI needs to be constructed before analogShield
changing the order of includes influenced the link order and 
thus the order of global variable construction.
*/
#include <SPI.h>	//required for ChipKIT and Arduino DUE 
//#include <analogShield.h>   //Include to use analog shield.


  #define adccs   2
  #define adcbusy   3
  
  #define syncPin   5
  #define ldacPin   6
  #define DIFF_MODE ture
  #define SIGL_MODE false
  int shieldMode;

#include <AnalogSmooth.h>
AnalogSmooth as = AnalogSmooth();
AnalogSmooth as20 = AnalogSmooth(20);



void setup()
{
  //no setup

  
  Serial.begin(115200);
    pinMode(adccs, OUTPUT);
    digitalWrite(adccs, HIGH);
    pinMode(adcbusy, INPUT);
    
    //DAC pins
    pinMode(syncPin, OUTPUT);
    pinMode(ldacPin, OUTPUT);
    digitalWrite(syncPin,HIGH);  
    digitalWrite(ldacPin,LOW);
      
    shieldMode = 0;


   SPI.begin();

  //SPI.end(); 
   delay(30000);
   AQwrite(0,0);
   AQwrite(1,0);
   AQwrite(2,0);
   AQwrite(3,32767);
}

unsigned int ramp = 0;
unsigned int AIvalue0 = 0, AIvalue1=0, AIvalue2=0, AIvalue3=0, AIvalue01=0, AIvalue23=0;
bool b_adcbusy;

void loop() {
  // put your main code here, to run repeatedly: 
 
  AIvalue0 = AIread(0, SIGL_MODE);
 
  AIvalue1 = AIread(1, SIGL_MODE);
  
  AIvalue2 = AIread(2, SIGL_MODE);
  
  AIvalue3 = AIread(3, SIGL_MODE);

/*
  AIvalue01 = AIread(0, DIFF_MODE);

  AIvalue23 = AIread(2, DIFF_MODE);
*/
  //myRA01.addValue(AIvalue01);
  //myRA23.addValue(AIvalue23);
  /*
  Serial.println("calling setChannelAndModebyte"); 
  digitalWrite(adccs,LOW);
  byte control = B10000010;
  control = control | B00010000;
  //control = control | B00000100; // Not differential mode 
  control = control & B11111011; // differential mode
  SPI.transfer(control);
  digitalWrite(adccs,HIGH); 

  //b_adcbusy = digitalRead(adcbusy);
  
  Serial.print("adcbusy:");
  Serial.println(digitalRead(adcbusy));  
 */
 /*
  while( b_adcbusy == 1); //wait for pin 3 to == 0
  {
    b_adcbusy = digitalRead(adcbusy);
  }
  */

  /*
  digitalWrite(adccs,LOW);
    //collect data
    byte high = SPI.transfer(0x00);
    byte low = SPI.transfer(0x00);    
    //release chip select
  digitalWrite(adccs,HIGH);
  
  Serial.println("wait for a while");
*/
  /*
   
    AIvalue = (int)high<<24;
    AIvalue+= low<<16;
    //make into an unsigned int for compatibility with the DAC used on the analog shield.
    if(AIvalue < 0)
    {
      AIvalue = AIvalue >> 16;
      AIvalue &= 0x7FFF;
    }
    else
    {
      AIvalue = AIvalue >> 16;
      AIvalue |= 0x8000;
    }
    */

/*
    AIvalue = (int)high<<8;
    AIvalue+= low;
  */
  /*
  b_adcbusy = digitalRead(adcbusy);
  while(b_adcbusy == 0){
    b_adcbusy = digitalRead(adcbusy);
    Serial.println(" abcbusy = false");
    delay(100);
  }
  */
  //int var = 0;
  /*
  while(var < 10){
   // do something repetitive 200 times
   var++;
  }
  */
   ramp++;
  Serial.println(ramp);
  String Str1 = String(AIvalue0) +" || "+ String(AIvalue1) +" || "+ String(AIvalue2) +" || "+ String(AIvalue3);
  Serial.println(Str1);
 // String Str2 = String(AIvalue01) +" || "+ String(AIvalue23);
  //double RA01 = myRA01.getAverage();
  //double RA23 = myRA23.getAverage();
  //String Str2 = D2string(RA01,0) +" || "+ D2string(RA23,0);
  //Serial.println(Str2);
  //setChannelAndModeByte(0,false);
  float analog = static_cast<float>(AIvalue01);  //unsigned int to float
  float analogSmooth = as.smooth(analog);
  float analogSmooth20 = as20.smooth(analog);
  Serial.println(analogSmooth);
  Serial.println(analogSmooth20);
  unsigned int aa = ramp%65535;
  if (ramp%100==1){AQwrite(0,aa);}
  if (ramp%100==51){AQwrite(1,34077);}
  //if (ramp%20==11){AQwrite(2,40000);}
  //if (ramp%20==16){AQwrite(3,50000);}
  delay(100);
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


  unsigned int AIread(int channel, bool mode) {
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

void AQwrite(int channel, unsigned int value){
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
