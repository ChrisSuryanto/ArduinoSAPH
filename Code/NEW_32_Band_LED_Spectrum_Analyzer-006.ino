// Modified code by Christian Suryanto, from (c) 2019 Shajeeb TM
// HAZI TECH
// Updated by Christian Suryanto
// Last update : Thursday, 2024/04/11, 23:20
// 

#include <arduinoFFT.h>
#include <MD_MAX72xx.h>
#include <EEPROM.h>

#include "narrow.h"

#include <ezButton.h>
//#include <SPI.h>


//LED Matrix
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW                     // Set display type  so that  MD_MAX72xx library treets it properly
#define CLK_PIN   13                                          // Clock pin to communicate with display
#define DATA_PIN  11                                          // Data pin to communicate with display
#define CS_PIN    10                                          // Control pin to communicate with display
#define MAX_DEVICES  4                                        // Total number display modules
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   // display object

//AUDIO FFT
#define SAMPLES 64                                           // Must be a power of 2
#define  xres 32                                              // Total number of  columns in the display, must be <= SAMPLES/2
#define  yres 8                                               // Total number of  rows in the display

float vReal[SAMPLES];
float vImag[SAMPLES];
char data_avgs[xres];
int  oldBarHeights[]                   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int yvalue;
byte displaycolumn , displayvalue;
int peaks[xres]                        = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long startPeakMillis[xres]    = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, 100, true);

//EZButton
ezButton buttonA(2);                                      // the number of the pushbutton pin
ezButton buttonB(3);                                      // the number of the pushbutton pin
const int SHORT_PRESS_TIME = 500; // 500 milliseconds
const int LONG_PRESS_TIME = 2000; // 500 milliseconds
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

//MENU
byte currMode = 0;
char *menuName[] = {"MODE", "LED", "GAIN", "PEAK"};
//const byte fontWidth = 3;
const byte fontWidth = 5;
int menuTimeOut = 2000;
unsigned long menuStartTime = 0;
bool menuActive = false;
bool peakOn = true;

byte displayMode = 0; 
byte LEDintensity = 5;
byte audioResponse = 20;                                        // put a value between 10 and 80. Smaller the number, higher the audio response

//SPECTRUM BAR Pattern
byte MY_ARRAY[]={0, 128, 192, 224, 240, 248, 252, 254, 255};    // default = standard pattern
byte MY_PEAK[]={0, 128, 64, 32, 16, 8, 4, 2, 1};                // peak pattern
unsigned long prevPeakMillis = 0;
uint16_t peakDecayTime = 100;
uint16_t peakHoldTime = 800;

byte eq0A[32] = {40, 50, 70, 70, 100, 100, 100, 60, 60, 100, 130, 100, 120, 120, 120, 120, 90, 100, 100, 100, 100, 100, 170, 170, 140, 140, 200, 200, 190, 190, 220, 220};
byte eq1A[16] = {45, 70, 100, 80, 80, 115, 120, 120, 95, 100, 100, 170, 140, 200, 190, 220};
byte eq2A[11] = {57, 90, 80, 115, 120, 120, 95, 100, 130, 170, 205};

//For Printing Text
#define   CHAR_SPACING  1 // pixels between characters


//Built-in Arduino RESET functions
void(* resetFunc) (void) = 0;


void setup()
{
    //(memory address, value), RUN THIS FOR THE FIRST TIME
//    EEPROM.update(1,0);                                           //displayMode 0
//    EEPROM.update(2,4);                                           //LEDintensity 4
//    EEPROM.update(3,22);                                          //audioResponse 22
//    EEPROM.update(4,1);                                           //peakOn true
    //analogReference(EXTERNAL);
    currMode = 0;
    
    buttonA.setDebounceTime(1);
    buttonB.setDebounceTime(1);

    Serial.begin(9600);

    displayMode   = EEPROM.read(1);
    LEDintensity  = EEPROM.read(2);
    audioResponse = EEPROM.read(3);
    peakOn        = EEPROM.read(4);

    ADCSRA = 0b11100101;                                          // set ADC to free running mode and set pre-scalar to 32 (0xe5)
    ADMUX = 0b00000000;                                           // use pin A0 and external voltage reference
    
    mx.begin();                                                   // initialize display
    mx.control(MD_MAX72XX::INTENSITY, LEDintensity);              // set LED intensity
    //mx.setFont(font3x5CAP);
    //mx.setFont(font3x7);
    mx.setFont(fontNarrow);
    
    delay(50);                                                    // wait to get reference voltage stabilized
    
    printText(true, 0, 0, MAX_DEVICES-1, "v.039");
    
    delay(2000);
    mx.clear();
}



void loop()
{
   unsigned long currPeakMillis = millis();
   // ++ Sampling
    byte numData;
    
    buttonA.loop();
    buttonB.loop();

    for(byte i=0; i<SAMPLES; i++)
    {
        while(!(ADCSRA & 0x10));                                    // wait for ADC to complete current conversion ie ADIF bit set
        ADCSRA = 0b11110101;                                       // clear ADIF bit so that ADC can do next operation (0xf5)
        ADMUX = 0b00000000;

        int value = ADC - 512 ;                                     // Read from ADC and subtract DC offset caused value
        vReal[i] = value / 8;
        vImag[i] = 0;                         
    }
    
    // -- Sampling
    //++ FFT
    FFT.dcRemoval();
    //FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    //FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  /* Weigh data */

    FFT.compute(FFTDirection::Forward); /* Compute FFT */

    FFT.complexToMagnitude(); /* Compute magnitudes */
    // -- FFT

// re-mapping data - Customize by Christian Suryanto ///
    byte b = 2;
    switch (displayMode)
    {
      case 0 :
      {
        numData = 32;
        data_avgs[0] = (vReal[2] + vReal[3])/2;
        data_avgs[1] = (vReal[3] + vReal[4] + vReal[5] )/3;
        data_avgs[2] = (vReal[5] + vReal[6])/2;
        data_avgs[3] = vReal[3];
        data_avgs[4] = vReal[4];
        data_avgs[5] = vReal[5];
        data_avgs[6] = vReal[6];
        data_avgs[7] = vReal[7];
        data_avgs[8] = vReal[8];
        data_avgs[9] = vReal[9];
        data_avgs[10] = vReal[10];
        data_avgs[11] = vReal[11];
        data_avgs[12] = vReal[12];
        data_avgs[13] = vReal[13];
        data_avgs[14] = vReal[14];
        data_avgs[15] = vReal[15];
        data_avgs[16] = vReal[16];
        data_avgs[17] = vReal[17];
        data_avgs[18] = vReal[18];
        data_avgs[19] = vReal[19];
        data_avgs[20] = vReal[20];
        data_avgs[21] = vReal[21];
        data_avgs[22] = vReal[22];
        data_avgs[23] = vReal[23];
        data_avgs[24] = (vReal[16] + vReal[17] + vReal[24])/3;
        data_avgs[25] = (vReal[17] + vReal[18] + vReal[25])/3;
        data_avgs[26] = (vReal[18] + vReal[19] + vReal[26])/3;
        data_avgs[27] = (vReal[19] + vReal[20] + vReal[27])/3;
        data_avgs[28] = (vReal[20] + vReal[21] + vReal[28])/3;
        data_avgs[29] = (vReal[21] + vReal[22] + vReal[29])/3;
        data_avgs[30] = (vReal[22] + vReal[23] + vReal[30])/3;
        data_avgs[31] = (vReal[23] + vReal[24] + vReal[31])/3;
      }
      break;

      case 1 :
      {
        numData = 16;
//        data_avgs[0] = (vReal[2] + vReal[2] + vReal[3] + vReal[3])/(Gain*0.4*b);
//        data_avgs[1] = (vReal[7] + vReal[3] + vReal[4] + vReal[5] + vReal[6])/(Gain*0.2*b);
//        data_avgs[2] = (vReal[7] + vReal[8])/(Gain*0.15*b);
//        data_avgs[3] = (vReal[9] + vReal[10] + vReal[11])/(Gain*0.15*b);
//        data_avgs[4] = (vReal[11] + vReal[12] + vReal[13])/(Gain*0.15*b);
//        data_avgs[5] = (vReal[14] + vReal[15])/(Gain*0.15*b);
//        data_avgs[6] = (vReal[16] + vReal[17])/(Gain*0.1*b);
//        data_avgs[7] = (vReal[18] + vReal[18])/(Gain*0.1*b);
//        data_avgs[8] = (vReal[18] + vReal[19] + vReal[20])/(Gain*0.1*b);
//        data_avgs[9] = (vReal[21] + vReal[22])/(Gain*0.1*b);
//        data_avgs[10] = (vReal[23] + vReal[23])/(Gain*0.1*b);
//        data_avgs[11] = (vReal[24] + vReal[25])/(Gain*0.1*b);
//        data_avgs[12] = (vReal[25] + vReal[26] + vReal[27])/(Gain*0.1*b);
//        data_avgs[13] = (vReal[28] + vReal[28])/(Gain*0.1*b);
//        data_avgs[14] = (vReal[28] + vReal[29] + vReal[30])/(Gain*0.1*b);
//        data_avgs[15] = (vReal[29] + vReal[29])/(Gain*0.1*b);

        data_avgs[0] = (vReal[2] + vReal[3] + vReal[4] + vReal[5])/4;
        data_avgs[1] = (vReal[3] + vReal[4] + vReal[5] + vReal[6])/4;
        data_avgs[2] = (vReal[4] + vReal[5])/2;
        data_avgs[3] = (vReal[6] + vReal[7])/2; //
        data_avgs[4] = (vReal[8] + vReal[9])/2; //
        data_avgs[5] = (vReal[10] + vReal[11])/2;
        data_avgs[6] = (vReal[12] + vReal[13])/2;
        data_avgs[7] = (vReal[14] + vReal[15])/2;
        data_avgs[8] = (vReal[16] + vReal[17])/2;
        data_avgs[9] = (vReal[18] + vReal[19])/2;
        data_avgs[10] = (vReal[20] + vReal[21])/2;
        data_avgs[11] = (vReal[22] + vReal[23])/2;
        data_avgs[12] = (vReal[16] + vReal[17] + vReal[24] + vReal[17] + vReal[18] + vReal[25])/6;
        data_avgs[13] = (vReal[18] + vReal[19] + vReal[26] + vReal[19] + vReal[20] + vReal[27])/6;
        data_avgs[14] = (vReal[20] + vReal[21] + vReal[28] + vReal[21] + vReal[22] + vReal[29])/6;
        data_avgs[15] = (vReal[22] + vReal[23] + vReal[30] + vReal[23] + vReal[24] + vReal[31])/6;
      }
      break;

      case 2 :
      {
        numData = 11;
//        data_avgs[0] = (vReal[2] + vReal[2])/(Gain*0.2*b);
//        data_avgs[1] = (vReal[2] + vReal[2] + vReal[3] + vReal[61]) / (Gain*0.3*b);
//        data_avgs[2] = (vReal[3] + vReal[61] + vReal[1] + vReal[61] + vReal[2] + vReal[61] + vReal[3] + vReal[61])/(Gain*1.1*b);
//        data_avgs[3] = (vReal[2] + vReal[61] + vReal[3] + vReal[61] + vReal[4] + vReal[60])/(Gain*0.65*b);
//        data_avgs[4] = (vReal[5] + vReal[59] + vReal[6] + vReal[58] + vReal[7] + vReal[57])/(Gain*0.65*b);
//        data_avgs[5] = (vReal[8] + vReal[56] + vReal[9] + vReal[55] + vReal[10] + vReal[54] + vReal[11] + vReal[53])/(Gain*0.65*b);
//        data_avgs[6] = (vReal[12] + vReal[52] + vReal[13] + vReal[51] + vReal[14] + vReal[50] + vReal[15] + vReal[49])/(Gain*0.65*b);
//        data_avgs[7] = (vReal[16] + vReal[48] + vReal[17] + vReal[47] + vReal[18] + vReal[46])/(Gain*0.45*b);
//        data_avgs[8] = (vReal[19] + vReal[45] + vReal[20] + vReal[44] + vReal[21] + vReal[43] + vReal[22] + vReal[42])/(Gain*0.45*b);
//        data_avgs[9] = (vReal[23] + vReal[41] + vReal[24] + vReal[40] + vReal[25] + vReal[39] + vReal[26] + vReal[38] + vReal[27] + vReal[37])/(Gain*0.45*b);
//        data_avgs[10] = (vReal[28] + vReal[36] + vReal[29] + vReal[35] + vReal[29] + vReal[34] + vReal[29] + vReal[33])/(Gain*0.45*b);
        
        data_avgs[0] = (vReal[2] + vReal[3] + vReal[4] + vReal[5] + vReal[3] + vReal[4])/6;
        data_avgs[1] = (vReal[5] + vReal[6] + vReal[4] + vReal[5])/4;
        data_avgs[2] = (vReal[4] + vReal[5] + vReal[6])/3;
        data_avgs[3] = (vReal[7] + vReal[8] + vReal[9])/3;
        data_avgs[4] = (vReal[10] + vReal[11] + vReal[12])/3;
        data_avgs[5] = (vReal[13] + vReal[14] + vReal[15])/3;
        data_avgs[6] = (vReal[16] + vReal[17] + vReal[18])/3;
        data_avgs[7] = (vReal[19] + vReal[20] + vReal[21])/3;
        data_avgs[8] = (vReal[22] + vReal[23] + vReal[16] + vReal[17] + vReal[24])/5;
        data_avgs[9] = (vReal[17] + vReal[18] + vReal[25] + vReal[18] + vReal[19] + vReal[26] + vReal[19] + vReal[20] + vReal[27])/9;
        data_avgs[10] = (vReal[26] + vReal[19] + vReal[20] + vReal[27] + vReal[20] + vReal[21] + vReal[28] + vReal[21] + vReal[22] + vReal[29])/10;
        data_avgs[11] = (vReal[21] + vReal[22] + vReal[29] + vReal[22] + vReal[23] + vReal[30] + vReal[23] + vReal[24] + vReal[31])/9;
      }
      break;
    }
// re-mapping data - Customize by Christian Suryanto ///

//    // ++ re-arrange FFT result to match with no. of columns on display ( xres )
//    numData = 32;
//    int step = (SAMPLES/2)/xres; 
//    int c=0;
//    for(int i=0; i<(SAMPLES/2); i+=step)  
//    {
//      data_avgs[c] = 0;
//      for (int k=0 ; k< step ; k++)
//      {
//          data_avgs[c] = data_avgs[c] + vReal[i+k];
//      }
//      data_avgs[c] = data_avgs[c]/step; 
//      c++;
//    }
//    // -- re-arrange FFT result to match with no. of columns on display ( xres )


    for(byte i=0; i<numData; i++)
    {
        switch (displayMode)
        {
          case 0 : 
            data_avgs[i] = (data_avgs[i]) * (float)(eq0A[i]) / 100; //apply eq filter
          break;

          case 1 : 
            data_avgs[i] = (data_avgs[i]) * (float)(eq1A[i]) / 100; //apply eq filter
          break;

          case 2 : 
            data_avgs[i] = (data_avgs[i]) * (float)(eq2A[i]) / 100; //apply eq filter
          break;
        }

      data_avgs[i] = constrain(data_avgs[i],0,audioResponse);              // set max & min values for buckets
      data_avgs[i] = map(data_avgs[i], 0, audioResponse, 0, yres);         // remap averaged values to yres

      data_avgs[i] = ((oldBarHeights[i] * 1) + data_avgs[i]) / 2;

      yvalue=data_avgs[i];

//      peaks[i] = peaks[i]-1;    // decay by one light
//      if (yvalue > peaks[i]) 
//          peaks[i] = yvalue ;
//      yvalue = peaks[i];    
//      if (yvalue < 1) yvalue = 1;
//      displayvalue = MY_ARRAY[yvalue];

      if (peakOn)
      {
        if (currPeakMillis - prevPeakMillis >= peakDecayTime)
        {
          for (byte pos=0; pos < numData; pos++)
            if (peaks[pos] > 0 && (millis() - startPeakMillis[pos] > peakHoldTime)) peaks[pos]--;    // decay by one light
          prevPeakMillis = currPeakMillis;
        }
  
        if (yvalue > peaks[i])
        {
          peaks[i] = yvalue ;
          startPeakMillis[i] = millis();
        }
        //yvalue = peaks[i];    
        if (yvalue < 1) yvalue = 1;
        displayvalue = MY_ARRAY[yvalue] | MY_PEAK[peaks[i]];
      }
      else
      {
        //for (byte pos=0; pos < xres; pos++)
        //  peaks[pos] = 0;    // reset all peak value

        peaks[i] = peaks[i]-1;    // decay by one light
        if (yvalue > peaks[i]) 
            peaks[i] = yvalue ;
        yvalue = peaks[i];    
        if (yvalue < 1) yvalue = 1;
        displayvalue = MY_ARRAY[yvalue];
      }
      
      switch (displayMode)
      {
        case 0: 
        {
          displaycolumn=31-(1*i);
          if (!menuActive)
          {
              mx.setColumn(displaycolumn, displayvalue);                // for left to right
          }
        }
        break;

        case 1: 
        {
          displaycolumn=31-(2*i);
          if (!menuActive)
          {
              mx.setColumn(displaycolumn, displayvalue);                // for left to right
          }
        }
        break;

        case 2: 
        {
          displaycolumn=31-(3*i);
          if (!menuActive)
          {
              mx.setColumn(displaycolumn-1, displayvalue);                // for left to right
              mx.setColumn(displaycolumn, displayvalue);                // for left to right
          }
        }
        break;
      }
      oldBarHeights[i] = data_avgs[i];
     }
     
     if ((millis() - menuStartTime > menuTimeOut) && menuActive) // 2000 = menuTimeOut
     { 
        menuActive = false; mx.clear(); 
     }
     buttonCheck();                                       // check if button pressed to change display mode
}


void buttonCheck()
{
  byte stateA = buttonA.getState();
  byte stateB = buttonB.getState();

    if ((buttonA.isPressed()) ||  (buttonB.isPressed())) { pressedTime = millis(); menuStartTime = millis();}
    else 
    {
      if (buttonA.isReleased())
      { // button A is released
        releasedTime = millis();
        long pressDuration = releasedTime - pressedTime;
        if( pressDuration < SHORT_PRESS_TIME ) selectMenuMode(); else resetFunc(); //long press to reset
      }

      if (buttonB.isReleased() && menuActive)
      { // button B is released
        releasedTime = millis();
        //Serial.println(currMode);
        long pressDuration = releasedTime - pressedTime;
        //if( pressDuration < SHORT_PRESS_TIME )
        switch (currMode)
        {
          case 0 : //Mode Full / 1Bar / 2Bar
          { DisplayModeUp(); /*EEPROM.update(1, displayMode);*/ }
          break;
    
          case 1 : //LED intensity 0 - 15
          { LEDintensityUp(); /*EEPROM.update(2, LEDintensity);*/ }
          break;
    
          case 2 : //audio response
          { audioResponseUp(); /*EEPROM.update(3, audioResponse);*/ }
          break;

          case 3 : //toggle Peak
          { togglePeak(); /*EEPROM.update(4, peakOn);*/ }
          break;
        }
      }

      if (buttonB.isReleased() && !menuActive)
      { // button B is released when no active menu
        releasedTime = millis();
        //Serial.println(currMode);
        long pressDuration = releasedTime - pressedTime;
        if( pressDuration > SHORT_PRESS_TIME ) resetFunc(); //reset
      }
    }
}


void selectMenuMode()
{
  if (menuActive) {if (currMode < 3) currMode++; else currMode = 0;}
  mx.clear();
  menuActive = true; 
  printText(true, 0, 0, MAX_DEVICES-1, menuName[currMode]);
}



void DisplayModeUp()
{
  if (displayMode < 2) displayMode++;
  else displayMode = 0;
  mx.clear();  
  if (displayMode == 0) printText(true, 4, 0, MAX_DEVICES-1, "M : 0");
  if (displayMode == 1) printText(true, 4, 0, MAX_DEVICES-1, "M : 1");
  if (displayMode == 2) printText(true, 4, 0, MAX_DEVICES-1, "M : 2");
  //delay(2000);
  EEPROM.update(1, displayMode);
  //mx.clear();  
}


void LEDintensityUp()
{
  if (LEDintensity < 15) LEDintensity++; 
  else LEDintensity = 0;
  mx.clear();  
  if (LEDintensity == 0) printText(true, 4, 0, MAX_DEVICES-1, "L : 0");
  if (LEDintensity == 1) printText(true, 4, 0, MAX_DEVICES-1, "L : 1");
  if (LEDintensity == 2) printText(true, 4, 0, MAX_DEVICES-1, "L : 2");
  if (LEDintensity == 3) printText(true, 4, 0, MAX_DEVICES-1, "L : 3");
  if (LEDintensity == 4) printText(true, 4, 0, MAX_DEVICES-1, "L : 4");
  if (LEDintensity == 5) printText(true, 4, 0, MAX_DEVICES-1, "L : 5");
  if (LEDintensity == 6) printText(true, 4, 0, MAX_DEVICES-1, "L : 6");
  if (LEDintensity == 7) printText(true, 4, 0, MAX_DEVICES-1, "L : 7");
  if (LEDintensity == 8) printText(true, 4, 0, MAX_DEVICES-1, "L : 8");
  if (LEDintensity == 9) printText(true, 4, 0, MAX_DEVICES-1, "L : 9");
  if (LEDintensity == 10) printText(true, 4, 0, MAX_DEVICES-1, "L : 10");
  if (LEDintensity == 11) printText(true, 4, 0, MAX_DEVICES-1, "L : 11");
  if (LEDintensity == 12) printText(true, 4, 0, MAX_DEVICES-1, "L : 12");
  if (LEDintensity == 13) printText(true, 4, 0, MAX_DEVICES-1, "L : 13");
  if (LEDintensity == 14) printText(true, 4, 0, MAX_DEVICES-1, "L : 14");
  if (LEDintensity == 15) printText(true, 4, 0, MAX_DEVICES-1, "L : 15");
  mx.control(MD_MAX72XX::INTENSITY, LEDintensity);
  EEPROM.update(2, LEDintensity);
}



void audioResponseUp()
{
  if (audioResponse > 2) audioResponse-=2;
  else audioResponse = 40;

  mx.clear();  
  if (audioResponse == 0) printText(true, 4, 0, MAX_DEVICES-1, "G : 21");
  if (audioResponse == 2) printText(true, 4, 0, MAX_DEVICES-1, "G : 20");
  if (audioResponse == 4) printText(true, 4, 0, MAX_DEVICES-1, "G : 19");
  if (audioResponse == 6) printText(true, 4, 0, MAX_DEVICES-1, "G : 18");
  if (audioResponse == 8) printText(true, 4, 0, MAX_DEVICES-1, "G : 17");
  if (audioResponse == 10) printText(true, 4, 0, MAX_DEVICES-1, "G : 16");
  if (audioResponse == 12) printText(true, 4, 0, MAX_DEVICES-1, "G : 15");
  if (audioResponse == 14) printText(true, 4, 0, MAX_DEVICES-1, "G : 14");
  if (audioResponse == 16) printText(true, 4, 0, MAX_DEVICES-1, "G : 13");
  if (audioResponse == 18) printText(true, 4, 0, MAX_DEVICES-1, "G : 12");
  if (audioResponse == 20) printText(true, 4, 0, MAX_DEVICES-1, "G : 11");
  if (audioResponse == 22) printText(true, 4, 0, MAX_DEVICES-1, "G : 10");
  if (audioResponse == 24) printText(true, 4, 0, MAX_DEVICES-1, "G : 9");
  if (audioResponse == 26) printText(true, 4, 0, MAX_DEVICES-1, "G : 8");
  if (audioResponse == 28) printText(true, 4, 0, MAX_DEVICES-1, "G : 7");
  if (audioResponse == 30) printText(true, 4, 0, MAX_DEVICES-1, "G : 6");
  if (audioResponse == 32) printText(true, 4, 0, MAX_DEVICES-1, "G : 5");
  if (audioResponse == 34) printText(true, 4, 0, MAX_DEVICES-1, "G : 4");
  if (audioResponse == 36) printText(true, 4, 0, MAX_DEVICES-1, "G : 3");
  if (audioResponse == 38) printText(true, 4, 0, MAX_DEVICES-1, "G : 2");
  if (audioResponse == 40) printText(true, 4, 0, MAX_DEVICES-1, "G : 1");

  EEPROM.update(3, audioResponse);
}


void togglePeak()
{
  //if (displayMode < 2) displayMode++;
  peakOn = !peakOn;
  mx.clear();  
  if (peakOn) printText(true, 4, 0, MAX_DEVICES-1, "P : ON");
  else printText(true, 4, 0, MAX_DEVICES-1, "P : X");
  
  EEPROM.update(4, peakOn);
}


void printText(bool isCentered, uint8_t colFix, uint8_t modStart, uint8_t modEnd, char *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint8_t   state = 0;
  uint8_t    curLen;
  uint16_t  showLen;
  uint8_t   cBuf[8];
  byte colToCenter;
  //byte colFix;
  int16_t   col = ((modEnd + 1) * COL_SIZE) - 1;
  byte txtLen = 0;

  String txtStr = String(pMsg);
  txtLen = txtStr.length();
  if (isCentered) colToCenter = ((8*MAX_DEVICES)-(txtLen*(fontWidth+1)))/2; else colToCenter = 0;
  colToCenter+=colFix;
  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do     // finite state machine to print the characters in the space available
  {
    switch(state)
    {
      case 0: // Load the next character from the font table
        // if we reached end of message, reset the message pointer
        if (*pMsg == '\0')
        {
          showLen = col - (modEnd * COL_SIZE);  // padding characters
          state = 2;
          break;
        }

        // retrieve the next character form the font file
        showLen = mx.getChar(*pMsg++, sizeof(cBuf)/sizeof(cBuf[0]), cBuf);
        curLen = 0;
        state++;
        // !! deliberately fall through to next state to start displaying

      case 1: // display the next part of the character
        //col = col - colToCenter;
        mx.setColumn((col--) - colToCenter, cBuf[curLen++]);

        // done with font character, now display the space between chars
        if (curLen == showLen)
        {
          showLen = CHAR_SPACING;
          state = 2;
        }
        break;

      case 2: // initialize state for displaying empty columns
        curLen = 0;
        state++;
        // fall through

      case 3: // display inter-character spacing or end of message padding (blank columns)
        mx.setColumn((col--) - colToCenter, 0);
        curLen++;
        if (curLen == showLen)
          state = 0;
        break;

      default:
        col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}
