/*
KISS FC OSD v1.0
By Felix Niessen (felix.niessen@googlemail.com)
for Flyduino.net

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>
*/




// CONFIGURATION
//=========================================================================================================================
#define NICKNAME "SKYGREG"

// video system
//=============================
#define PAL
//#define NTSC

// MAX7456 Charset (change if you get sensless signs)
//=============================
#define USE_MAX7456_ASCII
//#define USE_MAX7456_MAXIM

// motors magnepole count (to display the right RPMs)
//=============================
#define MAGNETPOLECOUNT 14 // 2 for ERPMs

// Filter for ESC datas (higher value makes them less erratic) 0 = no filter, 20 = very strong filter
//=============================
#define ESC_FILTER 10

// displayed datas
//=============================
#define DISPLAY_NICKNAME
#define DISPLAY_TIME
#define DISPLAY_RC_THROTTLE
#define DISPLAY_COMB_CURRENT
#define DISPLAY_LIPO_VOLTAGE
#define DISPLAY_MA_CONSUMPTION
//#define DISPLAY_ESC_KRPM
//#define DISPLAY_ESC_CURRENT
#define DISPLAY_ESC_TEMPERATURE

// displayed datas in reduced mode
//=============================
#define RED_DISPLAY_NICKNAME
#define RED_DISPLAY_TIME
//#define RED_DISPLAY_RC_THROTTLE
//#define RED_DISPLAY_COMB_CURRENT
#define RED_DISPLAY_LIPO_VOLTAGE
#define RED_DISPLAY_MA_CONSUMPTION
//#define RED_DISPLAY_ESC_KRPM
//#define RED_DISPLAY_ESC_CURRENT
#define RED_DISPLAY_STATS
#define RED_DISPLAY_ESC_TEMPERATURE

//#define RED_DISPLAY_MAXC
// configure max C with Lipo capacity:
#if defined(RED_DISPLAY_MAXC)
const int LIPOS[] = {1300,1500,1000};
#endif

//#define RED_DISPLAY_DEBUG



// reduced mode channel config
//=============================
#define RED_MODE_AUX_CHAN 0 // 0-4, 0 = none

#define RED_ON_AUX_LOW
//#define RED_ON_AUX_MID
//#define RED_ON_AUX_HIGH

// internals
//=============================
#define KISSOSDVERS "Kiss OSD - V1.3"
// debug
#ifdef RED_DISPLAY_DEBUG
#undef RED_DISPLAY_NICKNAME
#undef DISPLAY_NICKNAME
#undef RED_DISPLAY_STATS
#undef RED_DISPLAY_TIME
//#define RED_DISPLAY_RC_THROTTLE
#endif
// END OF CONFIGURATION
//=========================================================================================================================

#include <SPI.h>
#include <MAX7456.h>

const byte osdChipSelect             =            6;
const byte masterOutSlaveIn          =            MOSI;
const byte masterInSlaveOut          =            MISO;
const byte slaveClock                =            SCK;
const byte osdReset                  =            2;

MAX7456 OSD( osdChipSelect );


static char clean[30];

void setup(){
  uint8_t i = 0;
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV2 ); 
  //OSD.begin();
  #if defined(PAL)
    OSD.begin(28,15,0);
    OSD.setTextOffset(-1,-6);
    OSD.setDefaultSystem(MAX7456_PAL);
  #endif
  #if defined(NTSC)
    OSD.begin(MAX7456_COLS_N1,MAX7456_ROWS_N0);
    OSD.setDefaultSystem(MAX7456_NTSC);
  #endif
  OSD.setSwitchingTime( 5 );   
  #if defined(USE_MAX7456_ASCII)
    OSD.setCharEncoding( MAX7456_ASCII );  
  #endif 
  #if defined(USE_MAX7456_MAXIM)
    OSD.setCharEncoding( MAX7456_MAXIM );  
  #endif 
  OSD.display(); 
  
  //clean used area
  for(i=0;i<30;i++) clean[i] = ' ';
  while (!OSD.notInVSync());
  for(i=0;i<20;i++){
      OSD.setCursor( 0, i );
      OSD.print( clean );
  }

  Serial.begin(115200);
}

static int16_t  throttle = 0;
static uint16_t current = 0;
static int8_t armed = 0;
static int8_t calybGyro = 0;
static uint8_t failsafe = 0;
static int8_t mode = 0;
static int8_t idleTime = 0;
static uint16_t LipoVoltage = 0;
static uint16_t LipoMAH = 0;
static uint16_t MaxAmps = 0;
static uint16_t MaxC    = 0;
static uint16_t MaxRPMs = 0;
static uint16_t MaxWatt = 0;
static uint16_t MaxTemp = 0;
static uint16_t MinBat = 0;
static uint16_t motorKERPM[4] = {0,0,0,0};
static uint16_t motorCurrent[4] = {0,0,0,0};
static uint16_t ESCTemps[4] = {0,0,0,0};
static int16_t  AuxChanVals[4] = {0,0,0,0};
static uint8_t  reducedMode = 0;
static uint16_t start_time = 0;
static unsigned long time = 0;
static uint16_t total_time = 0;

uint8_t print_int16(int16_t p_int, char *str, uint8_t dec, uint8_t AlignLeft){
    uint16_t useVal = p_int;
    uint8_t pre = ' ';
    if(p_int < 0){
        useVal = p_int*-1;
        pre = '-';
    }
    uint8_t aciidig[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint8_t i = 0;
        uint8_t digits[6] = {0,0,0,0,0,0};
    while(useVal >= 10000){digits[0]++; useVal-=10000;}
    while(useVal >= 1000){digits[1]++; useVal-=1000;}
    while(useVal >= 100){digits[2]++; useVal-=100;}
    while(useVal >= 10){digits[3]++; useVal-=10;}
    digits[4] = useVal;
        char result[6] = {' ',' ',' ',' ',' ','0'};
    uint8_t signdone = 0;
    for(i = 0; i < 6;i++){
        if(i == 5 && signdone == 0) continue;
        else if(aciidig[digits[i]] != '0' && signdone == 0){
            result[i] = pre;
            signdone = 1;
        }else if(signdone) result[i] = aciidig[digits[i-1]];
    }
        uint8_t CharPos = 0;
        for(i = 0; i < 6;i++){
          if(result[i] != ' ' || (AlignLeft == 0 || (i > 5-dec))) str[CharPos++] = result[i];
          if(dec != 0 && i == 5-dec) str[CharPos++] = '.';
          if(dec != 0 && i > 5-dec && str[CharPos-1] == ' ') str[CharPos-1] = '0';
        }
        
        return CharPos;
}	

void print_time(uint16_t seconds, char *time_str) {
    uint8_t minutes = 0;
    if (seconds > 60) {
      minutes = seconds/60;
    } else {
      minutes = 0;
    }
    seconds = seconds - (minutes * 60); // reste
    static char time_sec[6];
    uint8_t i = 0;
    uint8_t time_pos = print_int16(minutes, time_str,0,1);
    time_str[time_pos++] = 'm';
    print_int16(seconds, time_sec,0,1);
    for (i=0; i<6; i++)
    {
      time_str[time_pos++] = time_sec[i];
    }
    for (i=time_pos; i<30; i++)
    {
      time_str[time_pos++] = ' ';
    }
}

uint32_t ESC_filter(uint32_t oldVal, uint32_t newVal){
  return (uint32_t)((uint32_t)((uint32_t)((uint32_t)oldVal*ESC_FILTER)+(uint32_t)newVal))/(ESC_FILTER+1);
}



void loop(){
  uint16_t i = 0;
  uint8_t KRPMPoses[4];
  static uint8_t lastMode = 0;
  
  static char Motor1KERPM[30];
  static char Motor2KERPM[30];
  static char Motor3KERPM[30];
  static char Motor4KERPM[30];  
  
  uint8_t CurrentPoses[4];
  static char Motor1Current[30];
  static char Motor2Current[30];
  static char Motor3Current[30];
  static char Motor4Current[30];  
  
  uint8_t TempPoses[4];
  static char ESC1Temp[30];
  static char ESC2Temp[30];
  static char ESC3Temp[30];
  static char ESC4Temp[30];  
  
  static char LipoVoltC[30];
  static char LipoMinVoltC[30];
  static char LipoMAHC[30];
  
  static char Throttle[30];
  static char Current[30];

  static char Time[10];
  static char TotalTime[10];
  static char MaxTempC[30];
  
  static uint8_t serialBuf[255];
  static uint8_t minBytes = 0;
  static uint8_t recBytes = 0;
  
  static uint32_t LastLoopTime = 0;
  static uint8_t blink_i = 0;
  
  if(micros()-LastLoopTime > 10000){
    LastLoopTime = micros();
    blink_i++;
    if (blink_i >= 40){
      blink_i = 0;
    }
  
    Serial.write(0x20); // request telemetrie
    
    minBytes = 100;
    recBytes = 0;
   
    while(recBytes < minBytes && micros()-LastLoopTime < 20000){
      #define STARTCOUNT 2
      while(Serial.available()) serialBuf[recBytes++] = Serial.read();
      if(recBytes == 1 && serialBuf[0] != 5)recBytes = 0; // check for start byte, reset if its wrong
      if(recBytes == 2) minBytes = serialBuf[1]+STARTCOUNT+1; // got the transmission length
      if(recBytes == minBytes){
         uint32_t checksum = 0;
         for(i=2;i<minBytes;i++){
            checksum += serialBuf[i];
         }
         checksum = (uint32_t)checksum/(minBytes-3);
         
         if(checksum == serialBuf[recBytes-1]){
          
           throttle = ((serialBuf[STARTCOUNT]<<8) | serialBuf[1+STARTCOUNT])/10;
           LipoVoltage =   ((serialBuf[17+STARTCOUNT]<<8) | serialBuf[18+STARTCOUNT]);

           int8_t current_armed = serialBuf[16+STARTCOUNT];
           // switch disarmed => armed
           if (armed == 0 && current_armed > 0) {
             start_time = millis();
           }
           // switch armed => disarmed
           else if (armed > 0 && current_armed == 0) {
             total_time = total_time + (millis() - start_time);
             start_time = 0;
           } else if (armed > 0) {
             time = millis() - start_time;
           }
           armed = current_armed;

           #ifdef RED_DISPLAY_DEBUG
           /*
           armed =   ((serialBuf[15+STARTCOUNT]<<8) | serialBuf[16+STARTCOUNT]);
           calybGyro =   ((serialBuf[39+STARTCOUNT]<<8) | serialBuf[40+STARTCOUNT]);
           */
           mode =   serialBuf[65+STARTCOUNT];
           idleTime =   serialBuf[82+STARTCOUNT];
           //calybGyro =   serialBuf[40+STARTCOUNT];
           if (serialBuf[41+STARTCOUNT] > 0)
           {
               failsafe =   1;
           } else {
               failsafe =   0;
           }
           //failsafe =   ((serialBuf[40+STARTCOUNT]<<8) | serialBuf[41+STARTCOUNT]);
           if ((serialBuf[36+STARTCOUNT] + serialBuf[37+STARTCOUNT] + serialBuf[38+STARTCOUNT] + serialBuf[39+STARTCOUNT] + serialBuf[40+STARTCOUNT]) == 0)
           {
             calybGyro = 1;
           } else {
             calybGyro = 0;
           }
           #endif

           
           uint32_t tmpVoltage = 0;
           uint32_t voltDev = 0;
           if(((serialBuf[85+STARTCOUNT]<<8) | serialBuf[86+STARTCOUNT]) > 5){ // the ESC's read the voltage better then the FC
             tmpVoltage += ((serialBuf[85+STARTCOUNT]<<8) | serialBuf[86+STARTCOUNT]);
             voltDev++;
           }
           if(((serialBuf[95+STARTCOUNT]<<8) | serialBuf[96+STARTCOUNT]) > 5){ 
             tmpVoltage += ((serialBuf[95+STARTCOUNT]<<8) | serialBuf[96+STARTCOUNT]);
             voltDev++;
           }
           if(((serialBuf[105+STARTCOUNT]<<8) | serialBuf[106+STARTCOUNT]) > 5){
             tmpVoltage += ((serialBuf[105+STARTCOUNT]<<8) | serialBuf[106+STARTCOUNT]);
             voltDev++;
           }
           if(((serialBuf[115+STARTCOUNT]<<8) | serialBuf[116+STARTCOUNT]) > 5){ 
             tmpVoltage += ((serialBuf[115+STARTCOUNT]<<8) | serialBuf[116+STARTCOUNT]);
             voltDev++;
           }
           if(((serialBuf[125+STARTCOUNT]<<8) | serialBuf[126+STARTCOUNT]) > 5){
             tmpVoltage += ((serialBuf[125+STARTCOUNT]<<8) | serialBuf[126+STARTCOUNT]);
             voltDev++;
           }
           if(((serialBuf[125+STARTCOUNT]<<8) | serialBuf[126+STARTCOUNT]) > 5){ 
             tmpVoltage += ((serialBuf[125+STARTCOUNT]<<8) | serialBuf[126+STARTCOUNT]);
             voltDev++;
           }
           
           if(voltDev!=0) LipoVoltage = tmpVoltage/voltDev;

           if (MinBat == 0)
           {
             MinBat = LipoVoltage;
           }
           else if (LipoVoltage < MinBat)
           {
             MinBat = LipoVoltage;
           }
           
           MaxAmps =       ((serialBuf[146+STARTCOUNT]<<8) | serialBuf[147+STARTCOUNT]);
           LipoMAH =       ((serialBuf[148+STARTCOUNT]<<8) | serialBuf[149+STARTCOUNT]);
           MaxRPMs =       ((serialBuf[150+STARTCOUNT]<<8) | serialBuf[151+STARTCOUNT]);
           MaxWatt =       ((serialBuf[152+STARTCOUNT]<<8) | serialBuf[153+STARTCOUNT]);
           #if defined(RED_DISPLAY_MAXC)
           MaxC    =       (double)MaxAmps * (double)1000 / (double)LIPOS[0];
           #endif
           
           static uint32_t windedupfilterdatas[8];
           
           windedupfilterdatas[0] = ESC_filter((uint32_t)windedupfilterdatas[0],(uint32_t)((serialBuf[91+STARTCOUNT]<<8) | serialBuf[92+STARTCOUNT])/(MAGNETPOLECOUNT/2)<<4);
           windedupfilterdatas[1] = ESC_filter((uint32_t)windedupfilterdatas[1],(uint32_t)((serialBuf[101+STARTCOUNT]<<8) | serialBuf[102+STARTCOUNT])/(MAGNETPOLECOUNT/2)<<4);
           windedupfilterdatas[2] = ESC_filter((uint32_t)windedupfilterdatas[2],(uint32_t)((serialBuf[111+STARTCOUNT]<<8) | serialBuf[112+STARTCOUNT])/(MAGNETPOLECOUNT/2)<<4);
           windedupfilterdatas[3] = ESC_filter((uint32_t)windedupfilterdatas[3],(uint32_t)((serialBuf[121+STARTCOUNT]<<8) | serialBuf[122+STARTCOUNT])/(MAGNETPOLECOUNT/2)<<4);
           
           motorKERPM[0] = windedupfilterdatas[0]>>4;
           motorKERPM[1] = windedupfilterdatas[1]>>4;
           motorKERPM[2] = windedupfilterdatas[2]>>4;
           motorKERPM[3] = windedupfilterdatas[3]>>4;
           
           
           windedupfilterdatas[4] = ESC_filter((uint32_t)windedupfilterdatas[4],(uint32_t)((serialBuf[87+STARTCOUNT]<<8) | serialBuf[88+STARTCOUNT])<<4);
           windedupfilterdatas[5] = ESC_filter((uint32_t)windedupfilterdatas[5],(uint32_t)((serialBuf[97+STARTCOUNT]<<8) | serialBuf[98+STARTCOUNT])<<4);
           windedupfilterdatas[6] = ESC_filter((uint32_t)windedupfilterdatas[6],(uint32_t)((serialBuf[107+STARTCOUNT]<<8) | serialBuf[108+STARTCOUNT])<<4);
           windedupfilterdatas[7] = ESC_filter((uint32_t)windedupfilterdatas[7],(uint32_t)((serialBuf[117+STARTCOUNT]<<8) | serialBuf[118+STARTCOUNT])<<4);
           
           motorCurrent[0] = windedupfilterdatas[4]>>4;
           motorCurrent[1] = windedupfilterdatas[5]>>4;
           motorCurrent[2] = windedupfilterdatas[6]>>4;
           motorCurrent[3] = windedupfilterdatas[7]>>4;
           
           
           ESCTemps[0] = ((serialBuf[83+STARTCOUNT]<<8) | serialBuf[84+STARTCOUNT]);
           ESCTemps[1] = ((serialBuf[93+STARTCOUNT]<<8) | serialBuf[94+STARTCOUNT]);
           ESCTemps[2] = ((serialBuf[103+STARTCOUNT]<<8) | serialBuf[104+STARTCOUNT]);
           ESCTemps[3] = ((serialBuf[113+STARTCOUNT]<<8) | serialBuf[114+STARTCOUNT]);
           if (ESCTemps[0] > MaxTemp)
           {
             MaxTemp = ESCTemps[0];
           }
           if (ESCTemps[1] > MaxTemp)
           {
             MaxTemp = ESCTemps[1];
           }
           if (ESCTemps[2] > MaxTemp)
           {
             MaxTemp = ESCTemps[2];
           }
           if (ESCTemps[3] > MaxTemp)
           {
             MaxTemp = ESCTemps[3];
           }
           if (MaxTemp < 0) MaxTemp = 0; // bug ???

           AuxChanVals[0] = ((serialBuf[8+STARTCOUNT]<<8) | serialBuf[9+STARTCOUNT]);
           AuxChanVals[1] = ((serialBuf[10+STARTCOUNT]<<8) | serialBuf[11+STARTCOUNT]);
           AuxChanVals[2] = ((serialBuf[12+STARTCOUNT]<<8) | serialBuf[13+STARTCOUNT]);
           AuxChanVals[3] = ((serialBuf[14+STARTCOUNT]<<8) | serialBuf[15+STARTCOUNT]);
           
           current = (uint16_t)(motorCurrent[0]+motorCurrent[1]+motorCurrent[2]+motorCurrent[3])/10;
        }
      }
    }
    
  
    while (!OSD.notInVSync());
    
    
    for(i=0;i<10;i++){
      Motor1KERPM[i] = ' ';
      Motor2KERPM[i] = ' ';
      Motor3KERPM[i] = ' ';
      Motor4KERPM[i] = ' ';
      
      Motor1Current[i] = ' ';
      Motor2Current[i] = ' ';
      Motor3Current[i] = ' ';
      Motor4Current[i] = ' ';
      
      ESC1Temp[i] = ' ';
      ESC2Temp[i] = ' ';
      ESC3Temp[i] = ' ';
      ESC4Temp[i] = ' ';
      
      LipoVoltC[i] = ' ';
      LipoMinVoltC[i] = ' ';
      LipoMAHC[i] = ' ';
      Throttle[i] = ' ';
      MaxTempC[i] = ' ';
      Time[i]=' ';
      TotalTime[i]=' ';
    }
    
    
    uint8_t ThrottlePos = print_int16(throttle, Throttle,0,1);
    Throttle[ThrottlePos++] = '%';
    
    uint8_t CurrentPos = print_int16(current, Current,1,0);
    Current[CurrentPos++] = 'a';
    Current[CurrentPos++] = 't';

    KRPMPoses[0] = print_int16(motorKERPM[0], Motor1KERPM,1,1);
    Motor1KERPM[KRPMPoses[0]++] = 'k';
    Motor1KERPM[KRPMPoses[0]++] = 'r';
    
    KRPMPoses[1] = print_int16(motorKERPM[1], Motor2KERPM,1,0);
    Motor2KERPM[KRPMPoses[1]++] = 'k';
    Motor2KERPM[KRPMPoses[1]++] = 'r';
    
    KRPMPoses[2] = print_int16(motorKERPM[2], Motor3KERPM,1,0);
    Motor3KERPM[KRPMPoses[2]++] = 'k';
    Motor3KERPM[KRPMPoses[2]++] = 'r';
   
    KRPMPoses[3] = print_int16(motorKERPM[3], Motor4KERPM,1,1);
    Motor4KERPM[KRPMPoses[3]++] = 'k';
    Motor4KERPM[KRPMPoses[3]++] = 'r';
    
    
    CurrentPoses[0] = print_int16(motorCurrent[0], Motor1Current,2,1);
    Motor1Current[CurrentPoses[0]++] = 'a';
    
    CurrentPoses[1] = print_int16(motorCurrent[1], Motor2Current,2,0);
    Motor2Current[CurrentPoses[1]++] = 'a';
    
    CurrentPoses[2] = print_int16(motorCurrent[2], Motor3Current,2,0);
    Motor3Current[CurrentPoses[2]++] = 'a';
   
    CurrentPoses[3] = print_int16(motorCurrent[3], Motor4Current,2,1);
    Motor4Current[CurrentPoses[3]++] = 'a';
    
    
    
    TempPoses[0] = print_int16(ESCTemps[0], ESC1Temp,0,1);
    ESC1Temp[TempPoses[0]++] = '°';
    
    TempPoses[1] = print_int16(ESCTemps[1], ESC2Temp,0,0);
    ESC2Temp[TempPoses[1]++] = '°';
    
    TempPoses[2] = print_int16(ESCTemps[2], ESC3Temp,0,0);
    ESC3Temp[TempPoses[2]++] = '°';
   
    TempPoses[3] = print_int16(ESCTemps[3], ESC4Temp,0,1);
    ESC4Temp[TempPoses[3]++] = '°';

    uint8_t lipoVoltPos = print_int16(LipoVoltage, LipoVoltC,2,1);
    LipoVoltC[lipoVoltPos++] = 'v';
    uint8_t lipoMinVoltPos = print_int16(MinBat, LipoMinVoltC,2,1);
    LipoMinVoltC[lipoMinVoltPos++] = 'v';
    
    uint8_t lipoMAHPos = print_int16(LipoMAH, LipoMAHC,0,1);
    LipoMAHC[lipoMAHPos++] = 'm';
    LipoMAHC[lipoMAHPos++] = 'a';

    uint8_t ESCmarginBot       = 0;
    uint8_t ESCmarginTop       = 0;
    uint8_t TMPmargin          = 0;
    uint8_t CurrentMargin      = 0;
    uint8_t middle_infos_y     = 7;

    uint8_t displayNickname    = 0;
    uint8_t displayRCthrottle  = 0;
    uint8_t displayCombCurrent = 0;
    uint8_t displayLipoVoltage = 0;
    uint8_t displayConsumption = 0;
    uint8_t displayKRPM        = 0;
    uint8_t displayCurrent     = 0;
    uint8_t displayTemperature = 0;
    uint8_t displayStats       = 0;
    uint8_t displayTime        = 0;
    
     
    
    #if(RED_MODE_AUX_CHAN != 0)
    
      #if defined(RED_ON_AUX_LOW)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] < -250
      #endif
      #if defined(RED_ON_AUX_MID)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] > -250 && RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] < 250
      #endif
      #if defined(RED_ON_AUX_HIGH)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] > 250
      #endif
      
      if(RED_MODE_ACTIVE)reducedMode = 1;
      else reducedMode = 0;
    #else
      if(armed == 0)reducedMode = 1;
      else reducedMode = 0;
    #endif
    // debug
    #ifdef RED_DISPLAY_DEBUG
    reducedMode = 1;
    #endif
    
    if(reducedMode != lastMode){
      lastMode = reducedMode;
      for(i=0;i<20;i++){
          OSD.setCursor( 0, i );
          OSD.print( clean );
      }
      while (!OSD.notInVSync());
    }
    
    if(reducedMode == 0){
      #if defined(DISPLAY_NICKNAME)
      displayNickname = 1;
      #endif
      #if defined(DISPLAY_RC_THROTTLE)
      displayRCthrottle = 1;
      #endif
      #if defined(DISPLAY_COMB_CURRENT)
      displayCombCurrent = 1;
      #endif
      #if defined(DISPLAY_LIPO_VOLTAGE)
      displayLipoVoltage = 1;
      #endif
      #if defined(DISPLAY_MA_CONSUMPTION)
      displayConsumption = 1;
      #endif
      #if defined(DISPLAY_ESC_KRPM)
      displayKRPM = 1;
      #endif
      #if defined(DISPLAY_ESC_CURRENT)
      displayCurrent = 1;
      #endif
      #if defined(DISPLAY_ESC_TEMPERATURE)
      displayTemperature = 1;
      #endif      
      #if defined(DISPLAY_TIME)
      displayTime = 1;
      #endif 
    }else{
      #if defined(RED_DISPLAY_NICKNAME)
      displayNickname = 1;
      #endif
      #if defined(RED_DISPLAY_RC_THROTTLE)
      displayRCthrottle = 1;
      #endif
      #if defined(RED_DISPLAY_COMB_CURRENT)
      displayCombCurrent = 1;
      #endif
      #if defined(RED_DISPLAY_LIPO_VOLTAGE)
      displayLipoVoltage = 1;
      #endif
      #if defined(RED_DISPLAY_MA_CONSUMPTION)
      displayConsumption = 1;
      #endif
      #if defined(RED_DISPLAY_ESC_KRPM)
      displayKRPM = 1;
      #endif
      #if defined(RED_DISPLAY_ESC_CURRENT)
      displayCurrent = 1;
      #endif
      #if defined(RED_DISPLAY_ESC_TEMPERATURE)
      displayTemperature = 1;
      #endif    
      #if defined(RED_DISPLAY_STATS)
      // if armed at least 1x
      if (total_time > 10000) {
        displayStats = 1;
      }
      #endif    
      #if defined(RED_DISPLAY_TIME)
      displayTime = 1;
      #endif 
    }

    print_time(time/1000, Time);
    
    if(displayRCthrottle){
      OSD.setCursor( 0, 0 );
      //OSD.print( "throt:" );
      OSD.print( Throttle );
      
      if(displayNickname){
        OSD.setCursor( 10, 0 );
        OSD.print( NICKNAME );
      }
      ESCmarginTop = 1;
    } else {
      if(displayStats){
        middle_infos_y = middle_infos_y - 4;
      }
      if(displayNickname){
        OSD.setCursor( 11, middle_infos_y );
        OSD.print( NICKNAME );
      }
      if (displayStats){
        middle_infos_y++;

        print_time(total_time/1000, TotalTime);
        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "time     : " );
        OSD.print( TotalTime );

        OSD.setCursor( 5, ++middle_infos_y );
        //OSD.setCursor( -(5+lipoMAHPos), -1 );
        OSD.print( "max Amps : " );
        OSD.print( MaxAmps );
        if (MaxC > 0) {
          OSD.print( "A | " );
          OSD.print( MaxC );
          OSD.print( "C     " );
        } else {
          OSD.print( "A        " );
        }

        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "conso    : " );
        OSD.print( LipoMAHC );

        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "max RPMs : " );
        OSD.print( MaxRPMs );
        OSD.print( "         " );

        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "max Watt : " );
        OSD.print( MaxWatt );
        OSD.print( "W        " );

        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "max Temp : " );
        uint8_t MaxTempPos = print_int16(MaxTemp, MaxTempC,0,1);
        MaxTempC[MaxTempPos++] = '°';
        OSD.print( MaxTempC );
        OSD.print( "        " );

        OSD.setCursor( 5, ++middle_infos_y );
        OSD.print( "min bat  : " );
        OSD.print( LipoMinVoltC );
      }
    }
    
    if(displayCombCurrent){
      OSD.setCursor( -CurrentPos, 0 );
      OSD.print( Current );
      ESCmarginTop = 1;
    }
    
    if(displayLipoVoltage){
      OSD.setCursor( 0, -1 );
      OSD.print( "bat:" );
      OSD.print( LipoVoltC );
      ESCmarginBot = 1;
    }
    
    if(displayConsumption){
      OSD.setCursor( -(5+lipoMAHPos), -1 );
      //OSD.print( "co:" );
      OSD.print( LipoMAHC );
      ESCmarginBot = 1;
    }
    
    if(displayKRPM){
      OSD.setCursor( 0, ESCmarginTop );
      OSD.print( Motor1KERPM );
      OSD.setCursor( -KRPMPoses[1], ESCmarginTop );
      OSD.print( Motor2KERPM );
      OSD.setCursor( -KRPMPoses[2], -(1+ESCmarginBot) );
      OSD.print( Motor3KERPM );
      OSD.setCursor( 0, -(1+ESCmarginBot) );
      OSD.print( Motor4KERPM );
      TMPmargin++;
      CurrentMargin++;
    }
 
    if(displayCurrent){
      OSD.setCursor( 0, CurrentMargin+ESCmarginTop );
      OSD.print( Motor1Current );
      OSD.setCursor( -CurrentPoses[1], CurrentMargin+ESCmarginTop );
      OSD.print( Motor2Current );
      OSD.setCursor( -CurrentPoses[2], -(1+CurrentMargin+ESCmarginBot) );
      OSD.print( Motor3Current );
      OSD.setCursor( 0, -(1+CurrentMargin+ESCmarginBot) );
      OSD.print( Motor4Current );
      TMPmargin++;
    }
    
    if(displayTemperature){
      OSD.setCursor( 0, TMPmargin+ESCmarginTop );
      OSD.print( ESC1Temp );
      OSD.setCursor( -TempPoses[1], TMPmargin+ESCmarginTop );
      OSD.print( ESC2Temp );
      OSD.setCursor( -TempPoses[2], -(1+TMPmargin+ESCmarginBot) );
      OSD.print( ESC3Temp );
      OSD.setCursor( 0, -(1+TMPmargin+ESCmarginBot) );
      OSD.print( ESC4Temp );
    }  

    if(displayTime) {
      OSD.setCursor( 12, -2 );
      OSD.print( Time );
      //ESCmarginTop = 1;
    }
      
    #ifdef RED_DISPLAY_DEBUG
    /* debug */
    if (blink_i % 10 == 0) {
      middle_infos_y -= 6;
      OSD.setCursor( 0, middle_infos_y );
      OSD.print( "armed=" );
      OSD.print(armed);
      OSD.print( " mode=" );
      OSD.print(mode);
      OSD.print( " idle=" );
      OSD.print(idleTime);
      OSD.print( "         " );

      middle_infos_y++;
      OSD.setCursor( 0, middle_infos_y );
      OSD.print( "calyb=" );
      OSD.print(calybGyro);
      OSD.print( " failsafe=" );
      OSD.print(failsafe);
      OSD.print( "         " );
      middle_infos_y++;
      for(i=0; i<=9; i++)
      {
          OSD.setCursor( 0, middle_infos_y+i );
          OSD.print(i+36);
          OSD.print(":");
          OSD.print(serialBuf[i+36]);
          OSD.print("     ");
      }
      for(i=0; i<=9; i++)
      {
          OSD.setCursor( 10, middle_infos_y+i );
          OSD.print(i+80);
          OSD.print(": ");
          OSD.print(serialBuf[i+80]);
          OSD.print("      ");
      }
    }

/*
    if(reducedMode == 1){
      OSD.setCursor( 10, -2 );
      if (calybGyro > 0) {
        if (blink_i >= 20) {
          OSD.print( "calibrating" );
        } else {
        }
      } else {
          OSD.print( "           " );
      }
      */
      /*
      if (armed == 0) {
        OSD.setCursor( 11, -4 );
        OSD.print( "disarmed" );
      }
    }
      */
    #endif
  }    
} 


// vim: set ts=4 sw=4 expandtab:
