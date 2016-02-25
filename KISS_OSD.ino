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
#define DISPLAY_RC_THROTTLE
#define DISPLAY_COMB_CURRENT
#define DISPLAY_LIPO_VOLTAGE
#define DISPLAY_MA_CONSUMPTION
#define DISPLAY_ESC_KRPM
#define DISPLAY_ESC_CURRENT
#define DISPLAY_ESC_TEMPERATURE

// displayed datas in reduced mode
//=============================
//#define RED_DISPLAY_RC_THROTTLE
//#define RED_DISPLAY_COMB_CURRENT
#define RED_DISPLAY_LIPO_VOLTAGE
#define RED_DISPLAY_MA_CONSUMPTION
//#define RED_DISPLAY_ESC_KRPM
//#define RED_DISPLAY_ESC_CURRENT
//#define RED_DISPLAY_ESC_TEMPERATURE


// reduced mode channel config
//=============================
#define RED_MODE_AUX_CHAN 4 // 0-4, 0 = none

#define RED_ON_AUX_LOW
//#define RED_ON_AUX_MID
//#define RED_ON_AUX_HIGH


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
static uint16_t LipoVoltage = 0;
static uint16_t LipoMAH = 0;
static uint16_t motorKERPM[4] = {0,0,0,0};
static uint16_t motorCurrent[4] = {0,0,0,0};
static uint16_t ESCTemps[4] = {0,0,0,0};
static int16_t  AuxChanVals[4] = {0,0,0,0};
static uint8_t  reducedMode = 0;

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
  static char LipoMAHC[30];
  
  static char Throttle[30];
  static char Current[30];
  
  static uint8_t serialBuf[255];
  static uint8_t minBytes = 0;
  static uint8_t recBytes = 0;
  
  static uint32_t LastLoopTime = 0;
  
  if(micros()-LastLoopTime > 10000){
    LastLoopTime = micros();
  
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
             
           
           LipoMAH =       ((serialBuf[148+STARTCOUNT]<<8) | serialBuf[149+STARTCOUNT]);
           
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
      Throttle[i] = ' ';
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
    
    uint8_t lipoMAHPos = print_int16(LipoMAH, LipoMAHC,0,1);
    
    uint8_t ESCmarginBot       = 0;
    uint8_t ESCmarginTop       = 0;
    uint8_t TMPmargin          = 0;
    uint8_t CurrentMargin      = 0;
    
    uint8_t displayRCthrottle  = 0;
    uint8_t displayCombCurrent = 0;
    uint8_t displayLipoVoltage = 0;
    uint8_t displayConsumption = 0;
    uint8_t displayKRPM        = 0;
    uint8_t displayCurrent     = 0;
    uint8_t displayTemperature = 0;
    
     
    
    #if(RED_MODE_AUX_CHAN != 0)
    
      #if defined(RED_ON_AUX_LOW)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] < -250
      #endif
      #if defined(RED_ON_AUX_MID)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] > -250 && RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] < 250
      #endif
      #if defined(RED_ON_AUX_MID)
        #define RED_MODE_ACTIVE AuxChanVals[RED_MODE_AUX_CHAN-1] > 250
      #endif
      
      if(RED_MODE_ACTIVE)reducedMode = 1;
      else reducedMode = 0;
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
    }else{
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
    }
    
    if(displayRCthrottle){
      OSD.setCursor( 0, 0 );
      OSD.print( "throt:" );
      OSD.print( Throttle );
      ESCmarginTop = 1;
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
      OSD.print( "co:" );
      OSD.print( LipoMAHC );
      OSD.print( "ma" );
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
      
  }    
} 



