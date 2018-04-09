//=============================
// APPLICATION VARS and INIT
//=============================

//target boiler temp 
#define DEFAULT_TARGET_TEMP 25 //120

//debug mode
#define LOG_MESSAGES //debug ON
#ifdef LOG_MESSAGES
  #define HWLOGGING Serial
#else
  #define HWLOGGING if (1) {} else Serial
#endif

//=====================
// CHRONO init
//=====================
#define PIN_SW_BAS  6
#define PIN_SW_HAUT  7

int isChronoPreOn = 0;
int isChronoShotOn = 0;
int flagPreinf = 0;
//time of shot
unsigned long shotPreinfTime=0;
unsigned long shotPreinfFinalTime=0;
unsigned long shotStartTime=0;
unsigned long shotTimeMS = 0;

void test_switch()
{
  //detect switch to start preinf 
  if((digitalRead(PIN_SW_HAUT) == LOW) && !isChronoPreOn){
    shotPreinfTime = millis();
    isChronoPreOn = 1;
    return;
  }
  //flag to detect lever rise 
  if((digitalRead(PIN_SW_BAS) == LOW) && isChronoPreOn && !isChronoShotOn && !flagPreinf){
    flagPreinf = 1;
    return;
  }
  //detect switch to start shot 
  if((digitalRead(PIN_SW_BAS) == HIGH) && flagPreinf && !isChronoShotOn){
    shotStartTime = millis();
    shotPreinfFinalTime = shotStartTime-shotPreinfTime;
    isChronoShotOn = 1;
    return;
  }
  //detect switch to stop shot
  if((digitalRead(PIN_SW_HAUT) == HIGH)&& (isChronoShotOn || isChronoPreOn)){
    //case if no shot (only preinf)
    if(!isChronoShotOn){
      shotPreinfFinalTime = millis()-shotPreinfTime;
      shotTimeMS = 0;
    }
    isChronoShotOn = 0;
    isChronoPreOn = 0;
    flagPreinf = 0;
  }
}


//=====================
// PID init
//=====================
#include "PID_v1.h"
#include "TimerOne.h"

//Define Variables we'll be connecting to
double PID_setpoint, PID_input, PID_output;
double last_PID_output;
//Specify the links and initial tuning parameters
double Kp=50, Ki=0.1, Kd=0.1;
PID myPID(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, P_ON_E,DIRECT);

void setup_pid(){
  PID_setpoint = DEFAULT_TARGET_TEMP;
  myPID.SetOutputLimits(0,100);
  myPID.SetSampleTime(250);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //set a timer to drive the SSR
  Timer1.initialize(250000); // set a timer of 0,25 second length (4Hz)
  //PWM on pin 9, set to 0 for now
  Timer1.pwm(9,0);
}

//=====================
// OLED init
//=====================
/*                   Define the serial port to use here, if using software serial set it to  */
/*                   something like SerialS.                                                 */
#include <SoftwareSerial.h>
SoftwareSerial DisplaySerial(2,3) ;// pin 2 = TX of display, pin3 = RX

#include "Goldelox_Serial_4DLib.h"
#include "Goldelox_Const4D.h"

Goldelox_Serial_4DLib Display(&DisplaySerial);

//variables
#define iOledLudoH 0x0000
#define iOledLudoL 0x6400
#define iBgDashboardH     0x0001
#define iBgDashboardL     0x0600
#define ibar0H            0x0001
#define ibar0L            0xA600
#define ibar1H            0x0001
#define ibar1L            0xAE00
#define ibar2H            0x0001
#define ibar2L            0xB600
#define ibar3H            0x0001
#define ibar3L            0xBE00
#define Inputs 0
#define  Strings1Count    0
#define  Strings1Size     1
#define  Strings2Count    10
#define  Strings2Size     2
#define  Strings2FontStartL   0x0
#define  Strings1StartH   0x0
#define  Strings1StartL   0xC400
#define  Strings2StartH   0x0
#define  Strings2StartL   0xC600
#define  Strings3Count    1
#define  Strings3Size     12
#define  Strings3FontStartL   0x11
#define  Strings3StartH   0x1
#define  Strings3StartL   0xAC00

void setup_oled() {
  //For handling errors
  Display.Callback4D = mycallback ;

  //5 second timeout on all commands  
  Display.TimeLimit4D = 5000 ;

  //SET HIGH SPEED BAUD RATE
  //DisplaySerial.begin(9600) ;
  DisplaySerial.begin(57600);//115200);
  //delay(50) ; // Display sleeps for 100
  //DisplaySerial.flush() ;
  //Display.GetAck() ;

  //--------------------------------Optional reset routine-----------------------------------
  pinMode(4, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(4, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(4, 0);  // unReset the Display via D4
  
  delay (5000); //let the display start up  

  Display.gfx_ScreenMode(LANDSCAPE);
  Display.SSTimeout(0) ;
  Display.SSSpeed(0) ;
  Display.SSMode(0) ;
  Display.gfx_Cls();            //clear the screen

  //init OLED SD card
  Display.println("Starting\n") ;
  while(!Display.media_Init())
  {
        Display.println("Drive not mounted...");
        delay(200);
        Display.gfx_Cls();
  }
  //background pic
  Display.media_SetAdd(iOledLudoH, iOledLudoL) ;      // point to the OledLudo image
  Display.media_Image(0, 0) ;            // show image
  //string with custom font
  Display.media_SetSector(0, Strings2FontStartL) ;    // must come b4 setting fontID
  Display.txt_FontID(MEDIA) ; // Font index correct at time of code generation
  Display.txt_FGcolour(WHITE) ;
  Display.txt_BGcolour(BLACK) ;
}


//=============================
// RTD temp probe init
//=============================
#include <Adafruit_MAX31865.h>
#include <SPI.h>

// Use software SPI: CS, DI, DO, CLK (use 8 instead of 10 because of pwm)
Adafruit_MAX31865 max = Adafruit_MAX31865(8, 11, 12, 13);
Adafruit_MAX31865 max2 = Adafruit_MAX31865(10, 11, 12, 13);

// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

void setup_temp()
{
  //init sensors
  max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  max2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
}

float get_temp(int id)
{
  uint16_t rtd;
  float tempnow;
  uint8_t fault;
  
  if(id == 1){
    rtd = max.readRTD();
    tempnow = max.temperature(100, RREF);
    fault = max.readFault();
  }else{
    rtd = max2.readRTD();
    tempnow = max2.temperature(100, RREF);
    fault = max2.readFault();
  }
  //float ratio = rtd;
  //ratio /= 32768;
  
  // Check and print any faults
  if (fault) {
    HWLOGGING.print("Fault 0x"); HWLOGGING.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      HWLOGGING.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      HWLOGGING.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      HWLOGGING.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      HWLOGGING.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      HWLOGGING.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      HWLOGGING.println("Under/Over voltage"); 
    }
    if(id == 1)
      max.clearFault();
    else
      max2.clearFault();
  }
  return tempnow;
}

//=============================
// MPR121 init (touch sensor)
//=============================
#include <Wire.h>
#include "Adafruit_MPR121.h"

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

void setup_touch(){
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    HWLOGGING.println("MPR121 not found, check wiring?");
    while (1);
  }
  HWLOGGING.println("MPR121 found!");
}

int getTouched(){
  // Get the currently touched pads
  currtouched = cap.touched();

  // FOR DEBUG ONLY
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) /*&& !(lasttouched & _BV(i))*/ ) {
      HWLOGGING.print(i); HWLOGGING.println(" touched");
      // reset our state
      lasttouched = currtouched;
      return i;
    }
    //if it *was* touched and now *isnt*, alert!
    //if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
    //  HWLOGGING.print(i); HWLOGGING.println(" released");
    //}
  }

  // reset our state
  lasttouched = currtouched;
  
  return 0;
}
//=============================
// SETUP
//=============================
void setup()
{
  HWLOGGING.begin(115200);
  HWLOGGING.println("Ludo app start!");
  setup_oled();
  setup_temp();
  setup_pid();
  setup_touch();
  //setup_switch();
  show_loading();
}

void show_loading(){
  int toggle = 1;
  int i=0;

  //loading for 10 seconds
  for(i=0;i<8;i++){
    if(toggle){
      Display.gfx_MoveTo(80 , 80) ;
      Display.putstr("Loading...") ;
      toggle = 0;
      delay(1000);
    }
    else{
      Display.gfx_MoveTo(80 , 80) ;
      Display.putstr("                  ") ;
      toggle = 1;
      delay(500);
    }
  }
  //PREPARE dashboard screen
  //Display.gfx_Cls();            //clear the screen
  Display.media_SetAdd(iBgDashboardH, iBgDashboardL) ;      // point to the OledLudo image
  Display.media_Image(0, 0) ;            // show image

  //SET BIG FONT
  Display.media_SetSector(0, Strings3FontStartL) ;    // must come b4 setting fontID
  Display.txt_FontID(MEDIA) ; // Font index correct at time of code generation

  //init few vars
  //shotStartTime = millis();
}

//=============================
// LOOP
//=============================
void loop() 
{
  float tSensor1= get_temp(1);//((get_temp()-18)*120)/(25-18);
  float tSensor2 = get_temp(2); 
  char sTemp[6];
  char buff[255];
  int ct;
  char degreChar[] = "\xb0";
  //profiling data
  unsigned long loopstart = millis();
  
  //check switch state different times (for better response time)
  test_switch();
  
  //Touch update
  //-----------------------------
  ct = getTouched();
  if(ct != 0){
    if(ct < 6)
      PID_setpoint--;
    else
      PID_setpoint++;
  }
  //PID update
  //-----------------------------
  PID_input = tSensor1;
  myPID.Compute();
  HWLOGGING.println("/PIDOutput/");HWLOGGING.print(PID_output); 
  
  //Icon PID display update
  //-----------------------------
  
  if(PID_output == 0)
    Display.media_SetAdd(ibar0H, ibar0L) ;      // point to the right image
  else if(PID_output <= 30)
    Display.media_SetAdd(ibar1H, ibar1L) ;      // point to the right image
  else if(PID_output <= 80)
    Display.media_SetAdd(ibar2H, ibar2L) ;      // point to the right image
  else
    Display.media_SetAdd(ibar3H, ibar3L) ;      // point to the right image
  Display.media_Image(14, 26) ;            // show image
  
  last_PID_output = PID_output;
  //SSR update
  //-----------------------------
  //PID output is percent, duty cycle is 1024, but we must translate to steps of 25 because of zero crossing 50Hz
  int PIDsteps = (int)(PID_output/4);
  int currentduty = (PIDsteps*1024)/25;
  Timer1.setPwmDuty(9,currentduty);
//  HWLOGGING.print("PID Input");HWLOGGING.println(PID_input);
/*
  HWLOGGING.print("Time/");HWLOGGING.print(millis());
  HWLOGGING.print("/Temp/");HWLOGGING.print(tSensor1);
  HWLOGGING.print("/PIDOutput/");HWLOGGING.print(PID_output);
  HWLOGGING.print("/PIDsteps/");HWLOGGING.print(PIDsteps);
  HWLOGGING.print("/duty/");HWLOGGING.print(currentduty);
*/
  
  //Display temperature sensor #1
  //-----------------------------
  Display.txt_MoveCursor(0,0);
  Display.gfx_MoveTo(55 , 27) ;
  //round temperature, only one number after virgule
  //dtostrf(tSensor1, 5, 1, sTemp );
  dtostrf(tSensor1, 5, 0, sTemp );
  sprintf(buff,"%s/%d%sC   ",sTemp,(int)PID_setpoint,degreChar);
  Display.putstr(buff);
  
  //check switch state different times (for better response time)
  test_switch();
    
  //Display temperature sensor #2
  //-----------------------------
  Display.gfx_MoveTo(50 , 60) ;
  //round temperature, only one number after virgule
  dtostrf(tSensor2, 5, 1, sTemp );
  sprintf(buff,"%s%sC   ",sTemp,degreChar);
  Display.putstr(buff);
  
  //dtostrf(PID_output, 5, 0, sTemp);
  //Display.print("PID:");
  //Display.print(sTemp);
  //Display.print("%   ");

  //Display time of shot
  //-----------------------------
  //check switch state different times (for better response time)
  test_switch();

  //if chrono is running, update shot time
  if(isChronoPreOn && !isChronoShotOn){
    shotTimeMS = millis()- shotPreinfTime;
  } 
  if(isChronoShotOn){
    shotTimeMS = millis()- shotStartTime;
  }
  Display.gfx_MoveTo(62 , 94) ;
  int shotMin,shotSec,shotMSec;
  shotMin = (int)(shotTimeMS / 60000);
  shotSec = (int)((shotTimeMS / 1000)- (shotMin*60));
  shotMSec = (int)(((shotTimeMS/100) - (shotMin*600) - (shotSec*10)));
  //sprintf(buff,"%d\"%d.%d  ",shotMin,shotSec,shotMSec);
  //sprintf(buff,"%d\"%d    ",shotMin,shotSec);
  
  if(isChronoPreOn && !isChronoShotOn)
    sprintf(buff,"%d' -PRE-   ",shotSec);
  else
    sprintf(buff,"%d' > %d'    ",(int)(shotPreinfFinalTime/1000), shotSec);
  Display.putstr(buff);
  
  //HWLOGGING.print("A7=");HWLOGGING.println(millis()-loopstart);

  //Idle before loop, sync to 1 second
  //-----------------------------
  int timeToWait = 685 - (millis()-loopstart);
  if(timeToWait > 0)
    delay(timeToWait);
  //HWLOGGING.print("A8=");HWLOGGING.println(millis()-loopstart);
}

//=============================
// ERROR SETTINGS
//=============================
void mycallback(int ErrCode, unsigned char ErrByte)
{
  // Pin 13 has an LED connected on most Arduino boards. Just give it a name
  int led = 13;
  const char *Error4DText[] = {"OK\0", "Timeout\0", "NAK\0", "Length\0", "Invalid\0"} ;
  HWLOGGING.println("Error callback!");
  HWLOGGING.print("errcode=");HWLOGGING.print(ErrCode);
  HWLOGGING.println(Error4DText[ErrCode]);

  if (ErrCode == Err4D_NAK)
  {
    HWLOGGING.print(F(" returned data= ")) ;
    HWLOGGING.println(ErrByte) ;
  }
  
  pinMode(led, OUTPUT);
  while(1)
  {
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                // wait for 200 ms
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(200);                // wait for 200 ms
  }
}

