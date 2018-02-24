//=============================
// APPLICATION VARS and INIT
//=============================

//target boiler temp 
#define DEFAULT_TARGET_TEMP 25 //120
//time of shot
unsigned long shotStartTime=0;
unsigned long shotTimeMS = 0;


//=====================
// PID init
//=====================
#include "PID_v1.h"

//Define Variables we'll be connecting to
double PID_setpoint, PID_input, PID_output;
//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
double Kp=25, Ki=0.1, Kd=0.1;
PID myPID(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, P_ON_E,DIRECT);

void setup_pid(){
  PID_setpoint = DEFAULT_TARGET_TEMP;
  myPID.SetOutputLimits(0,100);
  myPID.SetSampleTime(350);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

//=====================
// OLED init
//=====================
/*                   Define the serial port to use here, if using software serial set it to  */
/*                   something like SerialS.                                                 */
#include <SoftwareSerial.h>
SoftwareSerial DisplaySerial(2,3) ;// pin 2 = TX of display, pin3 = RX

#define LOG_MESSAGES //debug ON
#ifdef LOG_MESSAGES
  #define HWLOGGING Serial
#else
  #define HWLOGGING if (1) {} else Serial
#endif

#include "Goldelox_Serial_4DLib.h"
#include "Goldelox_Const4D.h"

Goldelox_Serial_4DLib Display(&DisplaySerial);

//variables
#define iOledLudoH 0x0000
#define iOledLudoL 0x6400
#define iBgDashboardH     0x0001
#define iBgDashboardL     0x0600
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

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

void setup_temp()
{
  //init sensor
  max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
}

float get_temp()
{
  uint16_t rtd = max.readRTD();
  //float ratio = rtd;
  //ratio /= 32768;
  float tempnow = max.temperature(100, RREF);

  // Check and print any faults
  uint8_t fault = max.readFault();
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
    max.clearFault();
  }
  HWLOGGING.println();
  return tempnow;
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
  shotStartTime = millis();
}

//=============================
// LOOP
//=============================
void loop() 
{
  float tSensor1= get_temp();//((get_temp()-18)*120)/(25-18);
  float tSensor2 = 0; 
  char sTemp[6];
  HWLOGGING.println("loop");
  //numx 
  //HWLOGGING.print("temp");
  //HWLOGGING.print(tSensor1);HWLOGGING.print("C");

  //PID update
  //-----------------------------
  PID_input = tSensor1;
  myPID.Compute();
//  HWLOGGING.print("PID Input");HWLOGGING.println(PID_input);
//  HWLOGGING.print("PID Output");HWLOGGING.println(PID_output);
//  HWLOGGING.print("PID P");HWLOGGING.println(myPID.GetKp());
//  HWLOGGING.print("PID I");HWLOGGING.println(myPID.GetKi());
//  HWLOGGING.print("PID D");HWLOGGING.println(myPID.GetKd());
  
  //Display temperature sensor #1
  //-----------------------------
  Display.txt_MoveCursor(0,0);
  Display.gfx_MoveTo(55 , 27) ;
  //round temperature, only one number after virgule
  dtostrf(tSensor1, 5, 1, sTemp );
  Display.print(sTemp);
  Display.print("/");
  Display.print((int)PID_setpoint);
  Display.print("C   ");

  //Display temperature sensor #2
  //-----------------------------
  Display.gfx_MoveTo(50 , 60) ;
  //round temperature, only one number after virgule
  //dtostrf(tSensor2, 5, 1, sTemp );
  dtostrf(PID_output, 5, 0, sTemp);
  Display.print("PID:");
  Display.print(sTemp);
  Display.print("%   ");

  //Display time of shot
  //-----------------------------
  shotTimeMS = millis()- shotStartTime;
  Display.gfx_MoveTo(62 , 94) ;
  int shotMin,shotSec,shotMSec;
  shotMin = (int)(shotTimeMS / 60000);
  shotSec = (int)((shotTimeMS / 1000)- (shotMin*60));
  shotMSec = (int)(((shotTimeMS/100) - (shotMin*600) - (shotSec*10)));
  Display.print(shotMin); Display.print("\"");
  Display.print(shotSec); Display.print(".");
  Display.print(shotMSec);  Display.print("     ");

  //Idle before loop
  //-----------------------------
  delay(50);
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

