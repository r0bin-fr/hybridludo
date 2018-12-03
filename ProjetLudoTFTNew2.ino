//=============================
// APPLICATION VARS and INIT
//=============================

//GLOBAL VARS
//---------------------
//target boiler temp 
#define DEFAULT_TARGET_TEMP 115
#define ECO_TARGET_TEMP     90
boolean isModeECOLudo = false;
boolean isStandBy = false;

//DEBUG mode
//#define LOG_MESSAGES //uncomment for debug, comment for histo
#ifdef LOG_MESSAGES
  #define HWLOGGING Serial
#else
  #define HWLOGGING if (1) {} else Serial
#endif

//ARDUINO PINOUT 
//---------------------
#define TFT_DC 2        //PIN2 = TFT DC
#define TFT_CS 3        //PIN3 = TFT CS
//(SD_CS_PIN dans pffArduino.h) //PIN 4 = SDCARD PIN 
#define TFT_BL 5        //PIN5 = TFT backlight
#define PIN_SW_BAS  6   //PIN6 = inter levier bas
#define PIN_SW_HAUT  7  //PIN7 = inter levier haut
#define MAXTEMP_CS1 8   //PIN8 = MAX temp sensor CS #1
#define SSR_PIN 9       //PIN9 = SSR output
#define MAXTEMP_CS2 10  //PIN10 = MAX temp sensor CS #1
//PIN11= SPI MOSI
//PIN12= SPI MISO
//PIN13= SPI SCK
#define PSCK_PIN   14   //PIN14 = balance SCK
#define DT_PIN    15    //PIN15 = balance DT
#define PIN_LED2 16     //PIN16 = led arriere machine
#define PIN_LED1 17     //PIN17 = led cafe
//PIN18= I2C SDA
//PIN19= I2C SL


//=====================
// TFT screen 
//=====================
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "PetitFS.h"
FATFS fs;     // File system object for SD

// Use hardware SPI (on Uno, #13, #12, #11) and the below for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup_tft() {
  tft.begin();
  //fillscreen du début
  analogWrite(TFT_BL, 50);
  tft.fillScreen(ILI9341_RED);
  yield();
  analogWrite(TFT_BL, 100);
  tft.fillScreen(ILI9341_GREEN);
  yield();
  analogWrite(TFT_BL, 200);
  tft.fillScreen(ILI9341_BLUE);
  yield();
  analogWrite(TFT_BL, 255);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //loading text
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(80,100);
  tft.print("Loading...");
}


// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
uint16_t read16() {
  uint16_t result;
  uint8_t  sdbuffer[3];
  unsigned int bRead; 
  
  pf_read(sdbuffer,2,&bRead);
  ((uint8_t *)&result)[0] =  sdbuffer[0];// LSB
  ((uint8_t *)&result)[1] = sdbuffer[1]; // MSB
  return result;
}
uint32_t read32() {
  uint32_t result;
  uint8_t  sdbuffer[5];
  unsigned int bRead; 

  pf_read(sdbuffer,4,&bRead);
  ((uint8_t *)&result)[0] = sdbuffer[0]; // LSB
  ((uint8_t *)&result)[1] = sdbuffer[1];
  ((uint8_t *)&result)[2] = sdbuffer[2];
  ((uint8_t *)&result)[3] = sdbuffer[3]; // MSB
  return result;
}

//=====================
// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth² of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.
//=====================
#define BUFFPIXEL 20
unsigned int bmpDraw2(char *filename, uint8_t x, uint16_t y, boolean  flip) {
// File     bmpFile;
 long int      bmpWidth, bmpHeight;   
 uint8_t  bmpDepth;              
 uint32_t bmpImageoffset;        
 uint32_t rowSize;               
 uint8_t  sdbuffer[3*BUFFPIXEL]; 
 uint8_t  buffidx = sizeof(sdbuffer);                
 int      w, h, row, col;
 uint8_t  r, g, b;
 uint32_t pos = 0, startTime = millis();
  unsigned int bRead; 
  
  //open file
  bRead = pf_mount(&fs);
  if(bRead){
    delay(1000);
    bRead = pf_mount(&fs);
  }
  //HWLOGGING.print("pfmount returned "); HWLOGGING.println(bRead); 
  bRead = pf_open(filename);
  //HWLOGGING.print("pfopen returned "); HWLOGGING.println(bRead); 
  if(bRead){
    delay(1000);
    bRead = pf_open(filename);
  }
  if(bRead)
    return bRead;
   
   read16(); 
   read32();
   (void)read32(); 
   bmpImageoffset = read32(); 
   
   read32();
   bmpWidth  = read32();
   bmpHeight = read32();
   bmpDepth = read16(); 
   /*
   HWLOGGING.print("bmpImageoffset="); HWLOGGING.println(bmpImageoffset); 
   HWLOGGING.print("bmpWidth="); HWLOGGING.println(bmpWidth); 
   HWLOGGING.print("bmpHeight="); HWLOGGING.println(bmpHeight); 
   HWLOGGING.print("bmpDepth="); HWLOGGING.println(bmpDepth); 
   */
   
   // If bmpHeight is negative, image is in top-down order.
   // This is not canon but has been observed in the wild.
   if(bmpHeight < 0) {
      bmpHeight = -bmpHeight;
      flip      = false;
   }

   rowSize = (bmpWidth * 3 + 3) & ~3;
       tft.setAddrWindow(x, y, x+bmpWidth-1, y+bmpHeight-1);
       //tft.setWindow(x, y, x+bmpWidth-1, y+bmpHeight-1);
       for (row=0; row<bmpHeight; row++) {  
          //pos = bmpImageoffset + row * rowSize;
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          pf_lseek(pos);
          buffidx = sizeof(sdbuffer);

         for (col=0; col<bmpWidth; col++) {         
           if (buffidx >= sizeof(sdbuffer)) { 
             pf_read(sdbuffer, sizeof(sdbuffer),&bRead);
             buffidx = 0; 
           }
           //HWLOGGING.print(F(
           b = sdbuffer[buffidx++];
           g = sdbuffer[buffidx++];
           r = sdbuffer[buffidx++];
           tft.pushColor(tft.color565(r,g,b));         
         }
       } 
   return 0;
}

//=====================
// Capteur de poids
//=====================
#include <Hx711.h>

/* initiate scale... */
Hx711 scale(DT_PIN, PSCK_PIN);
float oldPoids = 0;
//init
void setup_scale(){
  //scale setting
  scale.setScale(1627);
  scale.setOffset(scale.averageValue());
}
//pesee
float get_poids()
{
    float poids = scale.getGram2(5);
    oldPoids = poids; 
    return poids;
}

//=====================
// CHRONO init
//=====================
char isChronoPreOn = 0;
char isChronoShotOn = 0;
char flagPreinf = 0;
//time of shot
unsigned long shotPreinfTime=0;
unsigned long shotPreinfFinalTime=0;
unsigned long shotStartTime=0;
unsigned long shotTimeMS = 0;
boolean isBrewingShot = false;

void test_switch()
{
  //detect switch to start preinf _ detecter le commutateur pour demarrer preinf
  if((digitalRead(PIN_SW_BAS) == LOW) && !isChronoPreOn){
    shotPreinfTime = millis();
    //reset balance
    scale.setOffset(scale.getValue());
    isChronoPreOn = 1;
    //indicateur global
    isBrewingShot = true;
    setHistoBoiler(0);
    //reset display timeout
    reset_timetodisplay(1);
    return;
  }
  //flag to detect lever rise _ indicateur pour detecter l'elevation du levier
  if((digitalRead(PIN_SW_BAS) == HIGH) && isChronoPreOn && !isChronoShotOn && !flagPreinf){
    flagPreinf = 1;
    return;
  }
  //detect switch to start shot _ detecter le commutateur pour demarrer le shot
  if((digitalRead(PIN_SW_BAS) == HIGH) && flagPreinf && !isChronoShotOn){
    shotStartTime = millis();
    shotPreinfFinalTime = shotStartTime-shotPreinfTime;
    isChronoShotOn = 1;
    return;
  }
  //detect switch to stop shot _ detecter le commutateur pour arreter le shot
  if((digitalRead(PIN_SW_HAUT) == LOW)&& (isChronoShotOn || isChronoPreOn)){
    //case if no shot (only preinf)
    if(!isChronoShotOn){
      shotPreinfFinalTime = millis()-shotPreinfTime;
      shotTimeMS = 0;
    }
    isChronoShotOn = 0;
    isChronoPreOn = 0;
    flagPreinf = 0;
    //indicateur global
    isBrewingShot = true;
    setHistoBoiler(1);
    //reset display timeout
    reset_timetodisplay(1);
  }

  //test les boutons capacitifs 
  test_switch2();
}


//=====================
// PID init
//=====================
#include "PID_v1.h"
#include "TimerOne.h"

//Define Variables we'll be connecting to
double PID_setpoint, PID_backup, PID_input, PID_output;
float last_PID_output;
//Specify the links and initial tuning parameters
float Kp=50, Ki=0.1, Kd=0.1;
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
  Timer1.pwm(SSR_PIN,0);
}

//set mode ECO ludo
void setModeECO(int setOn){
  if(setOn){
    if(!isModeECOLudo){
      isModeECOLudo = true;
      PID_backup = PID_setpoint;
      PID_setpoint = ECO_TARGET_TEMP;
    }
  }else{
      if(isModeECOLudo){
        isModeECOLudo = false;
        PID_setpoint = PID_backup;
    }
  }
}


//=====================
// Eclairage led
//=====================
boolean etatRelais=0; // variable bit reflet etat du relais
boolean etatRelais2=0; // variable bit reflet etat du relais

 void setup_relais( ) { 
  pinMode ( PIN_LED1, OUTPUT ) ;
  digitalWrite ( PIN_LED1, HIGH ) ;

  pinMode ( PIN_LED2, OUTPUT ) ;
  digitalWrite ( PIN_LED2, HIGH ) ;
}

 

//=============================
// RTD temp probe init
//=============================
#include <Adafruit_MAX31865.h>
#include <SPI.h>

// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 max = Adafruit_MAX31865(MAXTEMP_CS1);
Adafruit_MAX31865 max2 = Adafruit_MAX31865(MAXTEMP_CS2);

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
      HWLOGGING.println("R1");//RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      HWLOGGING.println("R2");//RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      HWLOGGING.println("R3");//REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      HWLOGGING.println("R4");//REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      HWLOGGING.println("R5");//RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      HWLOGGING.println("R6");//Under/Over voltage"); 
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
    HWLOGGING.println("MPR121 not found");// check wiring?
    while (1);
  }
//  HWLOGGING.println("MPR121 OK");
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
  //nothing touched, return -1
  return -1;
}

//test les boutons du MPR121 (touch sensor)
void test_switch2()
{
  int ct = getTouched();
  
  //si ecran en veille et appui un bouton, on sort de veille
  if(ct >= 0){
    reset_timetodisplay(1);
  }
  
  //Eclairage led 1
  if(ct == 1) {
    etatRelais=!etatRelais; // inverse l'etat du relais
    digitalWrite ( PIN_LED1,etatRelais ) ;
  }
  //Eclairage led 2
  if(ct == 2) {
    etatRelais2=!etatRelais2; // inverse l'etat du relais
    digitalWrite ( PIN_LED2,etatRelais2 ) ;
  }
  //diminue la consigne pid
  if(ct == 7){
      //si mode echo, on desactive
      if(isModeECOLudo)
        setModeECO(0);
      else
        PID_setpoint--;
  }
  //augmente la consigne pid
  if(ct == 8){
      //si mode echo, on desactive
      if(isModeECOLudo)
        setModeECO(0);
      else
        PID_setpoint++;
  }
  //toggle ECO
  if(ct == 10){
    if(isModeECOLudo)
      setModeECO(0);
    else
      setModeECO(1);
  }
}

//=============================
// DISPLAY PRIMITIVES
//=============================
//variables de l'historique
#define HYHIGH   146    //borne haute
#define HYLOW    218    //borne basse
#define XDECAL   6
#define HX       (302-XDECAL)    //point de départ de l'histo
#define HX_LARG  (288-XDECAL)    //largeur totale
#define HY_HAUT  (HYLOW-HYHIGH)         //hauteur totale
#define HYMID    (HYHIGH+(HY_HAUT/2)) //point du milieu
#define HISTO_LEN  (HX_LARG/3) //2     //nombre de points de l'histo
unsigned int historique[HISTO_LEN];      //tableau de valeurs en degres a afficher
unsigned char histo_yval[HISTO_LEN];      //tableau de valeurs converties en y pour mieux effacer
#define HXCOLORBACKGND 0x2124    //couleur d'effacement
int histocolor = ILI9341_RED;    //couleur de la courbe
boolean isBoilerDisplayed = true;

//display histogram
void dispHisto(int tnow) 
{
  unsigned int i,j,tmin,tmax,valy1,valy2,vtmp;
  char buff[255];

  //standby message
  if(isStandBy){
    //dispStandBy();
    return;
  }
  //disp which data is shown
  if(isBoilerDisplayed)
  {
    tft.setCursor(140,148);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE,HXCOLORBACKGND);
    tft.print("BOILER ");
  }else{
    tft.setCursor(140,148);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE,HXCOLORBACKGND);
    tft.print("GROUP  ");
  }
  
  //saves memory space in debug mode otherwise it wont compile
  #ifdef LOG_MESSAGES
    return;
  #endif
  
  //calcul min et max puis shift
  tmin=tnow;tmax=tnow;
  for(i=HISTO_LEN-1;i>0;i--)
  {
    //shift
    historique[i] = historique[i-1];
    //calcul min max
    if((historique[i]<tmin) && (historique[i]>0))
      tmin = historique[i];
    if(historique[i]>tmax)
      tmax = historique[i];      
  }
  historique[0]=tnow;
  //minimal space between min and max
  if((tmax-tnow) < 20)
    tmax+= 20-(tmax-tnow);
  if((tnow-tmin) < 20)
    tmin-= 20-(tnow-tmin);
  //window minimum 4 degrees
  if((tmax-tmin) < 40){
    tmax+= (40-(tmax-tmin))/2;
    tmin-= 40-(tmax-tmin);
  }
  
  //draw histo
  for (i = 1; i < (HISTO_LEN-1); i++)
  { 
    //compute new y value
    if(historique[i])
      valy1 = HYLOW-(((historique[i]-tmin)* HY_HAUT) / (tmax-tmin));
    else
      valy1 = 0;
   if(historique[i-1])
      valy2 = HYLOW-(((historique[i-1]-tmin)* HY_HAUT) / (tmax-tmin));
   else
      valy2 = 0;
   //draw only if values are valid
   if((valy1 & valy2)&&(!isStandBy)){
     //erase previous value
     if(histo_yval[i]&histo_yval[i-1]){
        tft.drawLine(HX-(i*3),histo_yval[i],HX-((i-1)*3),histo_yval[i-1],HXCOLORBACKGND);//ILI9341_BLACK);
        //only for current value, erase Hline
        if(i==1){
           tft.drawLine(HX,histo_yval[i-1],HX+6,histo_yval[i-1],HXCOLORBACKGND);
           tft.setTextSize(1);
           tft.setTextColor(HXCOLORBACKGND,HXCOLORBACKGND);
           tft.setCursor(HX-15, histo_yval[i-1]); 
           sprintf(buff,"%d.%d",historique[1]/10,historique[1]%10);
           tft.print(buff);
        }
     }
     
     //draw new value
     tft.drawLine(HX-(i*3),valy1,HX-((i-1)*3),valy2,histocolor+(i/3)); 
     //only for current value, draw small Hline
     if(i==1){
        //add text information
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_WHITE,HXCOLORBACKGND);
        //display tmax
        tft.setCursor(HX-15, HYHIGH); 
        vtmp= tmax/10;
        if((tmax%10) > 5)
          vtmp++;
        sprintf(buff,"%d",vtmp);
        tft.print(buff);
        //display tmin
        tft.setCursor(HX-15, HYLOW-10);  
        vtmp= tmin/10;
        if((tmin%10) > 5)
          vtmp++;
        sprintf(buff,"%d",vtmp);
        tft.print(buff);
        //display current value
        tft.setCursor(HX-15, valy2);  
        sprintf(buff,"%d.%d",tnow/10,tnow%10);
        tft.print(buff);
        //draw Hline
        tft.drawLine(HX,valy2,HX+6,valy2,ILI9341_RED);
     }
     //update previous value array
     if((!histo_yval[i]) || (i == (HISTO_LEN-2)))
       histo_yval[i]=valy1;
       histo_yval[i-1]=valy2;
       //let the screen breath otherwise bus overflow
       yield();
   }
  } 
}
//reset histo values (fresh start)
void resetHisto(){
  unsigned int i;
  for(i=0;i<HISTO_LEN;i++)
  {
    historique[i] = 0;
    histo_yval[i] = 0;
  }
  //erase histo 
  tft.fillRect(HX+XDECAL-HX_LARG,HYHIGH,HX_LARG+1+XDECAL,HY_HAUT+1,HXCOLORBACKGND);
}
//set boiler for histogram data
void setHistoBoiler(int setb){
  //set boiler histo
  if(setb){
    if(isBoilerDisplayed)
      return;
    else{
      isBoilerDisplayed = true;
      resetHisto();
    }
  }//set group histo
  else{
    if(isBoilerDisplayed == false)
      return;
    else{
      isBoilerDisplayed = false;
      resetHisto();
    }
  }
}

char stbToggle = 1;
//display standby message
void dispStandBy()
{
  //erase histo 
  tft.fillRect(HX+XDECAL-HX_LARG,HYHIGH,HX_LARG+1+XDECAL,HY_HAUT+1,HXCOLORBACKGND);
 // if(stbToggle > 2)
 // {
    tft.setCursor(80,175);
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("STAND BY");
 // }
 // stbToggle++;
 // if(stbToggle>3)
 //   stbToggle=0;
}  

//loading screen
void show_loading(){
  //load first bitmap
  #define BMPFILE "HYBRID23.BMP"
  HWLOGGING.print("loading ");HWLOGGING.println(BMPFILE);
  //load startup screen
  /*
  tft.setRotation(2);
  bmpDraw2(BMPFILE, 0, 0,true);
  //HWLOGGING.print("done loading ");HWLOGGING.println(BMPFILE);
  delay(1000);
*/
  //loading text
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(80,100);
  tft.print("Please wait...");
  
  //load 2nd bitmap
  #define BMPFILE "DASH28.BMP" 
  HWLOGGING.print("loading ");HWLOGGING.println(BMPFILE);
  //load startup screen
  tft.setRotation(2);
  bmpDraw2(BMPFILE, 0, 0,true);
  //HWLOGGING.print("done loading ");HWLOGGING.println(BMPFILE);
}


//=============================
// SETUP
//=============================
void setup()
{
  HWLOGGING.begin(115200);
  HWLOGGING.println("Ludo app start!");
  //setup sensors
  HWLOGGING.println("init-TFT");
  setup_timetodisplay();
  setup_tft();
  
  HWLOGGING.println("init-temp");
  setup_temp();
  
  HWLOGGING.println("init-pid");
  setup_pid();
  
  HWLOGGING.println("init-touch");
  setup_touch();
  
  HWLOGGING.println("init-scale");
  setup_scale();
  
  HWLOGGING.println("init-relais");
  setup_relais();

  //update screen
  HWLOGGING.println("showload");
  show_loading();
  
  //reset display timeout
  reset_timetodisplay(0);
}

//=============================
// TIMEOUT DISPLAY
//=============================
#define TIME_TO_DISPLAY 20 //60 secondes
unsigned long timeStampResteAafficher;

void setup_timetodisplay(){
  //setup TFT backlight pin
  pinMode(TFT_BL, OUTPUT);
  analogWrite(TFT_BL, 255);
  reset_timetodisplay(0);
}

void reset_timetodisplay(int doFade)
{
  timeStampResteAafficher = millis();
  if(isStandBy && doFade){
    tft.fillRect(HX+XDECAL-HX_LARG,HYHIGH,HX_LARG+1+XDECAL,HY_HAUT+1,HXCOLORBACKGND);
    fade(1);
    yield();
  }
//  HWLOGGING.print("rttd");HWLOGGING.println(timeStampResteAafficher);
}

#define BL_STANDBY_VAL 8
//Fade backlight nicely
void fade(char turnON)
{
  int i;
  for(i=BL_STANDBY_VAL;i<=256;i+=8){
    //HWLOGGING.print("going to val:");HWLOGGING.println(255-(i-BL_STANDBY_VAL));
    //fade on
    if(turnON)
      analogWrite(TFT_BL, i-1); 
    //fade off
    else
      analogWrite(TFT_BL, 255-(i-BL_STANDBY_VAL));
    delay(15);
  }
}

//=============================
// LOOP
//=============================
#define MATT_RED_BOILER_OFF 0x60A2
#define MATT_BLUE_ECO_OFF 0x0A49
#define MATT_BLUE_ECO_ON  0x177E  
void loop() 
{ 
  float tSensor1= get_temp(1);//((get_temp()-18)*120)/(25-18);
  float tSensor2 = get_temp(2); 
  int timeDisplayed;
  char sTemp[6];
  char buff[255];
  int ct;
  unsigned int i,j,tnow,tmin,tmax,valy1,valy2;
  char chDegre[] = "\xf7";
  unsigned long loopstart = millis();//profiling data

  //important: we have rotation 3 for text
  tft.setRotation(3);

  //Display timeout
  //-----------------------------  
  //calcul du temps restant a afficher
  timeDisplayed = (millis()-timeStampResteAafficher)/1000;

  //standby?
  if(timeDisplayed>TIME_TO_DISPLAY)
    isStandBy = true;
  else
    isStandBy = false;
    
  //Touch update
  //--------------
  //check switch state different times (for better response time)
  test_switch();
   
  //PID update
  //-----------------------------
  PID_input = tSensor1;
  myPID.Compute();
  //HWLOGGING.println("/PIDOutput/");HWLOGGING.print(PID_output); 

  //affiche la consigne PID sur le moniteur serie
  //HWLOGGING.print(";pid;");
  //HWLOGGING.print(PID_setpoint);

  //Update mode ECO
  //-----------------------------
  if(isModeECOLudo)
    tft.fillRect(291,27,14,4,MATT_BLUE_ECO_ON);
  else
    tft.fillRect(291,27,14,4,MATT_BLUE_ECO_OFF);

  
  //Update voyant de chauffe
  //-----------------------------
  if(PID_output == 0)
    tft.fillRect(291,9,14,8,MATT_RED_BOILER_OFF);
  else if(PID_output <= 50)
    tft.fillRect(291,9,14,8,ILI9341_ORANGE);
  else 
    tft.fillRect(291,9,14,8,ILI9341_RED);
  
  last_PID_output = PID_output;
  //SSR update
  //-----------------------------
  //PID output is percent, duty cycle is 1024, but we must translate to steps of 25 because of zero crossing 50Hz
  int PIDsteps = (int)(PID_output/4);
  int currentduty = (PIDsteps*1024)/25;
  Timer1.setPwmDuty(SSR_PIN,currentduty);
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
  //round temperature, only one number after virgule
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  //sprintf(buff,"%s/%d%sC ",String(tSensor1,1).c_str(),(int)PID_setpoint,chDegre);
  sprintf(buff,"%s",String(tSensor1,1).c_str());
  tft.setCursor(64, 56);  
  tft.print(buff);
  if(isModeECOLudo)
    tft.setTextColor(MATT_BLUE_ECO_ON,ILI9341_BLACK);
  else
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  sprintf(buff,"/%d%sC ",(int)PID_setpoint,chDegre);
  tft.print(buff);
  
  //affiche la temperature de la chaudiere sur le moniteur serie
  HWLOGGING.print(";TC;");
  HWLOGGING.print(tSensor1);
  
  //check switch state different times (for better response time)
  test_switch();

  //Display temperature sensor #2
  //-----------------------------
  //round temperature, only one number after virgule
  //dtostrf(tSensor2, 5, 1, sTemp );
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  sprintf(buff,"%s%sC ",String(tSensor2,1).c_str(),chDegre);
  tft.setCursor(64, 104);
  tft.print(buff);

  //affiche la temperature du groupe sur le moniteur serie
  HWLOGGING.print(";TG;");
  HWLOGGING.print(tSensor2);

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
  int shotMin,shotSec,shotMSec;
  shotMin = (int)(shotTimeMS / 60000);
  shotSec = (int)((shotTimeMS / 1000)- (shotMin*60));
  shotMSec = (int)(((shotTimeMS/100) - (shotMin*600) - (shotSec*10)));
  /*
  if(isChronoPreOn && !isChronoShotOn){
    sprintf(buff,"%d' \n-PRE-",shotSec);
  }  else {
    sprintf(buff,"%d' \n%d' ",(int)(shotPreinfFinalTime/1000), shotSec);
  }
  tft.setCursor(256, 56);
  tft.print(buff);*/
  if(isChronoPreOn && !isChronoShotOn){
    tft.setTextColor(ILI9341_LIGHTGREY,ILI9341_BLACK);
    sprintf(buff,"%d'>> ",shotSec);
    tft.setCursor(256, 48);
    tft.print(buff);
    sprintf(buff,"---");
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setCursor(256, 64);
    tft.print(buff);
  }  else {
    tft.setTextColor(ILI9341_LIGHTGREY,ILI9341_BLACK);
    sprintf(buff,"%d'>> ",(int)(shotPreinfFinalTime/1000));
    tft.setCursor(256, 48);
    tft.print(buff);
    sprintf(buff,"%d' ",shotSec);
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setCursor(256, 64);
    tft.print(buff);
  }

  //affiche le chrono de pr�infusion sur le moniteur serie
  HWLOGGING.print(";ChrPrInf;");
  HWLOGGING.print(shotPreinfFinalTime/1000);
  
  //affiche le chrono d'extration sur le moniteur serie
  HWLOGGING.print(";ChrExtr;");
  HWLOGGING.print(shotSec);

  //Display poids
  //-----------------------------
  //HWLOGGING.print("A7=");HWLOGGING.println(millis()-loopstart);
  float poi = get_poids();
  //HWLOGGING.print("Poids:");HWLOGGING.print(poi);HWLOGGING.println("gr.");
  if (poi>999)
    poi=999;
  if (poi< -99)
    poi=-99;
  
  dtostrf(poi, 3, 0, sTemp );
  sprintf(buff,"%sg",sTemp);
  tft.setCursor(256, 104);
  tft.print(buff);

  //affiche le poids sur le moniteur serie
  HWLOGGING.print(";Poi;");
  HWLOGGING.println(poi);

  //Display histogram of previous temp values
  //-----------------------------------------
  dispHisto((int)(tSensor2*10));

  //Screen timeout and fading 
  //-------------------------
  if(timeDisplayed == (TIME_TO_DISPLAY+1)){
     fade(0);
     dispStandBy();
     //yield();
     //delay(1000);
     return;
  }else if(timeDisplayed > TIME_TO_DISPLAY+10){
     //tft.fillRect(HX+XDECAL-HX_LARG,HYHIGH,HX_LARG+1+XDECAL,HY_HAUT+1,HXCOLORBACKGND);
     //fade(1);
     reset_timetodisplay(1);
     //yield();
     return;
  }
  
  //Idle before loop, sync to 1 second
  //-----------------------------
  yield();
  //int timeToWait = 685 - (millis()-loopstart);
  //if(timeToWait > 0)
  //  delay(timeToWait);
  //HWLOGGING.print("A8=");HWLOGGING.println(millis()-loopstart);  
}


