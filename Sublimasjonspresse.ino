/*
Scetch for controlling a sublimationpress at EVI ski A/S
Written by Andreas Sandaa Hagen during an intership at EVI ski A/S summer 2016

Press is controlled with seperate PID loops for the top and base, type K thermocouplers is used to gage the temperatures.

Inspiration and parts of code gotten from:

Hedinn Gunhildrud's scetch PID_kontroller_v5.ino
http://hackdet.no

Mulitiple button handling by ladyada:
http://www.adafruit.com/blog/2009/10/20/example-code-for-multi-button-checker-with-debouncing/

Adafruit Sous Vide Coocker
https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino?view=all

All adapted for this project by Andreas Sandaa Hagen

Scetch layout
 00 Program setup
 01 EEPROM setup
 02 LCD setup
   02.1 Custom symbols
 03 PID library
 04 States for PID-controller and temperature reader setup
 05 Buttons setup
 06 Serial setup
 07 Timer interupt handler
 08 Setup
   08.1 PID initiation
   08.2 Relay initiation
   08.3 Serial initiation
   08.4 LCD setup, custom symbols and splash screen
   08.5 Button initiation
 09 loop
   09.1 Button loop
   09.2 Operation state loop
   09.3 Print to serial
 10 Temperature alarm
 11 Screen and states
   11.1 Startup
   11.2 Initiate PID
   11.3 Run PID
     11.3.1 Controll loop
     11.3.2 PID relay controll
   11.4 Tuning and calibrating
     11.4.1 Tuning Kp
       11.4.1.1 Top
       11.4.1.2 Base
     11.4.2 Tuning Ki
       11.4.2.1 Top
       11.4.2.2 Base
     11.4.3 Tuning Kd
       11.4.3.1 Top
       11.4.3.2 Base
     11.4.4 Manual calibrating of temperature sensor
       11.4.4.1 Top
       11.4.4.2 Base
   11.5 Setting setpoint
   11.6 Selecting program
     11.6.1 Manual
     11.6.2 Thickness 1
     11.6.3 Thickness 2
     11.6.4 Thickness 3
   11.7 Shut down PID
   11.8 Save to EEPROM
 12 Temperature reading
 13 Reading buttons
   13.1 Button values
     13.1.1 Set
     13.1.2 Down
     13.1.3 Up
     13.1.4 Right
     13.1.5 Left
 14 EEPROM
   14.1 Saving
   14.2 Loading
   14.3 Floating
     14.3.1 Writing
     14.3.2 Reading
 15 Datalog
*/


// =============================
// 00 Program settings
// =============================
// Time is given in seconds
  // =============================
  // 00.1 Program 1
  // =============================
  // Time
  int p1time = 120;
  // Temperatures
    // Top
  int p1tempT = 170;
    // Base
  int p1tempB = 172;
  // =============================
  // 00.2 Program 2
  // =============================
  // Time
  int p2time = 130;
  // Temperatures
    // Top
  int p2tempT = 170;
    // Base
  int p2tempB = 172;
  // =============================
  // 00.3 Program 3
  // =============================
  // Time
  int p3time = 140;
  // Temperatures
    // Top
  int p3tempT = 170;
    // Base
  int p3tempB = 172;



// =============================
// 01 EEPROM setup
// =============================
#include <EEPROM.h>

const int spAddresst = 0;
const int tkAddresst = 8;
const int kpAddresst = 16;
const int kiAddresst = 24;
const int kdAddresst = 32;
const int spAddressb = 40;
const int tkAddressb = 48;
const int kpAddressb = 56;
const int kiAddressb = 64;
const int kdAddressb = 72;
const int stimAddress = 80;

// =============================
// 02 LCD setup
// =============================
#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// =============================
//  02.1 Custom symbols
// =============================
// Custom symbols

// Percent (%) -symbol
byte pr[8] = {
  B00000,
  B10001,
  B00010,
  B00100,
  B01000,
  B10001,
  B00000,
};

// Degree (˚) -symbol
byte degr[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
};

// Norwegian symbols
byte oe[8] = {
  B00000,
  B00000,
  B01101,
  B10010,
  B10101,
  B01001,
  B10110,
};

byte aa[8] = {
  B01110,
  B00100,
  B01110,
  B00001,
  B01111,
  B10001,
  B01111,
};
int blinkNo;
// =============================
// 03 PID, Thermocouple & Solenoid libraries and variables
// =============================

// PID
  #include <PID_v1.h>

  // Define Variables we'll be connecting to
  double setPointT, InputT, OutputT, setPointB, InputB, OutputB;

  // PID tuning parameters
  double KpT, KiT, KdT, KpB, KiB, KdB;

  //Specify the links and initial tuning parameters
  PID PIDT(&InputT, &OutputT, &setPointT, KpT, KiT, KdT, DIRECT);
  PID PIDB(&InputB, &OutputB, &setPointB, KpB, KiB, KdB, DIRECT);

  // Time Proportional Output window.
  int WindowSizeT = 1000;
  unsigned long windowStartTimeT;
  int WindowSizeB = 1000;
  unsigned long windowStartTimeB;

  // Relay pin, control with optocoupler
  #define relayPinT 28
  #define relayPinB 30
  int relayStateT = 0; // Relay sate, 0 = off, 1 = on
  int relayStateB = 0; // Relay sate, 0 = off, 1 = on
// Relay indication pins
  #define redLEDpinT 6
  #define redLEDpinB 5
// Thermocouple - Adafruit_MAX31855
  #include <SPI.h>
  #include "Adafruit_MAX31855.h"
  #include <Wire.h>

  #define MAXDO   48
  #define MAXCLK  50
  #define MAXCST  51
  #define MAXCSB  49

  Adafruit_MAX31855 thermocoupleT(MAXCLK, MAXCST, MAXDO);
  Adafruit_MAX31855 thermocoupleB(MAXCLK, MAXCSB, MAXDO);
// Solenoid
  #define solenoidPin 26
  #define solenoidled 13
  int solenoidState = 0;
  int stime = 60;

// =============================
// 04 States for PID-controller and temperature reader setup
// =============================

enum operatingState { STARTUP = 0,
 TURN_ON_PID, RUN_PID, C_PT, C_IT, C_DT, C_PB, C_IB, C_DB, C_MTOP, C_MBASE, C_MTIME,  START_PROG1,
 START_PROG2, START_PROG3, C_TMPT, C_TMPB, TURN_OFF,
 SETPOINT1, SETPOINT2, PROGRAM, CALIBRATE, MANUALS, SOLENOID
};
operatingState opState = STARTUP;

// Temperature reader setup

float tmpcT;
float tmpcB;
float tempAkumT;
float tempAkumB;
float tempAvgT;
float tempAvgB;
float tempAvgPrevT;
float tempAvgPrevB;
int readNo;
int tKT;
int tKB;

// =============================
// 05 Button setup
// =============================
// Define buttons
#define DEBOUNCE 10  // Debouncer for buttons, set for best functionality
// Four display buttons, set and activate
byte buttons[] = {54, 55, 56, 57, 58, 59}; // Analog pins for button input.
#define NUMBUTTONS sizeof(buttons) // Macro for number of buttons.
volatile byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS]; // Checking of a button is "just pressed", "just released" or "pressed"

//
// Variabler for knapper
//

long buttonPushTime; // Variable that stores the time when a button was pressed

long buttonPushTime2; // Variable that stores the time when a button was pressed
long buttonDelay = 500; // Delay before a value is set, starts when a button is pressed

// =============================
// 06 Serial setup
// =============================

const int printDelay = 1000; // Print to serial timer.
float printTime; // Variable to calculate the print time.

// =============================
// 07 Timer interupt handler
// =============================

// Timer 2
// handels temperature and switches
SIGNAL(TIMER2_OVF_vect) {
  check_switches();
//  get_temp();

  if (opState == STARTUP)
  {
    digitalWrite(relayPinT, LOW);  // make sure relay is off
    digitalWrite(relayPinB, LOW);
  }
  else
  {
    drive_outputT();
    drive_outputB();
  }
} // Signal Timer SLUTT

// Timer 3
// Handles the timer for the solenoid
SIGNAL(TIMER3_OVF_vect){
  count();
}
unsigned long ticktime3;
void count(){
  ticktime3 = ticktime3 +1;
TCNT3 = 49910; // Set to 1 second. Calculated as =65536-(16MHz/(1024 prescaler*1Hz))-1
}
SIGNAL(TIMER4_OVF_vect){

  TCNT4 = 62410; // 5 Hz
//  TCNT4 = 63303; // 7 Hz
  get_temp();

  TCNT4 = 62410;
  get_temp();
//TCNT4= 13452;
}
// =============================
// 08 setup
// =============================
void setup() {

 // =============================
 // 08.1 PID initiation
 // =============================
 windowStartTimeT = millis();
 windowStartTimeB = millis();
 LoadParameters();
 PIDT.SetTunings(KpT,KiT,KdT);
 PIDT.SetSampleTime(1000);

 PIDT.SetOutputLimits(0, WindowSizeT);
 PIDT.SetMode(MANUAL);

 PIDB.SetTunings(KpB,KiB,KdB);
 PIDB.SetSampleTime(1000);

 PIDB.SetOutputLimits(0, WindowSizeB);
 PIDB.SetMode(MANUAL);
 // =============================
 // 08.2 Relay initiation
 // =============================
 pinMode(relayPinT, OUTPUT);
 digitalWrite(relayPinT, LOW);
 pinMode(redLEDpinT, OUTPUT);
 pinMode(relayPinB, OUTPUT);
 digitalWrite(relayPinB, LOW);
 pinMode(redLEDpinB, OUTPUT);

 // =============================
 // 08.3 Serial initiation
 // =============================

 Serial.begin(9600);
 printTime = millis();
 // Relay states
 relayStateT = 0;
 relayStateB = 0;
 solenoidState = 0;

 // =============================
 // 08.4 LCD setup, custom symbols and splash screen
 // =============================

 lcd.begin(16,2);
 lcd.clear();

 // Custom symbols:
 lcd.createChar(0, degr);
 lcd.createChar(1, pr);
 lcd.createChar(2, oe);
 lcd.createChar(3, aa);

 // Startup Splash-screen
 lcd.setCursor(4,0);
 lcd.print("EVI  Ski");
 lcd.setCursor(2,1);
 lcd.print("Sommer 2016");
 delay(2000);

 lcd.clear();
 lcd.print("Skript:");
 lcd.setCursor(0,1);
 lcd.print("Andreas Sandaa");
 delay(2000);

 lcd.clear();

 // =============================
 //  08.5 Button initiation
 // =============================

 byte i;

 for (i=0; i< NUMBUTTONS; i++){
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
 }
 // ============================
 // 08.6 Interupt timer
 // ============================
noInterrupts();
// Timer 2 is set to run every 15 mS, 1024 prescaler
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;

// Timer 3 is set to run every 1 s, 1024 prescaler
  TCCR3A = 0;
  TCCR3B = 0;

  //TCNT1 = 49911;
  TCCR3B |= (1<< CS32);
  TCCR3B |= (1<< CS30);
  TIMSK3 |= (1<< TOIE4);

  TCCR4A = 0;
  TCCR4B = 0;

  TCCR4B |= (1<< CS42);
  TCCR4B |= (1<< CS40);
  TIMSK4 |= (1<< TOIE4);


interrupts();
 // ============================
 // 08.7 Solenoid initiation
 //=============================
pinMode(solenoidPin, OUTPUT);
digitalWrite(solenoidPin, LOW);
pinMode(solenoidled, OUTPUT);
digitalWrite(solenoidled, LOW);


delay(1000);
}

// =============================
// 09 loop
// =============================
void loop() {
 // =============================
 //  09.1 Button loop
 // =============================
  for (byte i = 0; i < NUMBUTTONS; i++){
    if (justpressed[i]){
      justpressed[i] = 0;
      buttonPushTime = millis();
      buttonPushTime2 = millis();
    }
    if (justreleased[i]){
      switch(i){
        case 0:
          setValue();
        break;
        case 1:
        lcd.clear();
        changeValueDown();
        break;
      case 2:
        lcd.clear();
        changeScreenRight();
        break;
      case 3:
        lcd.clear();
        changeValueUp();
        break;
      case 4:
        lcd.clear();
        changeScreenLeft();
        break;
      case 5:
        lcd.clear();
        ticktime3 = 0;
        opState = SOLENOID;
    }
    justreleased[i]=0;
    }
    if (pressed[i]) {
      buttonPushTime = millis(); // Keeps the screen from going back to RUN_PID while a button is pressed.
      switch(i){
         case 0:
         //   setValue();
        break;
        case 1:
          if (buttonPushTime2 + buttonDelay < millis()){
            lcd.clear();
            changeValueDown();
          }
          break;
        case 2:
        //   changeScreenRight();
          break;
        case 3:
          if (buttonPushTime2 + buttonDelay < millis()){
            lcd.clear();
            changeValueUp();
          }
          break;
        case 4:
        //   changeScreenLeft();
          break;
      } // switch SLUTT
    } // if (pressed) SLUTT
  }
 // =============================
 // 09.2 Operation state loop
 // =============================


 switch (opState) {
   // Main menu
  case STARTUP:
    start_up();
    break;
  case TURN_ON_PID:
    turn_on_pid();
    break;
  case RUN_PID:
    run_pid();
    break;
  case MANUALS:
    manuals();
    break;
  case PROGRAM:
    program();
    break;
  case CALIBRATE:
    calibrate();
    break;
  case TURN_OFF:
    turn_off();
    break;
  // Manual menu
  case C_MTOP:
    c_mtop();
    break;
  case C_MBASE:
    c_mbase();
    break;
  case C_MTIME:
    c_mtime();
    break;
  // Program menu
  case START_PROG1:
    prog1();
    break;
  case START_PROG2:
    prog2();
    break;
  case START_PROG3:
    prog3();
    break;
  // Calibrate menu
  case C_PT:
    c_pt();
    break;
  case C_IT:
    c_it();
    break;
  case C_DT:
    c_dt();
    break;
  case C_PB:
    c_pb();
    break;
  case C_IB:
    c_ib();
    break;
  case C_DB:
    c_db();
    break;
  case C_TMPT:
    c_tmpT();
    break;
  case C_TMPB:
    c_tmpB();
    break;
  // Setpoint menu
  case SETPOINT1:
    setpoint1();
    break;
  case SETPOINT2:
    setpoint2();
    break;
  case SOLENOID:
    solenoid();
    break;
 }
 // =============================
 // 09.3 Datalog
 // =============================
 if (printTime + printDelay <= millis()){
   logData();
   printTime = millis();
   }
 // =============================
 // 09.4 Emergency shutoff
 // =============================
/*
 if (tempAvgT >= 215){
   emergency_off();
 }
 if (tempAvgB >= 215){
   emergency_off();
 }
 delay(1);
*/
}

// =============================
// 10 Temperature alarm
// =============================

void emergency_off(){
  PIDT.SetMode(MANUAL);
  digitalWrite(relayPinT, LOW);
  relayStateT = 0;
  digitalWrite(redLEDpinT, LOW);
  PIDB.SetMode(MANUAL);
  digitalWrite(relayPinB, LOW);
  relayStateB = 0;
  digitalWrite(redLEDpinB, LOW);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("TEMPERATUR-ALARM");
  lcd.setCursor(0,1);
  lcd.print("T: ");
  lcd.print((byte)tempAvgT);
  lcd.write(byte(0));
  lcd.setCursor(7,1);
  lcd.print(" B: ");
  lcd.print((byte)tempAvgB);
  lcd.write(byte(0));
  opState = STARTUP;

blinkNo = 0;
  while(blinkNo < 7){
  lcd.clear();
    delay(500);
  lcd.setCursor(0,0);
  lcd.print("TEMPERATUR-ALARM");
  lcd.setCursor(0,1);
  lcd.print("T: ");
  lcd.print((byte)tempAvgT);
  lcd.write(byte(0));
  lcd.setCursor(7,1);
  lcd.print(" B: ");
  lcd.print((byte)tempAvgB);
  lcd.write(byte(0));
    delay(500);
    blinkNo++;
    }

  lcd.clear();
//  delay(10000);
  start_up();
 }

// =============================
// 11 Screen and states
// =============================

 // =============================
 //  11.1 Startup
 // =============================
char message[] = "Sublimasjonspresse";
 void start_up() {
   PIDT.SetMode(MANUAL);
   PIDB.SetMode(MANUAL);
   digitalWrite(relayPinT, LOW);
   relayStateT = 0;
   digitalWrite(relayPinB, LOW);
   relayStateB = 0;
   /*for (int printStart = 15; printStart >= 0; printStart--)  {
     showLetters(printStart, 0);
   }
   for (int letter = 1; letter <= strlen(message); letter++) {
     showLetters(0, letter);
   }*/
   lcd.setCursor(4,0);
   lcd.print("EVI  Ski");
   lcd.setCursor(3,1);
   lcd.print("Trykk  set");

 }


 void showLetters(int printStart, int startLetter) {
  lcd.setCursor(4,0);
  lcd.print("EVI  Ski");
  lcd.setCursor(printStart,1);
  for (int currentLetter = startLetter; currentLetter < strlen(message); currentLetter++)
  {
    lcd.print(message[currentLetter]);
  }
  lcd.print(" ");
  delay(200);
 }

 // =============================
 //  11.2 Initiate PID
 // =============================
 void turn_on_pid()  {
   PIDT.SetMode(AUTOMATIC);
   PIDB.SetMode(AUTOMATIC);
   windowStartTimeT = millis();
   windowStartTimeB = millis();
   opState = RUN_PID;
   lcd.clear();
 }

 // =============================
 //  11.3 Run PID
 // =============================
 void run_pid() {
   lcd.setCursor(0,0);
   lcd.print("Varmer, temp:");
   lcd.setCursor(0,1);
   lcd.print("T: ");
   if (tempAvgT != tempAvgPrevT) {
     lcd.setCursor(3,1);
     if (tempAvgT < 100){
       lcd.print(" ");
     }
     if (tempAvgT < 10){
       lcd.print("  ");
     }
      lcd.print((byte)tempAvgT);
    lcd.setCursor(6,1);
    lcd.write(byte(0));
  }
 lcd.setCursor(7,1);
 lcd.print(" B: ");
  if (tempAvgB != tempAvgPrevB) {
    lcd.setCursor(11,1);
    if (tempAvgB < 100){
      lcd.print(" ");
    }
    if (tempAvgB < 10){
      lcd.print("  ");
    }
    lcd.print((byte)tempAvgB);
    lcd.setCursor(14,1);
    lcd.write(byte(0));
    lcd.print(" ");
  }
    run_pid_compute();

 }

 // =============================
 //   11.3.1 Control loop
 // =============================
 void run_pid_compute() {
   InputT = tempAvgT;
   PIDT.Compute();
   InputB = tempAvgB;
   PIDB.Compute();
 }

 // =============================
 //   11.3.2 PID relay control
 // =============================
 void drive_outputT() {
   unsigned long now = millis();
   if(now - windowStartTimeT>WindowSizeT)
   {
     windowStartTimeT += WindowSizeT;
   }
   if (OutputT > now - windowStartTimeT){
     digitalWrite(relayPinT, HIGH);
     relayStateT = 1;
     digitalWrite(redLEDpinT, HIGH);
   }
   else{
     digitalWrite(relayPinT, LOW);
     digitalWrite(redLEDpinT, LOW);
     relayStateT = 0;
   }
 }
 void drive_outputB() {
   unsigned long now = millis();
   if(now - windowStartTimeB>WindowSizeB)
   {
     windowStartTimeB += WindowSizeB;
   }
   if (OutputB > now - windowStartTimeB){
     digitalWrite(relayPinB, HIGH);
     relayStateB = 1;
     digitalWrite(redLEDpinB, HIGH);
   }
   else{
     digitalWrite(relayPinB, LOW);
     digitalWrite(redLEDpinB, LOW);
     relayStateB = 0;
   }
 }

 // =============================
 //  11.4 Tuning and calibrating
 // =============================
  void calibrate(){
    lcd.setCursor(0,0);
    lcd.print("For kalibrering");
    lcd.setCursor(0,1);
    lcd.print("trykk set");
  }
   // =============================
   //  11.4.1 Tuning Kp
   // =============================
      // =============================
      // 11.4.1.1 Top
      // =============================
      void c_pt() {
        lcd.setCursor(0,0);
        lcd.print("PID-tuning topp:");

          // Printing Kp
          lcd.setCursor(0,1);
          lcd.print(">");
          lcd.setCursor(1,1);
          lcd.print("Kp");
          lcd.setCursor(3,1);
          lcd.print("  ");
        if (KpT < 10){
          lcd.setCursor(5,1);
          }
        if (KpT >= 10){
          lcd.setCursor(4,1);
          }
        if (KpT >= 100){
          lcd.setCursor(3,1);
          }
          lcd.print(byte(KpT), DEC);

          // Printing Ki
          lcd.setCursor(7,1);
          lcd.print("Ki");
          lcd.setCursor(9,1);
          lcd.print("  ");
        if (KiT < 10){
          lcd.setCursor(10,1);
          }
        if (KiT >= 10){
          lcd.setCursor(9,1);
          }
          lcd.print(byte(KiT));

          // Printing Kd
          lcd.setCursor(12,1);
          lcd.print("Kd");
          lcd.setCursor(14,1);
          lcd.print("  ");
        if (KdT < 10){
          lcd.setCursor(15,1);
          }
        if (KdT >= 10){
          lcd.setCursor(14,1);
          }
          lcd.print(byte(KdT));

         if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
             PIDT.SetTunings(KpT,KiT,KdT);
             lcd.clear();
             opState = RUN_PID;
             return;
         }
      }
      // =============================
      // 11.4.1.2 Base
      // =============================
      void c_pb() {
        lcd.setCursor(0,0);
        lcd.print("PID-tuning bunn:");

           // Printing Kp
          lcd.setCursor(0,1);
          lcd.print(">");
          lcd.setCursor(1,1);
          lcd.print("Kp");
          lcd.setCursor(3,1);
          lcd.print("  ");
        if (KpB < 10){
          lcd.setCursor(5,1);
          }
        if (KpB >= 10){
          lcd.setCursor(4,1);
          }
        if (KpB >= 100){
          lcd.setCursor(3,1);
          }
          lcd.print(byte(KpB), DEC);

          // Printing Ki
          lcd.setCursor(7,1);
          lcd.print("Ki");
          lcd.setCursor(9,1);
          lcd.print("  ");
        if (KiB < 10){
          lcd.setCursor(10,1);
          }
        if (KiB >= 10){
          lcd.setCursor(9,1);
          }
          lcd.print(byte(KiB));

          // Printing Kd
          lcd.setCursor(12,1);
          lcd.print("Kd");
          lcd.setCursor(14,1);
          lcd.print("  ");
        if (KdB < 10){
          lcd.setCursor(15,1);
          }
        if (KdB >= 10){
          lcd.setCursor(14,1);
          }
          lcd.print(byte(KdB));

         if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
             PIDB.SetTunings(KpB,KiB,KdB);
             lcd.clear();
             opState = RUN_PID;
             return;
         }
      }
   // =============================
   //  11.4.2 Tuning Ki
   // =============================
     // =============================
     // 11.4.2.1 Top
     // =============================
     void c_it() {
       lcd.setCursor(0,0);
       lcd.print("PID-tuning topp:");

         // Printing Kp
         lcd.setCursor(1,1);
         lcd.print("Kp");
         lcd.setCursor(3,1);
         lcd.print("  ");
       if (KpT < 10){
         lcd.setCursor(5,1);
         }
       if (KpT >= 10){
         lcd.setCursor(4,1);
         }
       if (KpT >= 100){
         lcd.setCursor(3,1);
         }
         lcd.print(byte(KpT), DEC);

         // Printing Ki
         lcd.setCursor(6,1);
         lcd.print(">");
         lcd.setCursor(7,1);
         lcd.print("Ki");
         lcd.setCursor(9,1);
         lcd.print("  ");
       if (KiT < 10){
         lcd.setCursor(10,1);
         }
       if (KiT >= 10){
         lcd.setCursor(9,1);
         }
         lcd.print(byte(KiT));

         // Printing Kd
         lcd.setCursor(12,1);
         lcd.print("Kd");
         lcd.setCursor(14,1);
         lcd.print("  ");
       if (KdT < 10){
         lcd.setCursor(15,1);
         }
       if (KdT >= 10){
         lcd.setCursor(14,1);
         }
         lcd.print(byte(KdT));

        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
            PIDT.SetTunings(KpT,KiT,KdT);
            lcd.clear();
            opState = RUN_PID;
            return;
        }
     }
     // =============================
     // 11.4.2.2 Base
     // =============================
     void c_ib() {
       lcd.setCursor(0,0);
       lcd.print("PID-tuning bunn:");

          // Printing Kp
         lcd.setCursor(1,1);
         lcd.print("Kp");
         lcd.setCursor(3,1);
         lcd.print("  ");
       if (KpB < 10){
         lcd.setCursor(5,1);
         }
       if (KpB >= 10){
         lcd.setCursor(4,1);
         }
       if (KpB >= 100){
         lcd.setCursor(3,1);
         }
         lcd.print(byte(KpB), DEC);

         // Printing Ki
         lcd.setCursor(6,1);
         lcd.print(">");
         lcd.setCursor(7,1);
         lcd.print("Ki");
         lcd.setCursor(9,1);
         lcd.print("  ");
       if (KiB < 10){
         lcd.setCursor(10,1);
         }
       if (KiB >= 10){
         lcd.setCursor(9,1);
         }
         lcd.print(byte(KiB));

         // Printing Kd
         lcd.setCursor(12,1);
         lcd.print("Kd");
         lcd.setCursor(14,1);
         lcd.print("  ");
       if (KdB < 10){
         lcd.setCursor(15,1);
         }
       if (KdB >= 10){
         lcd.setCursor(14,1);
         }
         lcd.print(byte(KdB));

        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
            PIDB.SetTunings(KpB,KiB,KdB);
            lcd.clear();
            opState = RUN_PID;
            return;
        }
     }
   // =============================
   //  11.4.3 Tuning Kd
   // =============================
     // =============================
     // 11.4.3.1 Top
     // =============================
     void c_dt() {
       lcd.setCursor(0,0);
       lcd.print("PID-tuning topp:");

         // Printing Kp
         lcd.setCursor(1,1);
         lcd.print("Kp");
         lcd.setCursor(3,1);
         lcd.print("  ");
       if (KpT < 10){
         lcd.setCursor(5,1);
         }
       if (KpT >= 10){
         lcd.setCursor(4,1);
         }
       if (KpT >= 100){
         lcd.setCursor(3,1);
         }
         lcd.print(byte(KpT), DEC);

         // Printing Ki
         lcd.setCursor(7,1);
         lcd.print("Ki");
         lcd.setCursor(9,1);
         lcd.print("  ");
       if (KiT < 10){
         lcd.setCursor(10,1);
         }
       if (KiT >= 10){
         lcd.setCursor(9,1);
         }
         lcd.print(byte(KiT));

         // Printing Kd
         lcd.setCursor(11,1);
         lcd.print(">");
         lcd.setCursor(12,1);
         lcd.print("Kd");
         lcd.setCursor(14,1);
         lcd.print("  ");
       if (KdT < 10){
         lcd.setCursor(15,1);
         }
       if (KdT >= 10){
         lcd.setCursor(14,1);
         }
         lcd.print(byte(KdT));

        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
            PIDT.SetTunings(KpT,KiT,KdT);
            lcd.clear();
            opState = RUN_PID;
            return;
        }
     }
     // =============================
     // 11.4.3.2 Base
     // =============================
     void c_db() {
       lcd.setCursor(0,0);
       lcd.print("PID-tuning bunn:");
          // Printing Kp
         lcd.setCursor(1,1);
         lcd.print("Kp");
         lcd.setCursor(3,1);
         lcd.print("  ");
       if (KpB < 10){
         lcd.setCursor(5,1);
         }
       if (KpB >= 10){
         lcd.setCursor(4,1);
         }
       if (KpB >= 100){
         lcd.setCursor(3,1);
         }
         lcd.print(byte(KpB), DEC);

         // Printing Ki
         lcd.setCursor(7,1);
         lcd.print("Ki");
         lcd.setCursor(9,1);
         lcd.print("  ");
       if (KiB < 10){
         lcd.setCursor(10,1);
         }
       if (KiB >= 10){
         lcd.setCursor(9,1);
         }
         lcd.print(byte(KiB));

         // Printing Kd
         lcd.setCursor(11,1);
         lcd.print(">");
         lcd.setCursor(12,1);
         lcd.print("Kd");
         lcd.setCursor(14,1);
         lcd.print("  ");
       if (KdB < 10){
         lcd.setCursor(15,1);
         }
       if (KdB >= 10){
         lcd.setCursor(14,1);
         }
         lcd.print(byte(KdB));

        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
            PIDB.SetTunings(KpB,KiB,KdB);
            lcd.clear();
            opState = RUN_PID;
            return;
        }
     }
   // =============================
   //  11.4.4 Manual calibration of temperature sensor
   // =============================
     // =============================
     // 11.4.4.1 Top
     // =============================
     void c_tmpT(){

     lcd.setCursor(0,0);
      lcd.print("Kalib av ");
      lcd.setCursor(10,0);
      lcd.write(byte(0));
      lcd.setCursor(11,0);
      lcd.print(" topp");
     lcd.setCursor(0,1);
      lcd.print("Temp");
     // Printing new value only if there is a change since previous reading.
        if (tempAvgT != tempAvgPrevT) {
         if (tempAvgT < 10){
           lcd.setCursor(5,1);
           lcd.print(" ");
           lcd.setCursor(6,1);
           }
         if (tempAvgT >= 10){
           lcd.setCursor(5,1);
           }
         lcd.print((byte)tempAvgT);
         lcd.write(byte(0));
       }
     // Printing addition or subtraction of read temperature
      lcd.setCursor(9,1);
      lcd.print(">tK ");
     	if (tKT >= 0){
     		lcd.print("+");
     	}
     //	if (tK < 0){
     //		lcd.print("-");
     //	}
     // lcd.setCursor(14,1);
      lcd.print(tKT);
      lcd.write(byte(0));

           if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
              lcd.clear();
              opState = RUN_PID;
              return;
          }
     }
     // =============================
     // 11.4.4.2 Base
     // =============================
      void c_tmpB(){
        lcd.setCursor(0,0);
        lcd.print("Kalib av ");
        lcd.setCursor(10,0);
        lcd.write(byte(0));
        lcd.setCursor(11,0);
        lcd.print(" bunn");
        lcd.setCursor(0,1);
         lcd.print("Temp");
        // Printing new value only if there is a change since previous reading.
           if (tempAvgB != tempAvgPrevB) {
            if (tempAvgB < 10){
              lcd.setCursor(5,1);
              lcd.print(" ");
              lcd.setCursor(6,1);
              }
            if (tempAvgB >= 10){
              lcd.setCursor(5,1);
              }
            lcd.print((byte)tempAvgB);
            lcd.write(byte(0));
          }
        // Printing addition or subtraction of read temperature
         lcd.setCursor(9,1);
         lcd.print(">tK ");
        	if (tKB >= 0){
        		lcd.print("+");
        	}
         lcd.print(tKB);
         lcd.write(byte(0));

              if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
                 lcd.clear();
                 opState = RUN_PID;
                 return;
             }
      }
 // =============================
 //  11.5 Setting setpoint
 // =============================
   // =============================
   //  11.5 Setpoint1
   // =============================
      void setpoint1(){
        lcd.setCursor(0,0);
        lcd.print("Endre set point");
        lcd.setCursor(0,1);
        lcd.print(">T:");
        lcd.setCursor(3,1);
        if (setPointT >= 100){
        }
        if(setPointT < 100){
          lcd.print(" ");
        }
        if (setPointT < 10){
          lcd.print("  ");
        }
        lcd.print((byte)setPointT,DEC);
        lcd.setCursor(6,1);
        lcd.write(byte(0));
        lcd.setCursor(8,1);
        lcd.print(" B:");
        if (setPointB >= 100){
          lcd.setCursor(11,1);
        }
        if(setPointB < 100){
          lcd.print(" ");
          lcd.setCursor(12,1);
        }
        if (setPointB < 10){
          lcd.print("  ");
        }
        lcd.print((byte)setPointB,DEC);
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
           lcd.clear();
           opState = RUN_PID;
           return;
       }
          lcd.setCursor(13,1);
        }
        lcd.print((byte)setPointB,DEC);
        lcd.setCursor(14,1);
        lcd.print(byte(0));
      }
   // =============================
   //  11.5 Setpoint2
   // =============================
   void setpoint2(){
     lcd.setCursor(0,0);
     lcd.print("Endre set point");
     lcd.setCursor(0,1);
     lcd.print(" T:");
     lcd.setCursor(3,1);
     if (setPointT >= 100){
     }
     if(setPointT < 100){
       lcd.print(" ");
     }
     if(setPointT < 10){
       lcd.print("  ");
     }
     lcd.print((byte)setPointT,DEC);
     lcd.setCursor(6,1);
     lcd.write(byte(0));
     lcd.setCursor(8,1);
     lcd.print(">B:");
     if (setPointB >= 100){
     }
     if(setPointB < 100){
       lcd.print(" ");
     }
     if(setPointB < 10){
       lcd.print("  ");
     }
     lcd.print((byte)setPointB,DEC);
     lcd.setCursor(14,1);
     lcd.write(byte(0));
     if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
        lcd.clear();
        opState = RUN_PID;
        return;
    }
   }
 // =============================
 //  11.6 Selecting program
 // =============================
    /*
    Dimentions:
    0.1-1 mm
    Temperature range:
    172 - 200 degrees centigrade
    Posibility for setting temperature and time manually.
    */
    void program(){
      lcd.setCursor(0,0);
      lcd.print("For program");
      lcd.setCursor(0,1);
      lcd.print("trykk set");
    }
    // Program routine
      int progrm(int stimep,int temptp,int tempbp){
        lcd.setCursor(0,0);
        lcd.print("Tid:");
        if (stimep >= 100){
          lcd.setCursor(4,0);
        }
        if(stimep < 99){
          lcd.setCursor(5,0);
        }
        if(stimep < 10){
          lcd.setCursor(6,0);
        }
        lcd.print((byte)stimep,DEC);
        lcd.setCursor(7,0);
        lcd.print(" sekunder");
        lcd.setCursor(1,1);
        lcd.print("T:");
        if (temptp >= 100){
          lcd.setCursor(2,1);
        }
        if(temptp >= 10){
          lcd.setCursor(3,1);
        }
        if(temptp < 10){
          lcd.setCursor(4,1);
        }
        lcd.print((byte)temptp,DEC);
        lcd.setCursor(6,1);
        lcd.write(byte(0));
        lcd.setCursor(8,1);
        lcd.print(" B:");
        if (tempbp >= 100){
          lcd.setCursor(10,1);
        }
        if(tempbp >= 10){
          lcd.setCursor(11,1);
        }
        if(tempbp < 10){
          lcd.setCursor(12,1);
        }
        lcd.print((byte)tempbp,DEC);
        lcd.setCursor(14,1);
        lcd.write(byte(0));

            if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
               lcd.clear();
               opState = RUN_PID;
               //return;
           }
      }
    // =============================
    //   11.8.1 Manual
    // =============================
    void manuals(){
      lcd.setCursor(0,0);
      lcd.print("For manuell");
      lcd.setCursor(0,1);
      lcd.print("trykk set");
    }
      // =============================
      //   11.8.1.1 Set time
      // =============================
      void c_mtime(){
        lcd.setCursor(0,0);
        lcd.print(">Tid:");
        if (stime >= 100){
          lcd.setCursor(6,0);
        }
        if(stime <= 99){
          lcd.print(" ");
          lcd.setCursor(7,0);
        }
        if(stime < 10){
          lcd.print("  ");
          lcd.setCursor(8,0);
        }
        lcd.print((byte)stime,DEC);
        lcd.setCursor(9,0);
        lcd.print(" sek");
        lcd.setCursor(1,1);
        lcd.print("T:");
        lcd.setCursor(3,1);
        if (stime >= 100){

        }
        if(setPointT <= 99){
          lcd.print(" ");

        }
        if(setPointT < 10){
          lcd.print("  ");

        }
        lcd.print((byte)setPointT,DEC);
        lcd.setCursor(6,1);
        lcd.write(byte(0));
        lcd.setCursor(8,1);
        lcd.print(" B:");
        if (setPointB >= 100){
          lcd.setCursor(11,1);
        }
        if(setPointB <= 99){
          lcd.print(" ");
          lcd.setCursor(12,1);
        }
        if(setPointB < 10){
          lcd.print("  ");
          lcd.setCursor(13,1);
        }
        lcd.print((byte)setPointB,DEC);
        lcd.setCursor(14,1);
        lcd.write(byte(0));

            if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
               lcd.clear();
               opState = RUN_PID;
               return;
           }
      }
      // =============================
      //   11.8.1.2 Set temperature top
      // =============================
      void c_mtop(){
        lcd.setCursor(1,0);
        lcd.print("Tid:");
        if (stime >= 100){
          lcd.setCursor(6,0);
        }
        if(stime < 99){
          lcd.print(" ");
          lcd.setCursor(7,0);
        }
        if(stime < 10){
          lcd.print("  ");
          lcd.setCursor(8,0);
        }
        lcd.print((byte)stime,DEC);
        lcd.setCursor(9,0);
        lcd.print(" sek");
        lcd.setCursor(0,1);
        lcd.print(">T:");
        if (setPointT >= 100){
          lcd.setCursor(3,1);
        }
        if(setPointT <= 99){
          lcd.print(" ");
          lcd.setCursor(4,1);
        }
        if(setPointT < 10){
          lcd.print("  ");
          lcd.setCursor(5,1);
        }
        lcd.print((byte)setPointT,DEC);
        lcd.setCursor(6,1);
        lcd.write(byte(0));
        lcd.setCursor(8,1);
        lcd.print(" B:");
        if (setPointB >= 100){
          lcd.setCursor(11,1);
        }
        if(setPointB <= 99){
          lcd.print(" ");
          lcd.setCursor(12,1);
        }
        if(setPointB < 10){
          lcd.print("  ");
          lcd.setCursor(13,1);
        }
        lcd.print((byte)setPointB,DEC);
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
           lcd.clear();
           opState = RUN_PID;
           return;
       }
      }
      // =============================
      //   11.8.1.3 Set temperature base
      // =============================
      void c_mbase(){
        lcd.setCursor(1,0);
        lcd.print("Tid:");
        if (stime >= 100){
          lcd.setCursor(6,0);
        }
        if(stime <= 99){
          lcd.setCursor(7,0);
        }
        if(stime < 10){
          lcd.setCursor(8,0);
        }
        lcd.print((byte)stime,DEC);
        lcd.setCursor(9,0);
        lcd.print(" sek");
        lcd.setCursor(1,1);
        lcd.print("T:");
        if (setPointT >= 100){
          lcd.setCursor(3,1);
        }
        if(setPointT <= 99){
          lcd.setCursor(4,1);
        }
        if(setPointT < 10){
          lcd.setCursor(5,1);
        }
        lcd.print((byte)setPointT,DEC);
        lcd.setCursor(6,1);
        lcd.write(byte(0));
        lcd.setCursor(8,1);
        lcd.print(">B:");
        if (setPointB >= 100){
          lcd.setCursor(11,1);
        }
        if(setPointB <= 99){
          lcd.setCursor(12,1);
        }
        if(setPointB < 10){
          lcd.setCursor(13,1);
        }
        lcd.print((byte)setPointB,DEC);
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
           lcd.clear();
           opState = RUN_PID;
           return;
       }
      }
    // =============================
    //   11.8.2 Program 1
    // =============================
    void prog1() {
      progrm(p1time,p1tempT,p1tempB);
      stime = p1time;
      setPointT = p1tempT;
      setPointB = p1tempB;
    }
    // =============================
    //   11.8.3 Program 2
    // =============================
    void prog2() {
      progrm(p2time,p2tempT,p2tempB);
      stime = p2time;
      setPointT = p2tempT;
      setPointB = p2tempB;
    }
    // =============================
    //   11.8.4 Program 3
    // =============================
    void prog3() {
      progrm(p3time,p3tempT,p3tempB);
      stime = p3time;
      setPointT = p3tempT;
      setPointB = p3tempB;
    }
 // =============================


 // 11.9 Shut down PID
 // =============================
 void turn_off(){

    lcd.setCursor(0,0);
    lcd.print("Skru av varme?");
    lcd.setCursor(0,1);
    lcd.print("Ja: Trykk 'set'");

      if ((millis() - buttonPushTime) > 5000) {  // return to RUN after 5 seconds idle
          PIDT.SetTunings(KpT,KiT,KdT);
          PIDB.SetTunings(KpB,KiB,KdB);
          lcd.clear();
          opState = RUN_PID;
          return;
      }
 }
 // =============================
 // 11.10 Save to EEPROM
 // =============================

 void save_to_eeprom(){
   SaveParameters();
   PIDT.SetTunings(KpT,KiT,KdT);
   PIDB.SetTunings(KpB,KiB,KdB);

   lcd.clear();
   delay(200);
   lcd.setCursor(0,0);
   lcd.print("Verdier lagret");
   lcd.setCursor(0,1);
   lcd.print("til EEPROM");
   delay(500);

 blinkNo = 0;
   while(blinkNo < 10){
     lcd.clear(); // Av
     delay(50);
   lcd.print("Verdier lagret");// P?
   lcd.setCursor(0,1);
   lcd.print("til EEPROM");
     delay(50);
     blinkNo++;
       }
   delay(1000);
   lcd.clear();
   opState = RUN_PID;
   run_pid();
 }
// =============================
// 12 Temperature reading
// =============================

void get_temp(){
tmpcT = thermocoupleT.readCelsius();
tmpcB = thermocoupleB.readCelsius();
tempAkumT = tempAkumT + tmpcT;
tempAkumB = tempAkumB + tmpcB;
readNo = readNo + 1;
 if (readNo >= 7){
   tempAvgPrevT = tempAvgT;
   tempAvgT = ( tempAkumT/readNo);
   tempAvgT = tempAvgT + tKT;

   tempAvgPrevB = tempAvgB;
   tempAvgB = ( tempAkumB/readNo);
   tempAvgB = tempAvgB + tKB;

   tempAkumT = 0;
   tempAkumB = 0;
   readNo = 0;
 }
}

// =============================
// 13 Reading buttons
// =============================
void check_switches() {
	static byte previousstate[NUMBUTTONS];
	static byte currentstate[NUMBUTTONS];
	static long lasttime;
	byte index;

	if (millis() < lasttime) {
		// we wrapped around, lets just try again
		lasttime = millis();
	}

	if ((lasttime + DEBOUNCE) > millis()) {
		// not enough time has passed to debounce
		return;
	}

	// ok we have waited DEBOUNCE milliseconds, lets reset the timer
	lasttime = millis();

	for (index = 0; index < NUMBUTTONS; index++) {
		currentstate[index] = digitalRead(buttons[index]);   // read the button
		if (currentstate[index] == previousstate[index]) {
			if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
				// just pressed
				justpressed[index] = 1;
			}
			else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)){
				// just released
				justreleased[index] = 1;
			}
			pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
		}
		previousstate[index] = currentstate[index];   // keep a running tally of the buttons
	}
} // CHECK_SWITCHES SLUTT

  // =============================
  // 13.1 Button values
  // =============================
    // =============================
    // 13.1.1 Set
    // =============================
    void setValue() {
      //Main menu
      switch (opState) {
      case STARTUP:
        opState = TURN_ON_PID;
        break;
      case TURN_ON_PID:
        opState = RUN_PID;
        break;
      case RUN_PID:
        opState = SETPOINT1;
        break;
      case MANUALS:
        lcd.clear();
        opState = C_MTIME;
        break;
      case PROGRAM:
        opState = START_PROG1;
        break;
      case CALIBRATE:
        opState = C_PT;
        break;
      case TURN_OFF:
      lcd.clear();
        opState = STARTUP;
        break;
      // Manual menu
      case C_MTIME:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      case C_MTOP:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      case C_MBASE:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      // Program menu
      case START_PROG1:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      case START_PROG2:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      case START_PROG3:
        opState = RUN_PID;
        ticktime3 = 0;
        save_to_eeprom();
        break;
      // Calibrate menu
      case C_PT:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_IT:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_DT:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_PB:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_IB:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_DB:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_TMPT:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case C_TMPB:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      // Setpoint menu
      case SETPOINT1:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case SETPOINT2:
        opState = RUN_PID;
        save_to_eeprom();
        break;
      case SOLENOID:
        digitalWrite(solenoidPin, LOW);
        digitalWrite(solenoidled, LOW);
        solenoidState = 0;
        lcd.clear();
        opState = RUN_PID;
        break;
      }
    }
    // =============================
    // 13.1.2 Down
    // =============================
    void changeValueDown() {
      switch (opState) {
      // Main menu
      case STARTUP:
        opState = TURN_ON_PID;
        break;
      case TURN_ON_PID:
        opState = RUN_PID;
        break;
      case RUN_PID:
        break;
      case MANUALS:
        break;
      case PROGRAM:
        opState = START_PROG1;
        break;
      case CALIBRATE:
        break;
      case TURN_OFF:
        break;
      // Manual menu
      case C_MTIME:
        stime = stime-1;
        break;
      case C_MTOP:
        setPointT = setPointT-1;
        break;
      case C_MBASE:
        setPointB = setPointB-1;
        break;
      // Program menu
      case START_PROG1:
        opState = START_PROG2;
        break;
      case START_PROG2:
        opState = START_PROG3;
        break;
      case START_PROG3:
        opState = PROGRAM;
        break;
      // Calibrate menu
      case C_PT:
         KpT = KpT-1;
           if (KpT < 0){
           KpT = 0;
           }
        break;
      case C_IT:
         KiT = KiT-1;
           if (KiT < 0){
           KiT = 0;
           }
        break;
      case C_DT:
         KdT = KdT-1;
           if (KdT < 0){
           KdT = 0;
           }
        break;
      case C_PB:
         KpB = KpB-1;
           if (KpB < 0){
           KpB = 0;
           }
        break;
      case C_IB:
         KiB = KiB-1;
           if (KiB < 0){
           KiB = 0;
           }
        break;
      case C_DB:
         KdB = KdB-1;
           if (KdB < 0){
           KdB = 0;
           }
        break;
      case C_TMPT:
         tKT = tKT-1;
           if (tKT < -9){
           tKT = -9;
           }
        break;
      case C_TMPB:
         tKB = tKB-1;
           if (tKB < -9){
           tKB = -9;
           }
        break;
      // Setpoint menu
      case SETPOINT1:
        setPointT = setPointT-1;
          if (setPointT < 0){
            setPointT = 0;
          }
        break;
      case SETPOINT2:
      setPointB = setPointB-1;
          if (setPointB < 0){
            setPointB = 0;
          }
        break;
        case SOLENOID:
        break;
      }
    }
    // =============================
    // 13.1.3 Up
    // =============================
    void changeValueUp() {
      switch (opState) {
      // Main menu
      case STARTUP:
        opState = TURN_ON_PID;
        break;
      case TURN_ON_PID:
        opState = RUN_PID;
        break;
      case RUN_PID:
        break;
      case MANUALS:
        break;
      case PROGRAM:
        opState = START_PROG3;
        break;
      case CALIBRATE:
        break;
      case TURN_OFF:
        break;
      // Manual menu
      case C_MTIME:
        stime = stime+1;
        break;
      case C_MTOP:
        setPointT = setPointT +1;
        break;
      case C_MBASE:
        setPointB = setPointB +1;
        break;
      // Program menu
      case START_PROG1:
        opState = PROGRAM;
        break;
      case START_PROG2:
        opState = START_PROG1;
        break;
      case START_PROG3:
        opState = START_PROG2;
        break;
      // Calibrate menu
      case C_PT:
         KpT = KpT+1;
           if (KpT > 900){
           KpT = 900;
           }
        break;
      case C_IT:
         KiT = KiT+1;
           if (KiT > 99){
           KiT = 99;
           }
        break;
      case C_DT:
         KdT = KdT+1;
           if (KdT > 99){
           KdT = 99;
           }
        break;
      case C_PB:
         KpB = KpB+1;
           if (KpB > 900){
           KpB = 900;
           }
        break;
      case C_IB:
         KiB = KiB+1;
           if (KiB > 99){
           KiB = 99;
           }
        break;
      case C_DB:
         KdB = KdB+1;
           if (KdB > 99){
           KdB = 99;
           }
        break;
      case C_TMPT:
         tKT = tKT+1;
           if (tKT > 9){
           tKT = 9;
           }
        break;
      case C_TMPB:
         tKB = tKB+1;
           if (tKB > 9){
           tKB = 9;
           }
        break;
      // Setpoint menu
      case SETPOINT1:
        setPointT = setPointT+1;
          if (setPointT > 205){
            setPointT = 205;
          }
        break;
      case SETPOINT2:
      setPointB = setPointB+1;
          if (setPointB > 205){
            setPointB = 205;
          }
        break;
        case SOLENOID:
        break;
      }
    }
    // =============================
    // 13.1.4 Right
    // =============================
    void changeScreenRight() {
      switch (opState) {
      // Main menu
      case STARTUP:
        opState = TURN_ON_PID;
        break;
      case TURN_ON_PID:
        opState = RUN_PID;
        break;
      case RUN_PID:
        opState = MANUALS;
        break;
      case MANUALS:
        opState = PROGRAM;
        break;
      case PROGRAM:
        opState = CALIBRATE;
        break;
      case CALIBRATE:
        opState = TURN_OFF;
        break;
      case TURN_OFF:
        opState = RUN_PID;
        break;
      // Manual menu
      case C_MTIME:
        opState = C_MTOP;
        break;
      case C_MTOP:
        opState = C_MBASE;
        break;
      case C_MBASE:
        opState = C_MTIME;
        break;
        // Program menu
      case START_PROG1:
        opState = CALIBRATE;
        break;
      case START_PROG2:
        opState = CALIBRATE;
        break;
      case START_PROG3:
        opState = CALIBRATE;
        break;
      // Calibrate menu
      case C_PT:
        opState = C_IT;
        break;
      case C_IT:
        opState = C_DT;
        break;
      case C_DT:
        opState = C_TMPT;
        break;
      case C_TMPT:
        opState = C_PB;
        break;
      case C_PB:
        opState = C_IB;
        break;
      case C_IB:
        opState = C_DB;
        break;
      case C_DB:
        opState = C_TMPB;
        break;
      case C_TMPB:
        opState = C_PT;
        break;
      // Setpoint menu
      case SETPOINT1:
        opState = SETPOINT2;
        break;
      case SETPOINT2:
        opState = SETPOINT1;
        break;
        case SOLENOID:
        break;
      }
    }
    // =============================
    // 13.1.5 Left
    // =============================
    void changeScreenLeft() {
      switch (opState) {
      // Main menu
      case STARTUP:
        opState = TURN_ON_PID;
        break;
      case TURN_ON_PID:
        opState = RUN_PID;
        break;
      case RUN_PID:
        opState = TURN_OFF;
        break;
      case TURN_OFF:
        opState = CALIBRATE;
        break;
      case CALIBRATE:
        opState = PROGRAM;
        break;
      case PROGRAM:
        opState = MANUALS;
        break;
      case MANUALS:
        opState = RUN_PID;
        break;
      // Manual menu
      case C_MTIME:
        opState = C_MBASE;
        break;
      case C_MTOP:
        opState = C_MTIME;
        break;
      case C_MBASE:
        opState = C_MTOP;
        break;
      // Program menu
      case START_PROG1:
        opState = MANUALS;
        break;
      case START_PROG2:
        opState = MANUALS;
        break;
      case START_PROG3:
        opState = MANUALS;
        break;
      // Calibrate menu
      case C_PT:
        opState = C_TMPB;
        break;
      case C_IT:
        opState = C_PT;
        break;
      case C_DT:
        opState = C_IT;
        break;
      case C_TMPT:
        opState = C_DT;
        break;
      case C_PB:
        opState = C_TMPT;
        break;
      case C_IB:
        opState = C_PB;
        break;
      case C_DB:
        opState = C_IB;
        break;
      case C_TMPB:
        opState = C_DB;
        break;
      // Setpoint menu
      case SETPOINT1:
        opState = SETPOINT2;
        break;
      case SETPOINT2:
        opState = SETPOINT1;
        break;
        case SOLENOID:
        break;
      }
    }
    //=============================
    // Solenoid
    //=============================
    void solenoid(){
      if (stime-1 >= ticktime3){
      digitalWrite(solenoidPin, HIGH);
      digitalWrite(solenoidled, HIGH);
      solenoidState = 1;
      lcd.setCursor(0,0);
      lcd.print("Sublimerer ");
      if (stime - ticktime3 < 100){
        lcd.print(" ");
      }
      if (stime - ticktime3 < 10){
        lcd.print("  ");
      }
      lcd.print(stime - ticktime3);
      lcd.setCursor(0,1);
      lcd.print("T: ");
      if (tempAvgT != tempAvgPrevT) {
       lcd.setCursor(3,1);
       if (tempAvgT < 100){
         lcd.print(" ");
       }
       lcd.print((byte)tempAvgT);
       lcd.write(byte(0));
     }
    lcd.setCursor(7,1);
    lcd.print(" B: ");
     if (tempAvgB != tempAvgPrevB) {
       lcd.setCursor(11,1);
       if (tempAvgB < 100){
         lcd.print("  ");
       }
       lcd.print((byte)tempAvgB);
       lcd.write(byte(0));
     }
       run_pid_compute();


      }
      else{
        digitalWrite(solenoidPin, LOW);
        digitalWrite(solenoidled, LOW);
        solenoidState = 0;
        lcd.clear();
        opState = STARTUP;

      }

    }
// =============================
//14 EEPROM
// =============================

 // =============================
 // 14.1 Saving
 // =============================
 void SaveParameters()
 {
   // Top
    if (setPointT != EEPROM_readDouble(spAddresst))
    {
       EEPROM_writeDouble(spAddresst, setPointT);
    }
    if (tKT != EEPROM_readDouble(tkAddresst))
    {
       EEPROM_writeDouble(tkAddresst, tKT);
    }
    if (KpT != EEPROM_readDouble(kpAddresst))
    {
       EEPROM_writeDouble(kpAddresst, KpT);
    }
    if (KiT != EEPROM_readDouble(kiAddresst))
    {
       EEPROM_writeDouble(kiAddresst, KiT);
    }
    if (KdT != EEPROM_readDouble(kdAddresst))
    {
       EEPROM_writeDouble(kdAddresst, KdT);
    }
    // Base
    if (setPointB != EEPROM_readDouble(spAddressb))
    {
       EEPROM_writeDouble(spAddressb, setPointB);
    }
    if (tKB != EEPROM_readDouble(tkAddressb))
    {
       EEPROM_writeDouble(tkAddressb, tKB);
    }
    if (KpB != EEPROM_readDouble(kpAddressb))
    {
       EEPROM_writeDouble(kpAddressb, KpB);
    }
    if (KiB != EEPROM_readDouble(kiAddressb))
    {
       EEPROM_writeDouble(kiAddressb, KiB);
    }
    if (KdB != EEPROM_readDouble(kdAddressb))
    {
       EEPROM_writeDouble(kdAddressb, KdB);
    }
 }
 // =============================
 // 14.2 Loading
 // =============================
 void LoadParameters()
 {
   // Load from EEPROM
   // Top
    setPointT = EEPROM_readDouble(spAddresst);
    tKT = EEPROM_readDouble(tkAddresst);
    KpT = EEPROM_readDouble(kpAddresst);
    KiT = EEPROM_readDouble(kiAddresst);
    KdT = EEPROM_readDouble(kdAddresst);
    // Base
    setPointB = EEPROM_readDouble(spAddressb);
    tKB = EEPROM_readDouble(tkAddressb); // Kalibrering av temperatur
    KpB = EEPROM_readDouble(kpAddressb);
    KiB = EEPROM_readDouble(kiAddressb);
    KdB = EEPROM_readDouble(kdAddressb);
    // Use defaults if EEPROM values are invalid
    if (isnan(setPointT))
    {
      setPointT = 150; // 60
    }
    if (isnan(setPointB)){
      setPointB = 152;
    }
    if (isnan(KpT))
    {
      KpT = 200; // 850
    }
    if (isnan(KiT))
    {
      KiT = 50; // 0.5
    }
    if (isnan(KdT))
    {
      KdT = 25; // 0.1
    }
    if (isnan(tKT))
    {
      tKT = 0; // 0
    }
    if (isnan(KpB))
    {
      KpB = 200; // 850
    }
    if (isnan(KiB))
    {
      KiB = 50; // 0.5
    }
    if (isnan(KdB))
    {
      KdB = 25; // 0.1
    }
    if (isnan(tKB))
    {
      tKB = 0; // 0
    }
    if (isnan(stime)){
      stime = 60;
    }
 }
 // =============================
 // 14.3 Floating
 // =============================

    // =============================
    // 14.3.1 Writing
    // =============================
    void EEPROM_writeDouble(int address, double value)
    {
       byte* p = (byte*)(void*)&value;
       for (int i = 0; i < sizeof(value); i++)
       {
          EEPROM.write(address++, *p++);
       }
    }
    // =============================
    // 14.3.2 Reading
    // =============================
    double EEPROM_readDouble(int address)
    {
       double value = 0.0;
       byte* p = (byte*)(void*)&value;
       for (int i = 0; i < sizeof(value); i++)
       {
          *p++ = EEPROM.read(address++);
       }
       return value;
    }

// =============================
// 15 Datalog
// =============================
    void logData()
    {
      char buffer[5];
      Serial.print("#S|LOGTEMP|[");
//Top
      Serial.print(",");
      Serial.print(itoa((tempAvgT), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((setPointT), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((relayStateT), buffer, 10));
      Serial.print(",");

        Serial.print(itoa((KpT), buffer, 10));
        Serial.print(",");
        Serial.print(itoa((KiT), buffer, 10));
        Serial.print(",");
        Serial.print(itoa((KdT), buffer, 10));

      Serial.print(",");
      Serial.print(itoa((InputT), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((OutputT), buffer, 10));
      Serial.print(",");
// Base
      Serial.print(itoa((tempAvgB), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((setPointB), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((relayStateB), buffer, 10));
      Serial.print(",");

        Serial.print(itoa((KpB), buffer, 10));
        Serial.print(",");
        Serial.print(itoa((KiB), buffer, 10));
        Serial.print(",");
        Serial.print(itoa((KdB), buffer, 10));

      Serial.print(",");
      Serial.print(itoa((InputB), buffer, 10));
      Serial.print(",");
      Serial.print(itoa((OutputB), buffer, 10));
      Serial.print(",");
// Solenoid
      Serial.print(itoa((solenoidState),buffer,10));
      Serial.println("]#");
    }
