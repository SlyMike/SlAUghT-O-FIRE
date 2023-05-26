/*******************************************************************************

   SlAUghT-O-FIRE

   Autofire for lightgun/mouse/gamepad/joystick.
   Mick Dalton
   26/06/2019

  4 inputs: trigger, reload, Aux, mode
  2 outputs: TriggerOutPin, ReloadOutPin, AuxOutPin

  I2C OLED SCREEN
  SDA A4
  SCL A5
  0X3C


       PHYSICAL PINS

       INPUTS

  2    AUX       CONNECT TRIGGER BUTTON BETWEEN vcc AND D2
  3    TRIGGER   CONNECT TRIGGER BUTTON BETWEEN vcc AND D3
  4    RELOAD    CONNECT RELOAD BUTTON BETWEEN vcc AND D4
  5    MODE      CONNECT MODE BUTTON BETWEEN vcc AND D5

       OUTPUTS

  6    TRIGGER   CONNECT TO CONTROL BOARD (LOGIC HIGH = FIRE)
  7    RELOAD    CONNECT TO CONTROL BOARD (LOGIC HIGH = RELOAD)
  8    AUX       CONNECT TO CONTROL BOARD (LOGIC HIGH = AUX)

    --------------------------------------------------------
    -- USE NPN TRANSISTOR ON OUTPUT PINS TO INVERT LOGIC, --
    -- THIS WILL SHORT CONTROLLER PCB INPUTS TO GND.      --
    -- USING THE OUTPUTS_LOW OPTION WILL NOT PROTECT A    --
    -- LOW POWER SYSTEM WHEN USED WITH THE WRONG ARDUINO. --
    --------------------------------------------------------

    --------------------------------------------------------
    --       OLED                           DONE          --
    --       MANUAL MODE                    DONE          --
    --       BURST MODE                     DONE          --
    --       AUTOFIRE                       DONE          --
    --       AUTO RELOAD                    DONE          --
    --       SPEED MENU                     DONE          --
    --       BURST MENU                     DONE          --
    --       OUTPUT DISABLE(MENUS)          DONE          --
    --       FORMATTING                     DONE          --
    --       EEPROM                         DONE          --
    --       ADD MORE SPEED OPTIONS         DONE          --
    --       INVERTED MENU TITLES           DONE          --
    --       AUX BUTTON PASSTHROUGH         DONE          --
    --       STATRUP TEXT                   DONE          --
    --       ANTI CHEAT DETECTION           DONE          --
    --       DECOUPLE INPUTS                DONE          --
    --       ADD OUTPUT INVERSION           DONE          --
    --       CODE OPTIMIZATION              TODO          --
    --       TIDY MENU HEADER GFX           TODO          --
    --       ADD MORE I/O OPTIONS           TODO          --
    --       OTF SELECT I/O FOR AF          TODO          --
    --       SEPERATE FIRE MODES AND MENUS  TODO          --
    --       ADD SETTINGS BUTTON            TODO          --
    --------------------------------------------------------

  Modes: passthrough, burst, auto, setup.
  Options: auto reload.
  Extras: OLED Screen

  V1.6.2
  OLED                   DONE
  MANUAL MODE            DONE
  BURST MODE             DONE
  AUTOFIRE               DONE
  AUTO RELOAD            DONE
  SPEED MENU             DONE
  BURST MENU             DONE
  OUTPUT DISABLE(MENUS)  DONE
  FORMATTING             DONE
  EEPROM                 DONE
  ADD MORE SPEED OPTIONS DONE
  INCREASED ROUNDS TO X12
  ADDED - SINGLE SHOT MODE (SELECT 1 ROUND IN BURST MENU)

  V1.7
  ALL SHOOTING AND RELOADING IS HANDLED BY FUNCTIONS
  DECOUPLE I/Os          TODO
  ADD I/O INVERSION OPT  TODO
  CODE OPTIMISATION      TODO
  r1 CHANGED TO AN int IN ORDER TO STORE LONGER DURATION PAUSE

  V1.7.1
  INVERTED MENU TITLES

  V1.7.2
  Added aux button to program in passthrough mode only - for future use.

  V1.7.3
  ADDED NEW STARTUP TEXT ON OLED, UPDATED DOCUMENT HEADER

  V1.8
  UPDATED TRIGGER AUTOFIRE SPEEDS
   VSLOW  = 2CPS
   SLOW   = 4CPS
   MED    = 8CPS
   FAST   = 12CPS
   VFAST  = 16CPS

  OLED TWEEK

  UPDATE RELOAD SPEEDS
   VSLOW  = 3 SECONDS
   SLOW   = 2 SECONDS
   MED    = 1 SECOND
   FAST   = 0.3 SECONDS
   VFAST  = 0.1 SECONDS

  ENABLE SERIAL DEBUG VIA D9 TO GND
  ANTI-CHEAT MODE RANDOM NUMBER GENERATOR ac=1 TO ACTIVATE

  TODO - FIX EEPROM ROUTINE (EEPROM.update(12, r1);) DOES NOT WORK FOR STORING AND RETREVING INT VALUE ROR r1.

  v1.8.1

  anti cheet menu placeholder

  v1.8.1.1

  something went wrong - no oled  == wiring issue

  V1.82
  DONE - anticheat menus and oled routine
  Small code tidy
  TODO list remains!
  TODO remove "ac" from anticheat prototype, replace with "acVal", clean up EEPROM.
  NOTE - menu cycle required to store ARL value - fix as per V1.8
  TODO - fix inverted menu headers - add boxes

  V1.8.3
  r1 is now back to a byte with "rmf" implimented at EEPROM 13

  V1.9
  Anti cheet detection implemented, 4 modes -- off, mild, normal, aggressive.
  DONE - all EEPROM related variables implemented. -- ARL, uses two bytes.. value + multiplier.
  TODO LIST @ TOP OF TEXT.
  MUST LOOK INTO EEPROM ROUTINE, TIDY, ORGANISE, CHECK ALL.

  V1.9.1
  Reworked shoot() / s1, s2 and anti cheet jitter are seperated to prevent overflow in EEPROM space.
  shoot() requires 8 arguments to function.
  TODO acj only update figures whilst in menu... not during firing cycles!!
  ac works in all modes

  v1.9.12

  tidied comments, added first if to main - SINCE REMOVED v2.01

  V2.0
  Added option to invert logic on trigger with LOGIC_HIGH / LOW CTMs
  Only inverts logic on trigger ATM, inputs logic not implemented.
  Created "TriggerLogic", "ModeLogic", "ReloadLogic" & "AuxLogic" prototypes

  v2.01
  added reload and aux to invert list
  This line:   if (ModeReading == HIGH && TriggerReading == LOW) has to go for inverted logic to work#! -- WRONG!
  ERROR! mode3 lockup for 20 seconds on trigger press! WHY?? EDIT, fine now required menu cycle and AC on, off cycle???
  FIXED removed if statment preventing acj from aquiring a value on first pass through the loop!

  V2.1
  DONE - invert input logic option.

  v2.2
  DONE - invert output logic option.....nearly, !AUX
  
*******************************************************************************/
//OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include <EEPROM.h>
//OLED
#define AUTHOR "MICK DALTON"              /* Version info. */
#define INFO_MAJOR_VERSION "2"            /* Version info. */
#define INFO_MINOR_VERSION "2"           /* Version info. */
#define COMPILE_TIME __TIME__             /* Version info. */
#define COMPILE_DATE __DATE__             /* Version info. */
#define SOURCE_FILE __FILE__              /* Version info. */
#define COMPILER __VERSION__              /* Version info. */

//------------------------------COMMENT THE FOLLOWING OPTIONS AS APPROPRIATE------------------------------------

//--------------------------------------------------------------------------------------------------------------
//#define INDEBUG                         // Serial debugging - COMMENT TO DISABLE
//--------------------------------------------------------------------------------------------------------------
#define INPUTS_LOW                        // Inputs pulled low to activate - "INPUT_PULLUP" will be used and switches will short to GND, default for most systems where a switch to GND is closed to activate inputs.
//#define INPUTS_HIGH                     // Inputs pulled high to activate - Digital inputs will require external 10K resistor to GND - Switches will short to VCC.
//--------------------------------------------------------------------------------------------------------------
//#define OUTPUTS_LOW
#define OUTPUTS_HIGH                      //can be used with NPN transistor to implement open collector circuit - safest way to invert logic
//--------------------------------------------------------------------------------------------------------------

//All pin assignments are intended for Atmega 328P

const int TriggerInPin = 3;               //trigger on lightgun
const int ReloadInPin = 4;                //RHS aux button
const int AuxInPin = 2;                   //LHS aux button
const int ModePin = 5;                    //Selects menu positions

const int TriggerOutPin = 6;              //input on lightgun PCB for trigger
const int ReloadOutPin = 7;               //input on lightgun PCB RHS aux
const int AuxOutPin = 8;                  //input on lightgun PCB LHS aux

byte Mode = 1;                            //modes selected by mode button

byte burstrunxtimes = 1;                  //counter value to disable burst mode running continuously like autofire function
byte oledupdatetimes = 1;                 //counter to prevent OLED being continuously updated


//The following values will be overwritten by EEPROM or menu selections, don't edit them here: s1, s2, r1, rmf, acH, acL.

byte s1 = 100;                            //shoot trigger down duration MAX 255
byte s2 = 50;                             //shoot interval
byte r1 = 50;                             //reload duration
byte rmf = 1;                             //reload multiplacation factor

byte Modem1t = 3;                         // default slow, trigger speed menu 1
byte Modem1r = 3;                         // default slow, reload speed menu 1

byte tspeed = 3;                          //display above on oled
byte rspeed = 3;                          //display above on oled

byte Modem2r = 1;                         //mode menu 2 rounds
byte Modem2a = 1;                         //mode menu 2 ARL

byte rval = 2;                            //display above on oled
byte arlval = 0;                          //display above on oled

//loop timer
long lastMillis = 0;                      //used in serial debug
long loops = 0;
//loop timer

//AntiCheat
long randNumber;                          //holds a random number for anti cheet detection function
byte ac = 0;                              //enable anti cheating function with 1 use "AntiCheat(ac, acH, acL)"
byte acH = 25;                            //highest random number (+)
byte acL = 25;                            //lowest random number (-)
byte Modem3ac = 1;                        //mode 3 menu anti cheat protection
byte acVal = 0;                           //OLED menu value
//AntiCheat


//button logic - start
byte trigtmp;                             //temp trigger value - taken from reading digital pin - passed to prototype
byte TriggerReading = 0;                  //returned from prototype and passed on to "shoot"
byte rltmp;
byte ReloadReading = 0;
byte autmp;
byte AuxReading = 0;                      //Temp value for aux pin
byte motmp;
byte ModeReading = 0;
//button logic - end

void setup() {

  //AntiCheat
  randomSeed(analogRead(0));
  //AntiCheat

  //OLED
  // initialize with the I2C addr 0x3C (for the 128x32)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(5, 10);
  display.print("SlAUghT-O-FIRE V");
  display.print(INFO_MAJOR_VERSION);
  display.print( ".");
  display.println(INFO_MINOR_VERSION);
  display.setCursor(5, 24);
  display.println("SLY-TECH, 2019");
  display.display();
  //OLED

  //INPUTS
#ifdef INPUTS_HIGH
  pinMode(TriggerInPin, INPUT);
  pinMode(ReloadInPin, INPUT);
  pinMode(AuxInPin, INPUT);
  pinMode(ModePin, INPUT);
#endif


#ifdef INPUTS_LOW
  pinMode(TriggerInPin, INPUT_PULLUP);
  pinMode(ReloadInPin, INPUT_PULLUP);
  pinMode(AuxInPin, INPUT_PULLUP);
  pinMode(ModePin, INPUT_PULLUP);
#endif
  //INPUTS

  //OUTPUTS
  pinMode(TriggerOutPin, OUTPUT);
  pinMode(ReloadOutPin, OUTPUT);
  pinMode(AuxOutPin, OUTPUT);
  //OUTPUTS

  //SERIAL DEBUG
#ifdef INDEBUG
  Serial.begin(115200);
  delay(100);
  Serial.print("SlAUghT-O-FIRE");

  Serial.println(" ");
  Serial.print(AUTHOR);
  Serial.print(", V");
  Serial.print(INFO_MAJOR_VERSION);
  Serial.print(".");
  Serial.println(INFO_MINOR_VERSION);

  Serial.print(COMPILE_DATE);
  Serial.print(" - ");
  Serial.println(COMPILE_TIME);
  Serial.println(SOURCE_FILE);
  Serial.print("Compiled with = ");
  Serial.println(COMPILER);
#endif
  //SERIAL DEBUG

  delay(2000);

  //EEPROM
  //SAVES MENU POSITIONS

  //MODE(0)
  if (EEPROM.read(0) > 0)
  {
    Mode = EEPROM.read(0);
  }

  //AUTOFIRE SPEED(1)
  if (EEPROM.read(1) > 0)
  {
    Modem1t = EEPROM.read(1);
  }

  //RELOAD SPEED(2)
  if (EEPROM.read(2) > 0)
  {
    Modem1r = EEPROM.read(2);
  }

  //ROUNDS(3)
  if (EEPROM.read(3) > 0)
  {
    Modem2r = EEPROM.read(3);
  }

  //RELOAD SPEED(4)
  if (EEPROM.read(4) > 0)
  {
    Modem2a = EEPROM.read(4);
  }

  //AUTOFIRE TRIGGER DOWN DELAY(5)
  if (EEPROM.read(5) > 0)
  {
    s1 = EEPROM.read(5);
  }

  //AUTOFIRE TRIGGER UP DELAY(6)
  if (EEPROM.read(6) > 0)
  {
    s2 = EEPROM.read(6);
  }

  //  anti cheat(7)
  if (EEPROM.read(7) > 0)
  {
    acVal = EEPROM.read(7);
  }

  //rounds value(8)
  if (EEPROM.read(8) > 0)
  {
    rval = EEPROM.read(8);
  }

  //ARL value(9)
  if (EEPROM.read(9) > 0)
  {
    arlval = EEPROM.read(9);
  }

  //trigger speed
  if (EEPROM.read(10) > 0)
  {
    tspeed = EEPROM.read(10);
  }

  //reload speed
  if (EEPROM.read(11) > 0)
  {
    rspeed = EEPROM.read(11);
  }

  // RELOAD DURATION
  if (EEPROM.read(12) > 0)
  {
    r1 = EEPROM.read(12);
  }

  // RELOAD MULTIPLIER VALUE
  if (EEPROM.read(13) > 0)
  {
    rmf = EEPROM.read(13);
  }

  // ANTI CHEAT
  if (EEPROM.read(14) > 0)
  {
    ac = EEPROM.read(14);
  }

  // ANTI CHEAT MENU POSITION
  if (EEPROM.read(12) > 0)
  {
    Modem3ac = EEPROM.read(15);
  }
  //EEPROM

  //OLED
  display.clearDisplay();
  delay(200);
  //OLED

  //INITIALIZE OUTPUTS
#ifdef OUTPUTS_HIGH
  digitalWrite(TriggerOutPin, LOW);
  digitalWrite(ReloadOutPin, LOW);
  digitalWrite(AuxOutPin, LOW);
#endif

#ifdef OUTPUTS_LOW
  digitalWrite(TriggerOutPin, HIGH);
  digitalWrite(ReloadOutPin, HIGH);
  digitalWrite(AuxOutPin, HIGH);
#endif
  //INITIALIZE OUTPUTS
}//SETUP END


void loop() {
  //ANTI CHEAT
  int acj;                                            // holds temp value returned from prototype (acj=anti cheet jitter)
  acj = (AntiCheat(ac, acH, acL));                    // updates value every time through the loop to keep numbers randon
  //ANTI CHEAT

  //INPUT HANDLING
  motmp = digitalRead(ModePin);
  ModeReading = ModeLogic(motmp);

  rltmp = digitalRead(ReloadInPin);
  ReloadReading = ReloadLogic(rltmp);                 // ReloadReading passed to "Reload" prototype

  trigtmp = digitalRead(TriggerInPin);
  TriggerReading = TriggerLogic(trigtmp);             // TriggerReading passed to "Shoot" prototype

  autmp = digitalRead(AuxInPin);
  AuxReading = AuxLogic(autmp);
  
  // !!!!!!!! the following line needs changing and passing to its own prototype similar to shoot or reload
  // consider changing aux to special as in fire, reload, special weapon / grenade....
  digitalWrite (AuxOutPin, AuxReading);               // passes signal from the (unused) auxpin so it can be used to swap button
  //INPUT HANDLING


  //FIRING MODES + MENUS
  // mode selection, tap mode button to cycle through modes
  if (ModeReading == HIGH && TriggerReading == LOW)
  {
    Mode ++;
    delay (100);                                        // this is why we didn't use a debounce routine
    if (Mode == 7)
    {
      Mode = 1;
    }
    oledupdatetimes = 1;                                // reset variable to allow oled to update on every mode button press
    EEPROM.update(0, Mode);                             // save current mode selection to EEPROM
  }

  // *****MODE 1 START*****
  // MANUAL MODE - BUTTON IN = BUTTON OUT
  if (Mode == 1)
  {
    if (oledupdatetimes)
    {
      oled(1);
      oledupdatetimes --;                               // disallow oled routine to run every time through the loop
    }
    Shoot(TriggerReading, 1, 1, 1, 0, 0, 0, 0);         // always requires 8 arguments (bytes)
    Reload(ReloadReading, 1, rmf, 0);
  }
  // *****MODE 1 END*****

  // *****MODE 2 START*****
  // BURST MODE fires X shots, AUTO/manual reload
  if (Mode == 2)
  {
    if (oledupdatetimes)
    {
      oled(2);
      oledupdatetimes --;
    }

    if (burstrunxtimes)
    {
      if (TriggerReading == 1)
      {
        burstrunxtimes--;
        Shoot(1, rval, s1, s2, acj, 1, 0, 0);          // always requires 8 arguments (bytes)
        if (arlval == 1)
        {
          Reload(1, r1, rmf, 1);
        }
      }
    }
    if (TriggerReading == 0)
    {
      burstrunxtimes = 1;
    }
    Reload(ReloadReading, 1, rmf, 0);
  }
  // *****MODE 2 END*****

  // *****MODE 3 START*****
  if (Mode == 3)                                      // AUTOFIRE
  {
    if (oledupdatetimes)
    {
      oled(3);
      oledupdatetimes --;
    }
    Shoot(TriggerReading, 1, s1, s2, acj, 1, 0, 0);   // always requires 8 arguments (bytes)
    Reload(ReloadReading, 1, rmf, 0);
  }
  // *****MODE 3 END*****

  // *****MODE 4 START*****
  if (Mode == 4)                                      // autofire speed, reload speed
  {
    //mode selection
    if (TriggerReading == HIGH)/////////////////////////////////////////autofire speed select//////////////////////////////////
    {
      Modem1t ++; //mode Menu 1 Trigger, 5 options, slow medium, fast
      delay (100);// this is why we didn't use a debounce routine
      if (Modem1t == 6)
      {
        Modem1t = 1;
      }
      oledupdatetimes = 1; // enable (menu title) screen refresh after every button press


      // SPEED IS DETERMINED BY VALUES s1 AND s2

    }
    if (Modem1t == 1)
    {
      s1 = 250;                                       // time allocated to trigger pressed action
      s2 = 250;                                       // time allocated to delay between trigger presses
      tspeed = 1;                                     // linked to oled display, vslow, slow, medium, fast, vfast
    }
    if (Modem1t == 2)
    {
      s1 = 150;
      s2 = 100;
      tspeed = 2;
    }
    if (Modem1t == 3)
    {
      s1 = 75;
      s2 = 50;
      tspeed = 3;
    }
    if (Modem1t == 4)
    {
      s1 = 50;
      s2 = 33;
      tspeed = 4;
    }
    if (Modem1t == 5)
    {
      s1 = 40;
      s2 = 26;
      tspeed = 5;
    }
    EEPROM.update(1, Modem1t);                      // update EEPROM values if they have been changed
    EEPROM.update(5, s1);
    EEPROM.update(6, s2);
    EEPROM.update(10, tspeed);
    //autofire speed end//////////////////////////////////////////////

    if (ReloadReading == HIGH)
    {
      Modem1r ++;                                   // mode Menu 1 Reload, 5 options= vslow, slow, medium, fast, vfast
      delay (100);                                  // this is why we didn't use a debounce routine
      if (Modem1r == 6)
      {
        Modem1r = 1;
      }
      oledupdatetimes = 1;                          // enable (menu title) screen refresh after every button press
    }

    if (Modem1r == 1)
    {
      r1 = 250;                                     // rl * rmf = reload duration - 3 SECONDS RELOAD TIME
      rmf = 12;                                     // MULTIPLIER FOR ABOVE
      rspeed = 1;                                   // linked to OLED vslow, slow..........etc.
    }
    if (Modem1r == 2)
    {
      r1 = 250;                                     // 2 SECONDS RELOAD TIME
      rmf = 8;
      rspeed = 2;
    }
    if (Modem1r == 3)
    {
      r1 = 250;                                     // 1 SECOND RELOAD TIME
      rmf = 4;
      rspeed = 3;
    }
    if (Modem1r == 4)
    {
      r1 = 100;                                     // 0.3 SECONDS RELOAD TIME
      rmf = 3;
      rspeed = 4;
    }
    if (Modem1r == 5)
    {
      r1 = 100;                                     // 0.1 SECONDS RELOAD TIME
      rmf = 1;
      rspeed = 5;
    }
    EEPROM.update(2, Modem1r);
    EEPROM.update(11, rspeed);
    EEPROM.update(12, r1);
    EEPROM.update(13, rmf);
    if (oledupdatetimes)
    {
      oled(4);
      oledupdatetimes --;
    }
  }                                                 // menu 1 end

  if (Mode == 5)                                    // rounds, auto reload settings
  {

    if (TriggerReading == HIGH)/////////////////////////////////////////no of rounds select//////////////////////////////////
    {
      Modem2r ++;                                   // mode Menu 1 Trigger, 7 options, 1-12 rounds
      delay (100);                                  // this is why we didn't use a debounce routine
      if (Modem2r == 13)
      {
        Modem2r = 1;
      }
      oledupdatetimes = 1;                          // enable (menu title) screen refresh after every button press
    }
    if (Modem2r == 1)
    {
      rval = 1;                                     // option 1 code -- JUMPS TO SINGLE SHOT MODE
    }
    if (Modem2r == 2)
    {
      rval = 2;                                     // how many rounds are loaded
    }
    if (Modem2r == 3)
    {
      rval = 3;
    }
    if (Modem2r == 4)
    {
      rval = 4;
    }
    if (Modem2r == 5)
    {
      rval = 5;
    }
    if (Modem2r == 6)
    {
      rval = 6;
    }
    if (Modem2r == 7)
    {
      rval = 7;
    }
    if (Modem2r == 8)
    {
      rval = 8;
    }
    if (Modem2r == 9)
    {
      rval = 9;
    }
    if (Modem2r == 10)
    {
      rval = 10;
    }
    if (Modem2r == 11)
    {
      rval = 11;
    }
    if (Modem2r == 12)
    {
      rval = 12;
    }
    EEPROM.update(3, Modem2r);
    EEPROM.update(8, rval);
    // no of rounds end//////////////////////////////////////////////

    if (ReloadReading == HIGH)///////////////arl///////////////////////
    {
      Modem2a ++;                                   // mode Menu 1 Reload, 3 options, slow medium, fast
      delay (100);                                  // this is why we didn't use a debounce routine
      if (Modem2a == 3)
      {
        Modem2a = 1;
      }
      oledupdatetimes = 1;                          // enable (menu title) screen refresh after every button press
    }

    if (Modem2a == 1)
    {
      arlval = 0;                                   // auto reload is OFF
    }
    if (Modem2a == 2)
    {
      arlval = 1;
    }
    EEPROM.update(4, Modem2a);
    EEPROM.update(9, arlval);
    if (oledupdatetimes)
    {
      oled(5);
      oledupdatetimes --;
    }
  }                                                 // menu 2 end


  // menu 3 start --- anti cheet enable menu
  if (Mode == 6)
  {

    //mode selection
    if (TriggerReading == HIGH)/////////////////////////////////////////anti cheet select//////////////////////////////////
    {
      Modem3ac ++;                                  // mode Menu 3 Trigger, 2 options, off, on
      delay (100);                                  // this is why we didn't use a debounce routine
      if (Modem3ac == 5)
      {
        Modem3ac = 1;
      }
      oledupdatetimes = 1;                          // enable (menu title) screen refresh after every button press
    }
    if (Modem3ac == 1)
    {
      acVal = 0;                                    // anti cheet function OFF
      ac = acVal;                                   // update variable
    }
    if (Modem3ac == 2)
    {
      acVal = 1;                                    // anti cheet function - mild setting
      ac = acVal;
    }
    if (Modem3ac == 3)
    {
      acVal = 2;                                    // anti cheet function - normal setting
      ac = acVal;
    }
    if (Modem3ac == 4)
    {
      acVal = 3;                                    // anti cheet function - aggressive setting
      ac = acVal;
    }

    if (oledupdatetimes)
    {
      oled(6);
      oledupdatetimes --;
    }
    EEPROM.update(7, acVal);
    EEPROM.update(14, ac);
    EEPROM.update(15, Modem3ac);

  }  //menu 3 end
  //FIRING MODES + MENUS


  //SERIAL DEBUG
#ifdef INDEBUG
  /*
    // loop timer - start
    long currentMillis = millis();
    loops++;
    if (currentMillis - lastMillis > 1000) {
    Serial.println("");
    Serial.print("Loops last second:");
    Serial.print(loops);
    Serial.print(", Cycle time:");
    Serial.print(1000 / loops);
    Serial.println(" ms  ");
    lastMillis = currentMillis;
    loops = 0;
    }
    // loop timer - end
  */
  Serial.print ("TriggerReading ");
  Serial.print (TriggerReading);
  Serial.print ("  ReloadReading ");
  Serial.print (ReloadReading);
  Serial.print ("  AuxReading ");
  Serial.print (AuxReading);
  Serial.print ("  ModeReading ");
  Serial.print (ModeReading);
  Serial.print (" Mode ");
  Serial.print (Mode);
  Serial.print ("  s1 ");
  Serial.print (s1);
  Serial.print ("  s2 ");
  Serial.print (s2);
  Serial.print ("  ARL ");
  Serial.print (arlval);
  Serial.print ("  r1 ");
  Serial.print (r1);
  Serial.print ("  rmf ");
  Serial.print (rmf);
  Serial.print ("  acj ");
  Serial.print (acj);
  Serial.print ("  AntiCheat jitter ");
  Serial.println (AntiCheat(ac, acH, acL));
#endif
  //SERIAL DEBUG
}// void end

//OLED
void oled(int mode)
{
  if (mode == 0)
  {
    display.clearDisplay();
  }

  if (mode == 1)                                    // MANUAL
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.println("MANUAL MODE");
    display.display();
  }

  if (mode == 2)                                   // BURST
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);

    if (rval == 1)
    {
      display.println("SINGLE SHOT MODE");
    }
    else
    {
      display.println("BURST MODE");
    }
    display.setCursor(5, 24);
    display.println("ROUNDS=");
    display.setCursor(49, 24);
    display.println(rval);
    display.setCursor(71, 24);
    display.println("ARL=");
    display.setCursor(95, 24);
    if (arlval == 0)
    {
      display.println("OFF");
    }
    if (arlval == 1)
    {
      display.println("ON");
    }
    display.display();
  }

  if (mode == 3)//AUTOFIRE
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.println("AUTOFIRE MODE");
    display.display();
  }

  if (mode == 4)                                  // SPEED MENU
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK, WHITE);           // INVERTED MENU TITLE
    display.setCursor(5, 10);
    display.println("SPEED MENU");                // 1
    display.setCursor(5, 24);
    display.setTextColor(WHITE);                  // INVERTED MENU TITLE
    display.println("AF=");
    display.setCursor(23, 24);

    if (tspeed == 1)
    {
      display.println("VSLOW");
    }
    if (tspeed == 2)
    {
      display.println("SLOW");
    }
    if (tspeed == 3)
    {
      display.println("MED");
    }
    if (tspeed == 4)
    {
      display.println("FAST");
    }
    if (tspeed == 5)
    {
      display.println("VFAST");
    }

    display.setCursor(71, 24);
    display.println("ARL=");
    display.setCursor(95, 24);
    if (rspeed == 1)
    {
      display.println("VSLOW");
    }
    if (rspeed == 2)
    {
      display.println("SLOW");
    }
    if (rspeed == 3)
    {
      display.println("MED");
    }
    if (rspeed == 4)
    {
      display.println("FAST");
    }
    if (rspeed == 5)
    {
      display.println("VFAST");
    }
    display.display();
  }

  if (mode == 5)                                // BURST MENU
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK, WHITE);         // INVERTED MENU TITLE
    display.setCursor(5, 10);
    display.println("BURST MENU");
    display.setCursor(5, 24);
    display.setTextColor(WHITE);                // INVERTED MENU TITLE
    display.println("ROUNDS=");
    display.setCursor(49, 24);
    display.println(rval);
    display.setCursor(71, 24);
    display.println("ARL=");
    display.setCursor(95, 24);
    if (arlval == 0)
    {
      display.println("OFF");
    }

    if (arlval == 1)
    {
      display.println("ON");
    }

    display.display();
  }
  // menu 6 start  ---  anti cheaat
  if (mode == 6)                                // ANTI CHEET MENU
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK, WHITE);         // INVERTED MENU TITLE
    display.setCursor(5, 10);
    display.println("ANTI CHEAT MENU");
    display.setCursor(5, 24);
    display.setTextColor(WHITE);                // INVERTED MENU TITLE
    display.println("JITTER=");
    display.setCursor(49, 24);

    if (acVal == 0)
    {
      display.println("OFF");
    }

    if (acVal == 1)
    {
      display.println("MILD");
    }

    if (acVal == 2)
    {
      display.println("NORMAL");
    }

    if (acVal == 3)
    {
      display.println("AGGRESSIVE");
    }

    display.display();
  }
  //menu 6 end
}
//OLED

//RELOAD
void Reload(int rl, int d1, int mf, int ma)
//int rl  = reload button pressed
//int d1  = reload pressed duration
//int mf  = multiplication factor (keeps eeprom entries as bytes)
//int ma  = 0 - manual, 1 - automatic
{
  int i;
  if (rl > 0 && ma == 1)
  {
    for (i = 0; i < 1; i++) {
#ifdef OUTPUTS_HIGH
      digitalWrite(ReloadOutPin, HIGH);
      delay (d1 * mf);
      digitalWrite(ReloadOutPin, LOW);
#endif

#ifdef OUTPUTS_LOW
      digitalWrite(ReloadOutPin, LOW);
      delay (d1 * mf);
      digitalWrite(ReloadOutPin, HIGH);
#endif
    }
  }
  if (rl == 1 && ma == 0)
  {
#ifdef OUTPUTS_HIGH
    digitalWrite(ReloadOutPin, HIGH);
#endif

#ifdef OUTPUTS_LOW
    digitalWrite(ReloadOutPin, LOW);
#endif
  }
  if (rl == 0 && ma == 0)
  {
#ifdef OUTPUTS_HIGH
    digitalWrite(ReloadOutPin, LOW);
#endif

#ifdef OUTPUTS_LOW
    digitalWrite(ReloadOutPin, HIGH);
#endif
  }
}
//RELOAD

//SHOOT
void Shoot(byte sht, byte rds, byte d1, byte d2, int acj, byte ma, int y, int z)
//int sht = 0 - dont shoot, 1 - shoot
//int rds = loaded rounds
//int d1  = trigger pulled duration
//int d2  = delay between shots
//int acj = anti cheet jitter
//int ma  = 0 - manual, 1 - automatic
//int y   = place holder
//int z   = place holder

//Shoot(1,1,1,1,0,0,0,0) enters passthrough

{
  int i;
  if (sht >= 1 && rds > 1 && ma == 1)           // burst function
  {
    for (i = 0; i < rds; i++)
    {
#ifdef OUTPUTS_HIGH
      acj = (AntiCheat(ac, acH, acL));
      digitalWrite(TriggerOutPin, HIGH);
      delay (d1 + acj);
      acj = (AntiCheat(ac, acH, acL));
      digitalWrite(TriggerOutPin, LOW);
      delay (d2 + acj);
#endif

#ifdef OUTPUTS_LOW
      acj = (AntiCheat(ac, acH, acL));
      digitalWrite(TriggerOutPin, LOW);
      delay (d1 + acj);
      acj = (AntiCheat(ac, acH, acL));
      digitalWrite(TriggerOutPin, HIGH);
      delay (d2 + acj);
#endif
    }
  }

  if (sht >= 1 && rds == 1 && ma == 1)          // automatic function
  {
#ifdef OUTPUTS_HIGH
    digitalWrite(TriggerOutPin, HIGH);
    delay (d1 + acj);
    acj = (AntiCheat(ac, acH, acL));
    digitalWrite(TriggerOutPin, LOW);
    delay (d2 + acj);
#endif

#ifdef OUTPUTS_LOW
    digitalWrite(TriggerOutPin, LOW);
    delay (d1 + acj);
    acj = (AntiCheat(ac, acH, acL));
    digitalWrite(TriggerOutPin, HIGH);
    delay (d2 + acj);
#endif
  }
  if (sht == 1 && ma == 0)                      // manual passthrough
  {
#ifdef OUTPUTS_HIGH
    digitalWrite(TriggerOutPin, HIGH);
#endif

#ifdef OUTPUTS_LOW
    digitalWrite(TriggerOutPin, LOW);
#endif
  }
  if (sht == 0 && ma == 0)
  {
#ifdef OUTPUTS_HIGH
    digitalWrite(TriggerOutPin, LOW);
#endif

#ifdef OUTPUTS_LOW
    digitalWrite(TriggerOutPin, HIGH);
#endif
  }
}
//SHOOT

//ANTI CHEAT
int AntiCheat (byte a, byte h, byte l)
{
  //int a = 0=off, 1=mild, 2=normal, 3=aggressive
  //int h = high value
  //int l = low value

  int val;
  int rhigh = random(h);
  int rlow = random(l);

  if (a > 3)                                  // we only have 4 possible conditions for a
  {
    a = 3;
  }

  if (a >= 1)                                 // anti cheet on, take 2 random numbers * by a
  {
    val = (rhigh - rlow * a);
  }

  if (a == 0)                                 // anti cheet off - return 0
  {
    val = 0;
  }
  return val;                                 // return a value to the main loop
}
//ANTI CHEAT

//--------------------------------INPUT LOGIC INVERSION - START--------------------------------------

//TRIGGER
int TriggerLogic(byte tr)
{
  int t;
#ifdef INPUTS_HIGH
  if (tr == 1)
  {
    t = 1;
  }
  if (tr == 0)
  {
    t = 0;
  }
#endif

#ifdef INPUTS_LOW
  if (tr == 1)
  {
    t = 0;
  }
  if (tr == 0)
  {
    t = 1;
  }
#endif
  return t;
}
//TRIGGER

//RELOAD
int ReloadLogic(byte rl)
{
  int r;
#ifdef INPUTS_HIGH
  if (rl == 1)
  {
    r = 1;
  }
  if (rl == 0)
  {
    r = 0;
  }
#endif

#ifdef INPUTS_LOW
  if (rl == 1)
  {
    r = 0;
  }
  if (rl == 0)
  {
    r = 1;
  }
#endif
  return r;
}
//RELOAD

//AUX
int AuxLogic(byte au)
{
  int a;
#ifdef INPUTS_HIGH
  if (au == 1)
  {
    a = 1;
  }
  if (au == 0)
  {
    a = 0;
  }
#endif

#ifdef INPUTS_LOW
  if (au == 1)
  {
    a = 0;
  }
  if (au == 0)
  {
    a = 1;
  }
#endif
  return a;
}
//AUX

//MODE
int ModeLogic(byte mo)
{
  int m;
#ifdef INPUTS_HIGH
  if (mo == 1)
  {
    m = 1;
  }
  if (mo == 0)
  {
    m = 0;
  }
#endif

#ifdef INPUTS_LOW
  if (mo == 1)
  {
    m = 0;
  }
  if (mo == 0)
  {
    m = 1;
  }
#endif
  return m;
}
//MODE

//--------------------------------INPUT LOGIC INVERSION - END----------------------------------------
// FIN!
