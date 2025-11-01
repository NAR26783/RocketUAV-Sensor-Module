//*******************************************************************
//**       RocketUAV Dual Deploy - 10/6/25 - Mike Loman           ***
//**               Adafruit M0 Trinket, PAM8302,                  ***
//**           two BMP280 clones and an ADXL345 clone             ***
//**                  and an I2C Service Module                   ***
//**                     Vacuum testable!!                        ***
//*******************************************************************
//  include libraries
#include <Adafruit_DotStar.h>
#include <Talkie.h>
#include <TalkieUtils.h>
#include <Vocab_US_Large.h>
#include <Vocab_Special.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <SparkFunBME280.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MCP23XXX.h>
#include <Adafruit_MCP23X08.h>

//     ##############      I2C Service Module Silk                  #################  
  #define TEST1      2
  #define TEST2      4
  #define DROGUE     1   //  CH1,  blue pigtail
  #define MAIN       6   //  CH2,  yellow pigtrail  
  #define SNDROGUE   0
  #define SNMAIN     7
  #define PWRON      5
  #define BATTMON    3
  
//     ##############     M0 Trinket Pinouts      #################  
#define AudioPin   A0
#define FireBatt   A2
#define VBATT      A3
#define DotData     7  // Digital #7 - You can't see this pin but it is connected to the internal RGB DotStar data in pin
#define DotClock    8  // Digital #8 - You can't see this pin but it is connected to the internal RGB DotStar clock in pin
#define OnBoardLED 13  // Digital #13 - You can't see this pin but it is connected to the little red status LED

//  declare variable types
float baroAGL, baro2Alt, baro1Alt, alt2Zero, alt1Zero, baro1AGL, baro2AGL, sealvl, deltat;  
float AX, AY, AZ, Acc, AXYsqr, AZsqr, firebattery, battery, altMain, altMax, baro1static;
float AZreading[6],Accreading[6], p1reading[6], p2reading[6], p1ave, p1sum, p2ave, p2sum, AZave, AZsum, Accave, Accsum;
float AXreading[6],AYreading[6], AXave, AXsum, AYave, AYsum, baroAlt;
bool POINTEDUP, POINTEDDOWN, FAIL1, FAIL2, DrogueReq, MainReq, TestReq, resetReq, TestMain, LANDED, ARMED;
bool drogueTimerSet, mainTimerSet, testTimerSet, apogeeTimerSet, launchTimerSet, landedTimerSet, LAUNCH, APOGEE, MAINOUT;
bool launchDetected, landingDetected, ArmReq;
long realmillis, lastrealmillis, looptimer, timezerol, testTimerInterval, pyroTimerInterval;
long drogueTimer, mainTimer, motorTimer, testTimer, sayTimer, apogeeTimer, launchTimer,  landingTimer, landedTimer;
long timeOfLaunch, flightTime, announceTimer;
int keyinput, batt, firebatt, i, aveAGL;
    enum conditions {
      awaiting_POST,
      awaiting_Vertical, 
      awaiting_Launch,      
      awaiting_Apogee, 
      awaiting_MainAlt, 
      awaiting_Landing,
      awaiting_Recovery
    }FLTCON;
    
//   instantiate libraries
Adafruit_DotStar led(1, DotData, DotClock, DOTSTAR_BRG);
Adafruit_MCP23X08 DD;
BME280 baro1;
BME280 baro2;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Talkie voice;  

//  #####################################  Begin Setup  ###############################  
void setup() {

  Wire.begin();
//  Wire.setClock(400000); //Increase I2C data rate to 400kHz
    
  Serial.begin(115200);
  delay(1500);
  Serial.println("HW-611 (BMP280) and GY-291 (ADXL 345) Sensors with Dual Deploy and Talkie output ");
  
//  **********  start voice  AKA Talkie  ******************  
  voice.doNotUseInvertedOutput();
  
//  *********  start Trinket Built-in DotStar under Adafruit DotStar driver  ****************  
  led.begin();
  led.clear();  // Set all colors to 'off'
  led.show();   // Send the updated colors to the hardware.    
  
  //  *********  start I2C Deployment Module under Adafruit MCP23008 drivers  ****************  
  if (!DD.begin_I2C(0x23)) {   //  Module I2C address is 0x21 or 0x23. Default is 0x21.
    Serial.println("Error.  No MCP23008 detected on 0x23...");
  }
  // configure Test, Arm and Ch pins for output
  DD.pinMode(TEST1, OUTPUT);
  DD.pinMode(TEST2, OUTPUT);
  DD.pinMode(PWRON, OUTPUT);
  DD.pinMode(DROGUE, OUTPUT);
  DD.pinMode(MAIN, OUTPUT);
  
  // configure Sn and FireBatt pins for input
  DD.pinMode(BATTMON, INPUT); 
  DD.pinMode(SNDROGUE, INPUT); 
  DD.pinMode(SNMAIN, INPUT); 
  
  delay(100);  //   give it a few...
  //  set all outputs LOW
  DD.digitalWrite(TEST1, LOW);
  DD.digitalWrite(TEST2, LOW);
  DD.digitalWrite(PWRON, LOW);
  DD.digitalWrite(DROGUE, LOW);
  DD.digitalWrite(MAIN, LOW);
    
  //  *****************  start accel  *************************
  while (!accel.begin(0x53)){
    Serial.println("Ooops, no ADXL345 detected on 0x53 ... retrying...");
    delay(100);
  }
   accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  // accel.setDataRate(ADXL345_DATARATE_800_HZ);
   accel.setDataRate(ADXL345_DATARATE_50_HZ);
   
//  *****************************  start baro1 under Sparkfun  ****************
  baro1.setI2CAddress(0x76);
  while (!baro1.beginI2C()) {  
    Serial.println("Failed to find baro1 on 0x76... retrying...");
    delay(100);
  }
  baro1.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  baro1.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  baro1.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  baro1.setPressureOverSample(5);  //  8X  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  baro1.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  baro1.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  baro1.readTempC();
  baro1Alt = baro1.readFloatAltitudeFeet();       
    
//  *****************************  start baro2 under Sparkfun  ****************
  baro2.setI2CAddress(0x77);
  while (!baro2.beginI2C()) {    
    Serial.println("Failed to find baro2 on 0x77... retrying...");
    delay(100);
  }
  baro2.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  baro2.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  baro2.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  baro2.setPressureOverSample(5);  //  8X  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  baro2.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  baro2.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  baro2.readTempC();
  baro2Alt = baro2.readFloatAltitudeFeet();



//  set/initialize flight options
  FLTCON = awaiting_POST;
  testTimerInterval = 50l;   //  how long to put 50uA current through an igniter WITHOUT it going off
  pyroTimerInterval = 2000l; //  how long to put 2A current through an igniter to MAKE IT go off
  altMain=500.;
  altMax=300.;
  alt1Zero=0.;
  alt2Zero=0.;
    
  //  ****************  SET Initial Serial Display  ******************
   FAIL1 = true;
   FAIL2 = true;
   Serial.print(" Type '0' or 't' for continuity test,  ");   
   Serial.println(" Type '1', or '2', to fire the channel ");  

  //  ****************  Read and Announce MCU battery level  ******************
    batt = analogRead(VBATT);
    battery = batt * 3.3/2048 * 2;     //  reading, times ADC volts/bits ratio, times Voltage Divider resistor value ratio.
    voice.say(sp2_POWER); 
    voice.say(sp4_LEVEL);
    int intbattery=battery/1;
    int intdiff=(battery-intbattery)*10./1;
    sayNumber(intbattery);
    voice.say(sp2_POINT);   
    sayNumber(intdiff);
    voice.say(sp2_VOLTS);   
    delay(500);
    voice.say(sp2_FIRE);
    voice.say(sp2_POWER); 
    DD.digitalWrite(PWRON, HIGH);
    ARMED = DD.digitalRead(BATTMON);
    if(ARMED){voice.say(sp2_ON);}else{voice.say(sp3_BROKEN);}
    DD.digitalWrite(PWRON, LOW);

}
//  #####################################  End Setup Begin Loop  ###############################
void loop() {
      sensors_event_t event;

//   *********  read sensors  *********************    
    baro2.readTempC();
    baro2Alt = baro2.readFloatAltitudeFeet();
    
    baro1.readTempC();
    baro1Alt = baro1.readFloatAltitudeFeet();

    accel.getEvent(&event);    
//    AX=event.acceleration.x / 9.8 - .00; //  in gee's
//    AY=event.acceleration.y / 9.8 + .00; // in gee's
//    AZ=event.acceleration.z / 9.8 + .00; //   in gee's
    AX=event.acceleration.x / 9.8 + .02; // in offset corrected gee's
    AY=event.acceleration.y / 9.8 - .00; // in offset corrected gee's
    AZ=event.acceleration.z / 9.8 + .09; // in offset corrected gee's

//  ***********  calculate derived values  ************************

  AX*=32.2;  // in Ft/S^2
  AY*=32.2;  // in Ft/S^2
  AZ*=32.2;  // in Ft/S^2

  AXYsqr=(AX*AX+AY*AY);
  AZsqr=(AZ*AZ);
  Acc=sqrt(AXYsqr+AZsqr);  //  Absolute acceleration in Ft/s*s

//  **************  Rolling average of 5 values to find alt1Zero  ********************  
    p1sum=0.0;    
    for(i=4;i>0;i--){
      p1reading[i+1]=p1reading[i];
      p1sum+=p1reading[i];
    }
    p1reading[1]=baro1Alt;
    p1sum+=baro1Alt;
    p1ave=p1sum/5.;

    p2sum=0.0;
    for(i=4;i>0;i--){
      p2reading[i+1]=p2reading[i];
      p2sum+=p2reading[i];
    }
    p2reading[1]=baro2Alt;
    p2sum+=baro2Alt;
    p2ave=p2sum/5.;
    
    baro1AGL = baro1Alt-alt1Zero;
    baro2AGL = baro2Alt-alt2Zero;  
    baroAGL = (baro1AGL + baro2AGL)/2.;
    baroAlt = (baro1Alt + baro2Alt)/2.;

//   **********************  set bools  **********************************
// If Z is plus down, i.e. towards gravity, then:
// Assuming Acc is 1 gee, theta is less than 20 deg when:
    if((.342 * AZsqr) > AXYsqr && AZ < 0.){POINTEDDOWN = true;}else{POINTEDDOWN=false;}
    if((.342 * AZsqr) > AXYsqr && AZ > 0.){POINTEDUP = true;}else{POINTEDUP=false;}
//if(!POINTEDUP && ! POINTEDDOWN)Serial.println("SIDEWAYS");
//  if(POINTEDUP)Serial.println("POINTED UP");
//  if(POINTEDDOWN)Serial.println("POINTED DOWN");

      //  *****   Enter Flight Condition specific processing    **************
  switch (FLTCON){
    case awaiting_POST:
      if(millis() > 10000l){
    //  ****************  Announce Ground Level   ******************
        voice.say(sp5_GROUND); 
        voice.say(sp4_LEVEL);
        voice.say(sp4_IS);
        long intbaroAlt=baroAlt/1;
        sayNumber(intbaroAlt);
        voice.say(sp2_FEET);   
        resetReq=true;
        FLTCON = awaiting_Vertical;
      }
    break;    
    case awaiting_Vertical:
      if(POINTEDUP){

  //  ****************  Read and Announce Firing battery level  ******************
        firebatt = analogRead(FireBatt);
        Serial.println(firebatt);
        firebattery = firebatt * 3.3/2048 * 75./10.;     //  reading, times ADC volts/bits ratio, times Voltage Divider resistor value ratio.
        voice.say(sp2_FIRE);
        voice.say(sp2_POWER); 
        int intfirebattery=firebattery/1;
        int intfirediff=(firebattery-intfirebattery)*10./1;
        sayNumber(intfirebattery);
        voice.say(sp2_POINT);   
        sayNumber(intfirediff);
        voice.say(sp2_VOLTS);     
Serial.println("still processing...");

        DD.digitalWrite(PWRON, HIGH);
        ARMED = DD.digitalRead(BATTMON);
        DD.digitalWrite(PWRON, LOW);
        TestReq = true;
        resetReq=true;
        FLTCON = awaiting_Launch;
          
      }
    break;
    
    case awaiting_Launch:
  if(POINTEDUP){
      if(launchDetected){
          if(baroAGL < 100.)  {      //  false alarm, go back and wait for launch
            launchDetected=false;
          }
          if(launchTimer <= millis()){    //  we have liftoff!
            ArmReq = true;
            FLTCON = awaiting_Apogee;
          }
      }else{            
        if(baroAGL > 100.){           //   over 100' for 1/4s? we're there.
          timeOfLaunch = millis();
          launchDetected=true;
          launchTimer = millis() + 250l;
        }
      }
   }else{  
        FLTCON = awaiting_Vertical;   //  false alarm, go back and wait for Vertical
        }
    break;
    
    case awaiting_Apogee:
      if(baroAGL > altMax){                //  a fresh altMax... 
        altMax = baro1AGL;                //   still climbing...
        apogeeTimer = millis() + 1000l;
      }
      if(apogeeTimer <= millis()){    //   a stale altMax, call it
        DrogueReq = true;
        FLTCON = awaiting_MainAlt;
      }
    break;
    
    case awaiting_MainAlt:
      if(baroAGL <= altMain){
        MainReq = true;
        FLTCON = awaiting_Landing;
      }
    break;
    case awaiting_Landing:
        if(landingDetected){
            if(baroAGL > 50.)  {              //  false alarm, go back and wait for landing
                landingDetected=false;
            }
            if(landingTimer <= millis()){      //  we have landing!
                flightTime -= timeOfLaunch;
                announceTimer = millis();
                FLTCON = awaiting_Recovery;
            }
        }else{                                  //  landing not yet detected
            if(baro1AGL < 50.){              //  under 50' for 2s?  we're down.
                flightTime = millis();
                landingDetected=true;
                landingTimer = millis() + 2000l;
            }
        }
    break;
    case awaiting_Recovery:
        if(announceTimer <= millis()){     
          announceTimer = millis() + 15000l;
          voice.say(sp5_FLIGHT); 
          voice.say(sp2_TIME);
          long intflttime=flightTime/1000;
          long intdiff=(flightTime-intflttime)*10./1000;
          sayNumber(intflttime);
          voice.say(sp2_POINT);   
          sayNumber(intdiff);
          voice.say(sp2_SECONDS);   
          delay(200);
          voice.say(sp5_ALTITUDE); 
          long intaltMax=altMax/1;
          sayNumber(intaltMax);
          voice.say(sp2_FEET);       
        }        
    break;
  }

//  *****************   Take Actions   ********************************
  if(ArmReq){
    DD.digitalWrite(PWRON, HIGH);
    ArmReq = false;
    ARMED = DD.digitalRead(BATTMON);
    Serial.print("ARMED   ");
    Serial.println(ARMED);
  }
  if(TestReq){
    DD.digitalWrite(TEST1, HIGH);
    DD.digitalWrite(TEST2, HIGH);
    testTimer = millis()+500l;
    testTimerSet=true;
    TestReq=false;
  }
  if(DrogueReq){
    DD.digitalWrite(DROGUE, HIGH);
    drogueTimer = millis()+2000l;  
    drogueTimerSet=true;
    DrogueReq=false;
  }
  if(MainReq){
    DD.digitalWrite(MAIN, HIGH);
    mainTimer = millis()+2000l; 
    mainTimerSet=true;
    MainReq=false;
  }
  if(resetReq){
    resetReq=false;
    alt1Zero=p1ave;    
    alt2Zero=p2ave;      
  }    
  switch(keyinput){
  case 114:  // r
    resetReq = true;
  break ;          
  }
//  *****************   check timers   ********************************
  if(testTimerSet && testTimer<=millis()){
    //  Read and Report continuity test results
    led.clear();
    led.setPixelColor(0, led.Color(150, 150, 150));    //  Display white for default/unknown    
    FAIL1=DD.digitalRead(SNDROGUE);
          Serial.print("FAIL1   ");
          Serial.println(FAIL1);
    FAIL2=DD.digitalRead(SNMAIN);
          Serial.print("FAIL2   ");
          Serial.println(FAIL2);    
    if(!FAIL1 && !FAIL2 && ARMED){
      Serial.println("Ready to Launch!");
      led.clear();
      led.setPixelColor(0, led.Color(150, 0, 0));    //  Display green
      voice.say(sp2_READY); 
      voice.say(sp3_FOR);
      voice.say(sp5_LAUNCH);
    }
    
    if(FAIL1){
      Serial.println("Drogue failed continuity check...");
      led.clear();
      led.setPixelColor(0, led.Color(0, 150, 0));    //  Display red
      voice.say(sp2_CIRCUIT); 
      voice.say(sp2_ONE);
      voice.say(sp3_BROKEN);   
    }
    if(FAIL2){
      Serial.println("Main failed continuity check...");
      led.clear();
      led.setPixelColor(0, led.Color(150, 150, 0));  //  Display yellow
      voice.say(sp2_CIRCUIT); 
      voice.say(sp2_TWO);
      voice.say(sp3_BROKEN);
    }
    if(FAIL1 && FAIL2){
      Serial.println("BOTH failed continuity check...");
      led.clear();
      led.setPixelColor(0, led.Color(0, 50, 150));  //  Display purple
    }
    if(!ARMED){
      Serial.println("Not Armed - Battery may be weak or disconnected...");
      led.clear();
      led.setPixelColor(0, led.Color(0, 0, 150));    //  Display blue
      voice.say(sp4_ABORT);
      voice.say(sp5_LAUNCH);
    }
    led.show();
    DD.digitalWrite(TEST1, LOW);
    DD.digitalWrite(TEST2, LOW);
    testTimerSet=false;    
  }  
  if(drogueTimerSet && drogueTimer<=millis()){
    DD.digitalWrite(DROGUE, LOW);
    Serial.println("Drogue power off");  
    drogueTimerSet=false;
  }
  if(mainTimerSet && mainTimer<=millis()){
    DD.digitalWrite(MAIN, LOW);
    Serial.println("Main power off");  
    mainTimerSet=false;
  }

    //   ********************  store last values for next pass   ***************
  keyinput = 100;  //  d   UNUSED
}
//  #####################################  End Void  ###############################
//  ***********  sayNumber Subroutine  ************************
// Say any number between -999,999 and 999,999 
void sayNumber(long n) {
    if (n < 0) {
        voice.say(sp2_MINUS);
        sayNumber(-n);
    } else if (n == 0) {
        voice.say(sp2_ZERO);
    } else {
        if (n >= 1000) {
            int thousands = n / 1000;
            sayNumber(thousands);
            voice.say(sp2_THOUSAND);
            n %= 1000;
            if ((n > 0) && (n < 100))
                voice.say(sp2_AND);
        }
        if (n >= 100) {
            int hundreds = n / 100;
            sayNumber(hundreds);
            voice.say(sp2_HUNDRED);
            n %= 100;
            if (n > 0)
                voice.say(sp2_AND);
        }
        if (n > 19) {
            int tens = n / 10;
            switch (tens) {
            case 2:
                voice.say(sp2_TWENTY);
                break;
            case 3:
                voice.say(sp2_THIR_);
                voice.say(sp2_T);
                break;
            case 4:
                voice.say(sp2_FOUR);
                voice.say(sp2_T);
                break;
            case 5:
                voice.say(sp2_FIF_);
                voice.say(sp2_T);
                break;
            case 6:
                voice.say(sp2_SIX);
                voice.say(sp2_T);
                break;
            case 7:
                voice.say(sp2_SEVEN);
                voice.say(sp2_T);
                break;
            case 8:
                voice.say(sp2_EIGHT);
                voice.say(sp2_T);
                break;
            case 9:
                voice.say(sp2_NINE);
                voice.say(sp2_T);
                break;
            }
            n %= 10;
        }
        switch (n) {
        case 1:
            voice.say(sp2_ONE);
            break;
        case 2:
            voice.say(sp2_TWO);
            break;
        case 3:
            voice.say(sp2_THREE);
            break;
        case 4:
            voice.say(sp2_FOUR);
            break;
        case 5:
            voice.say(sp2_FIVE);
            break;
        case 6:
            voice.say(sp2_SIX);
            break;
        case 7:
            voice.say(sp2_SEVEN);
            break;
        case 8:
            voice.say(sp2_EIGHT);
            break;
        case 9:
            voice.say(sp2_NINE);
            break;
        case 10:
            voice.say(sp2_TEN);
            break;
        case 11:
            voice.say(sp2_ELEVEN);
            break;
        case 12:
            voice.say(sp2_TWELVE);
            break;
        case 13:
            voice.say(sp2_THIR_);
            voice.say(sp2__TEEN);
            break;
        case 14:
            voice.say(sp2_FOUR);
            voice.say(sp2__TEEN);
            break;
        case 15:
            voice.say(sp2_FIF_);
            voice.say(sp2__TEEN);
            break;
        case 16:
            voice.say(sp2_SIX);
            voice.say(sp2__TEEN);
            break;
        case 17:
            voice.say(sp2_SEVEN);
            voice.say(sp2__TEEN);
            break;
        case 18:
            voice.say(sp2_EIGHT);
            voice.say(sp2__TEEN);
            break;
        case 19:
            voice.say(sp2_NINE);
            voice.say(sp2__TEEN);
            break;
        }
    }
}
