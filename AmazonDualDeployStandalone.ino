///*********************************************************************
//**                        AmazonDualDeployStandalone               ***
//**              Dual Deploy Altimeter - 9/23/26 - Mike Loman       ***
//**                 Adafruit M0 Trinket, PAM8302 with               ***
//**                  Sensor Plate and Service Module                ***
//**                        VACUUM TESTABLE                          ***
//**********************************************************************

  #include <Talkie.h>
  #include <TalkieUtils.h>
  #include <Vocab_US_Large.h>
  #include <Vocab_Special.h>
  #include "Wire.h"
  #include <Adafruit_Sensor.h>
  #include <SparkFunBME280.h>
  #include <Adafruit_ADXL345_U.h>
  #include <Adafruit_MCP23X08.h>  
  #include <Adafruit_GFX.h>
  #include <Adafruit_SH110X.h>
  #include <Adafruit_DotStar.h>

//     ********************      declare variable types   ********************
  bool OLEDon, LoRaon, Common, firstFix, GPSon, baro1on, baro2on, accelon, I2C21on, I2C23on, MCPon;
  bool POINTEDUP, baroLaunchDetected, accelLaunchDetected, landingDetected, altMainDetected, FAIL1, FAIL2, TESTED, contOK;
  bool ArmReq, TestReq, DrogueReq, MainReq, DrogueTimerSet, MainTimerSet, TestTimerSet;  
  long announceTimer, descentTimer, readGPStimer, parseGPStimer, displayTimer, altMainTimer;
  long timeOfLaunch, flightTime;  
  long oldAltTimer, apogeeTimer, baroLaunchTimer, accelLaunchTimer,  landingTimer, TestTimer, DrogueTimer, MainTimer;
  float alt2Zero, alt1Zero, baro1AGL, baro2AGL, altDiff, oldAlt, altMain, altMax;
  float baro1Alt, baro2Alt, baroAGL, AX, AY, AZ, Acc, AXYsqr, AZsqr;
  float p1reading[6], p2reading[6], p1ave, p1sum, p2ave, p2sum, baroAlt;  
  int i;
  bool RTLAnnounced = false;
  bool gndLvlAnnounced = false;
  bool testAnnounced = false;
    
  enum conditions {
    awaiting_POST,
    awaiting_Vertical, 
    awaiting_Launch,      
    awaiting_Apogee, 
    awaiting_altMain, 
    awaiting_Landing,    
    awaiting_Recovery
  }FLTCON;

//#define VOICEPIN     A0     <--|
//pinMode(VOICEPIN, OUTPUT);  <--|--  BREAKS use of A0 pin for ADC output on M0 and M4 chipsets!

//     ##############     M0 Trinket Pinouts      #################  
#define DotData     7  // Digital #7 - You can't see this pin but it is connected to the internal RGB DotStar data in pin
#define DotClock    8  // Digital #8 - You can't see this pin but it is connected to the internal RGB DotStar clock in pin
#define OnBoardLED 13  // Digital #13 - You can't see this pin but it is connected to the little red status LED

//   ##############   RocketUAV Sensor Plate Pinouts    #################  
  #define VBATT        A3
  #define FIREBATT     A4  //  Silk sez A2, Oops!

//     ##############      Service Module Silk      #################  
  #define TEST1      2
  #define TEST2      4
  #define DROGUE     1   //  DROGUE,  blue pigtail
  #define MAIN       6   //  MAIN,  yellow pigtrail  
  #define SNDROGUE   0
  #define SNMAIN     7
  #define PWRON      5
  #define BATTMON    3     

//     ********************      instantiate libraries   *********************
  Adafruit_SH1107 OLED = Adafruit_SH1107(128, 128, &Wire, -1, 400000, 400000);
  BME280 baro1;
  BME280 baro2;
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  Adafruit_MCP23X08 DD;
  Talkie voice;
  Adafruit_DotStar led(1, DotData, DotClock, DOTSTAR_BRG);
    
//  ####################  Begin Setup  ###############################  
void setup() {

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
    
  Serial.begin(115200);
  delay(1000);
  Serial.print("Dual Deployment Altimeter with Amazon Sensors and Talkie HMI ");
  
//  **********  start voice  AKA Talkie  ******************  
  voice.doNotUseInvertedOutput();

//  *********  start Trinket Built-in DotDStar under Adafruit driver  ****************  
  led.begin();
  led.clear();  // Set all pixel colors to 'off'
  led.show();   // Send the updated pixel colors to the hardware.  
  
//  ********************  Start OLED  ********************
  delay(250);
  OLEDon = true;
  if(!OLED.begin(0x3D, true)){      // Address 0x3D default
    OLEDon = false;
  }
  Serial.println("  OLED should be on");
  if(OLEDon){
    OLED.clearDisplay();
    OLED.setTextSize(2);             // double pixel scale
    OLED.setTextColor(SH110X_WHITE);        // Draw white text
    OLED.setCursor(0,0);           
    OLED.println("Rocket DD");
    OLED.println("Standalone");
    OLED.println("Unit");
    OLED.display();    
  }
  delay(5000);
//     ********************     start accel  *************************
//    Serial.println("    Starting Accel ");
  accelon = true;
  if(!accel.begin(0x53)){              //  Module I2C address is 0x53 or 0x1D. Default is 0x53.
    accelon = false;
  }
  if(accelon){
//   accel.setRange(ADXL345_RANGE_16_G);
  accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  // accel.setDataRate(ADXL345_DATARATE_800_HZ);
   accel.setDataRate(ADXL345_DATARATE_50_HZ);
  }

//    ********************     start baro1 under Sparkfun  ****************
  baro1.setI2CAddress(0x76);
    baro1on = true;
  if(!baro1.beginI2C()) {  
    baro1on = false;
  }
  if(baro1on){
    baro1.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
    baro1.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
    //  baro1.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    baro1.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    baro1.setPressureOverSample(5);  //  8X  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    baro1.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    baro1.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
    baro1.readTempC();
    baro1Alt = baro1.readFloatAltitudeFeet();  
  }     
    
//   ********************     start baro2 under Sparkfun  ****************
  baro2.setI2CAddress(0x77);
  baro2on = true;
  if(!baro2.beginI2C()) {    
    baro2on = false;
  }
  if(baro2on){
    baro2.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
    baro2.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
    //  baro2.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    baro2.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    baro2.setPressureOverSample(5);  //  8X  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    baro2.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    baro2.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
    baro2.readTempC();
    baro2Alt = baro2.readFloatAltitudeFeet();
  }
    if(accelon && baro1on && baro2on){
      OLED.println("3 Sensors");
      OLED.println("Started");
      OLED.display();
    }else{
      OLED.println("Sensors");
      OLED.println("Startup");
      OLED.println("Failed");
      OLED.display();
      delay(5000);   
    }
//  *********  start I2C Deployment Module under Adafruit MCP23008 drivers  ****************  
  I2C21on = true;
  if(!DD.begin_I2C(0x21)) {   //  Module I2C address is 0x21 or 0x23. Default is 0x21.
    I2C21on = false;
  }

  if(!I2C21on){
    I2C23on = true;
    if(!DD.begin_I2C(0x23)) {   //  Module I2C address is 0x21 or 0x23. Default is 0x21.
      I2C23on = false;
    }
  }
  
//     ##############      configure Test, Arm and Ch pins for output
  DD.pinMode(TEST1, OUTPUT);
  DD.pinMode(TEST2, OUTPUT);
  DD.pinMode(PWRON, OUTPUT);
  DD.pinMode(DROGUE, OUTPUT);
  DD.pinMode(MAIN, OUTPUT);
  
//     ##############      configure Sn and Batt pins for input
  DD.pinMode(BATTMON, INPUT); 
  DD.pinMode(SNDROGUE, INPUT); 
  DD.pinMode(SNMAIN, INPUT); 
  delay(100);  //   give it a few...
  
//     ##############      set all outputs LOW
  DD.digitalWrite(TEST1, LOW);
  DD.digitalWrite(TEST2, LOW);
  DD.digitalWrite(PWRON, LOW);
  DD.digitalWrite(DROGUE, LOW);
  DD.digitalWrite(MAIN, LOW);

  if(OLEDon){
  OLED.clearDisplay();
  OLED.setCursor(0,0);           
  OLED.println("Service");
  OLED.println("Module");
    if(!I2C21on && !I2C23on){
      OLED.println("  MCP ");
      OLED.println("Failed");
      delay(5000);      
    }
    if(I2C23on){
      OLED.println("  MCP ");
      OLED.println("Started");
      OLED.println("on 0x23");
    }  
    if(I2C21on){
      OLED.println("  MCP ");
      OLED.println("Started");
      OLED.println("on 0x21");
    }    
  OLED.display();    
  }
  


  // configure battery pins for input
  pinMode(VBATT, INPUT);  
  pinMode(FIREBATT, INPUT); 

  //  ****************  Set Initial Conditions   ******************
    alt1Zero = 0.0;  
    alt2Zero = 0.0;
    altMax=0.;
    TESTED = false;
    FAIL1 =true;
    FAIL2 =true;
    altMain = 400.;
          if(OLEDon){
            OLED.clearDisplay();
            OLED.setCursor(0,0);        
            OLED.println("Setup");
            OLED.println("Done");
            OLED.display();
          }

    
}
//  #####################################  End Setup Begin Void  ###############################
void loop() {
    sensors_event_t event;
//   *********  read sensors  *********************    
    baro1.readTempC();
    baro1Alt = baro1.readFloatAltitudeFeet();       
    
    baro2.readTempC();
    baro2Alt = baro2.readFloatAltitudeFeet();       

    accel.getEvent(&event);    
//    AX=event.acceleration.x / 9.8 - .00; //  in gee's
//    AY=event.acceleration.y / 9.8 + .00; // in gee's
//    AZ=event.acceleration.z / 9.8 + .00; //   in gee's
    AX=event.acceleration.x / 9.8 + .00; // in offset corrected gee's
    AY=event.acceleration.y / 9.8 + .00; // in offset corrected gee's
    AZ=event.acceleration.z / 9.8 + .07
    ; // in offset corrected gee's

//     ##############      calculate derived values  ##############*    
  AX*=32.2;  // in Ft/S^2
  AY*=32.2;  // in Ft/S^2
  AZ*=32.2;  // in Ft/S^2
  AXYsqr=(AX*AX+AY*AY);
  AZsqr=(AZ*AZ);
  Acc=sqrt(AXYsqr+AZsqr);  //  Absolute acceleration in Ft/s*s
  
//  **************  Rolling average of 5 pressure values, averaged  ********************  
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
 
    baro1AGL = baro1Alt-alt1Zero;  //  reading - ave alt 0
    baro2AGL = baro2Alt-alt2Zero;
    baroAGL = (baro1AGL + baro2AGL)/2.;
    
//   ##############*  set bools  ##############
// If Z is plus down, i.e. towards gravity, then:
// Assuming Acc is 1 gee, theta is less than 20 deg when:
    if((.342 * AZsqr) > AXYsqr && AZ > 0.){POINTEDUP = true;}else{POINTEDUP=false;}


    
//  #############   Enter Flight Condition specific processing    ################
  switch (FLTCON){
    case awaiting_POST:
      if(oldAltTimer <= millis()){
        altDiff = oldAlt - baroAGL;
        if(fabs(altDiff) < 10.){   //  Has baroAGL stabilized to within 10 feet over the last 1/4s?    
      
//  ****************  Announce Power Level   ******************
          int batt = analogRead(VBATT);
          float battery = batt * 3.3/1024 * 20./10.;//  reading, times ADC volts/bits ratio, times Voltage Divider resistor value ratio.
          voice.say(sp2_POWER); 
          voice.say(sp4_LEVEL);
          int intbattery=battery/1;
          int intdiff=(battery-intbattery)*10./1;
          sayNumber(intbattery);
          voice.say(sp2_POINT);   
          sayNumber(intdiff);
          voice.say(sp2_VOLTS);   
      
//  ****************  Announce Ground Level   ******************
          voice.say(sp5_GROUND); 
          voice.say(sp4_LEVEL);
          voice.say(sp4_IS);
          int intaltZero=p1ave+p2ave/2;
          sayNumber(intaltZero);
          voice.say(sp2_FEET);   
          

          if(OLEDon){
            OLED.clearDisplay();
            OLED.setCursor(0,0);        
            OLED.println("Gnd Alt");
            OLED.println(baroAGL, 0);
            OLED.display();
          }
        Serial.print("  Ground Altitude is ");
        Serial.println(baroAGL, 0);        
        baroAGL = 0.0;    //  that's said.  Now, reset baroAGL to AGL 
        alt1Zero=p1ave;    //  reset baroAGL to AGL 
        alt2Zero=p2ave;         
        FLTCON = awaiting_Vertical;   
        }else{
          oldAlt = baroAGL;
          oldAltTimer = millis() + 250l;
        }
      } 
      break;    
    
    case awaiting_Vertical:
    
      if(!TESTED && POINTEDUP){
        if(!TestTimerSet){
            TestReq = true;    //   Continuity Test requested
            Serial.println("  Cont test req ");
        }            
      }
      if(TESTED && POINTEDUP){
        FLTCON = awaiting_Launch;
      }

    break;
    
    case awaiting_Launch:
      if(baroLaunchDetected){
          if(baroAGL < 100.)  {      //  false alarm, go back and wait for launch
            baroLaunchDetected=false;
          }
          if(baroLaunchTimer <= millis()){    //  we have liftoff!
            DD.digitalWrite(PWRON, HIGH);    //  arm 3S circuit            
            FLTCON = awaiting_Apogee;
          }
      }else{            
        if(baroAGL > 100.){           //   over 100' for 1/4s? we're there.
          baroLaunchDetected=true;
          timeOfLaunch = millis();          
          baroLaunchTimer = millis() + 250l;
        }
      }
      if(accelLaunchDetected){
          if(Acc < 100.)  {      //  false alarm, go back and wait for launch
            accelLaunchDetected=false;
          }
          if(accelLaunchTimer <= millis()){    //  we have liftoff!
            led.clear();
            led.show();                
            DD.digitalWrite(PWRON, HIGH);    //  arm 3S circuit            
            FLTCON = awaiting_Apogee;
          }
      }else{            
        if(Acc > 100.){           //   over 3g for 1/4s? we're there.
          accelLaunchDetected=true;
          timeOfLaunch = millis();
          accelLaunchTimer = millis() + 250l;
        }
      }
    break;
    
    case awaiting_Apogee:
      if(baroAGL > altMax){                //  a fresh altMax... 
        altMax = baroAGL;                //   still climbing...
        apogeeTimer = millis() + 1000l;
      }
      if(Acc < 40.){                //  almost quiescent... 
        if(-AZ/Acc < .6){         //  flopped over, call it
          DrogueReq = true;
          FLTCON = awaiting_altMain;        
        }
      }
      if(apogeeTimer <= millis()){    //   a stale altMax, call it
        DrogueReq = true;

        if(OLEDon){     
          OLED.clearDisplay();
          OLED.setCursor(0,0);     
          OLED.println("Apogee");
          OLED.println("altMAX");
          OLED.println(altMax, 0);  
          OLED.println("baroAGL");
          OLED.println(baroAGL, 0);  
          OLED.display();
        }
      FLTCON = awaiting_altMain;
      }
    break;
    
    case awaiting_altMain:
//********  send message every 8s during descent ********************      
        if(descentTimer <= millis()){     
          descentTimer = millis() + 8000l;

          if(OLEDon){     
            OLED.clearDisplay();
            OLED.setCursor(0,0);     
            OLED.println("Descent");
            OLED.println("baroAGL");
            OLED.println(baroAGL, 0);  
            OLED.display();
          }              
        }
      if(altMainDetected){
          if(baroAGL > altMain){      //  false alarm, go back and wait for altMain
            altMainDetected=false;
          }
          if(altMainTimer <= millis()){    //  pop the main
            MainReq = true;
            FLTCON = awaiting_Landing;
          }
      }else{            
         if(baroAGL <= altMain){           //   below main Alt for 1/2s? we're there.
          altMainDetected=true;
          altMainTimer = millis() + 500l;
        }
      }
    break;

    case awaiting_Landing:
        if(landingDetected){
          if(baroAGL > 50.)  {              //  false alarm, go back and wait for landing
              landingDetected=false;
          }
          if(landingTimer <= millis()){      //  we have landing!
              announceTimer = millis();
              flightTime -= timeOfLaunch;              
              FLTCON = awaiting_Recovery;
          }
        }else{                                  //  landing not yet detected
          if(baro1AGL < 50.){              //  under 50' for 2s?  we're down.
            flightTime = millis();            
            landingDetected=true;
            landingTimer = millis() + 2000l;
            if(descentTimer <= millis()){     
              descentTimer = millis() + 8000l;

              if(OLEDon){     
                OLED.clearDisplay();
                OLED.setCursor(0,0);     
                OLED.println("Main Out");
                OLED.println("baroAGL");
                OLED.println(baroAGL, 0);  
                OLED.display();
              }              
            }                
          }
        }
    break;
    
    case awaiting_Recovery:
        DD.digitalWrite(PWRON, LOW);    //  safe 3S circuit
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

//  ##################   Take Actions   ###########################

  if(TestReq){
  Serial.println("  Continuity test underway ");    
    DD.digitalWrite(TEST1, HIGH);
    DD.digitalWrite(TEST2, HIGH);
    TestTimer = millis()+500l;
    TestTimerSet=true;
    TestReq=false;
  }
  if(DrogueReq){
    DD.digitalWrite(DROGUE, HIGH);
    DrogueTimer = millis()+2000l;  
    DrogueTimerSet=true;
    DrogueReq=false;
  }
  if(MainReq){
    DD.digitalWrite(MAIN, HIGH);
    MainTimer = millis()+2000l; 
    MainTimerSet=true;
    MainReq=false;
  }
      
//  #####################   check timers   ####################

  if(TestTimerSet && TestTimer <= millis()){
  Serial.print("  Continuity test results are ");
  //  Read & report continuity & batt test results
    DD.digitalWrite(PWRON, HIGH);
    delay(50);
    int Vfire = analogRead(FIREBATT);
    float V3S = Vfire * 3.3/1024 * 85.0/10.0;//  reading, times ADC volts/bits ratio, times Voltage Divider resistor value ratio.
    Serial.print("  V3S  ");
    Serial.println(V3S);    
    DD.digitalWrite(PWRON, LOW);    
    FAIL1=DD.digitalRead(SNDROGUE);
    FAIL2=DD.digitalRead(SNMAIN);
    DD.digitalWrite(TEST1, LOW);
    DD.digitalWrite(TEST2, LOW);
    Serial.print("  FAIL1  ");
    Serial.println(FAIL1);   
    Serial.print("  FAIL2  ");
    Serial.println(FAIL2);   
    TestTimerSet=false;    
    TESTED = true;

    if(OLEDon){     
      OLED.clearDisplay();
      OLED.setCursor(0,0);     
      OLED.print("FAIL1 ");
      OLED.println(FAIL1);
      OLED.print("FAIL2 ");
      OLED.println(FAIL2);
      OLED.print("BATT ");
      OLED.println(V3S);  
      OLED.display();
    }
  
    if((!testAnnounced) && (TESTED)){
      contOK = true;
      if(V3S < 10.3){
        voice.say(sp2_FIRE); 
        voice.say(sp2_POWER); 
        voice.say(sp2_LOW);  
        voice.say(sp2_ABORT);  
        led.clear();
        led.setPixelColor(0, led.Color(0, 150, 0));    //  Display red        
        led.show();   
        contOK =  false;        
      }else{
        voice.say(sp2_FIRE); 
        voice.say(sp2_POWER); 
        voice.say(sp4_LEVEL);
        int intbattery=V3S/1;
        int intdiff=(V3S-intbattery)*10./1;
        sayNumber(intbattery);
        voice.say(sp2_POINT);   
        sayNumber(intdiff);
        voice.say(sp2_VOLTS);   
      }
      if(FAIL1){
        voice.say(sp2_CIRCUIT); 
        voice.say(sp2_ONE); 
        voice.say(sp3_BROKEN);
        led.clear();
        led.setPixelColor(0, led.Color(0, 150, 0));    //  Display red        
        led.show();           
        contOK =  false;
      }
      if(FAIL2){
        voice.say(sp2_CIRCUIT); 
        voice.say(sp2_TWO); 
        voice.say(sp3_BROKEN);   
                   led.clear();
        led.setPixelColor(0, led.Color(0, 150, 0));    //  Display red        
        led.show();   
        contOK =  false;
      }
      if(contOK){
        voice.say(sp4_READY); 
        voice.say(sp2_TWO); 
        voice.say(sp5_LAUNCH);
        led.clear();
        led.setPixelColor(0, led.Color(150, 0, 0));    //  Display green        
        led.show();        
      }
    testAnnounced = true;
    }
  }
  if(DrogueTimerSet && DrogueTimer<=millis()){
    DD.digitalWrite(DROGUE, LOW);
    DrogueTimerSet=false;
    if(OLEDon){     
      OLED.clearDisplay();
      OLED.setCursor(0,0);     
      OLED.println("Drogue");
      OLED.println("Off");
      OLED.display();  
    }      
  }
  if(MainTimerSet && MainTimer<=millis()){
    DD.digitalWrite(MAIN, LOW);
    MainTimerSet=false;
    if(OLEDon){     
      OLED.clearDisplay();
      OLED.setCursor(0,0);     
      OLED.println("Main");
      OLED.println("Off");
      OLED.display();  
    }    
  }
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
