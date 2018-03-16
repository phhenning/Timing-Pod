/**
 * 
 * This Program displays the Time and displays led and start tone once he has scanned his RFID watch.
 * The scan time is  written to the RFID watch
 * -----------------------------------------------------------------------------------------
 *  1. User action: Select Start pod ID by un-commenting the appropriate line in the 
 *                  STATION/POD IDENTIFICATION section, which makes the relevant
 *                  RFID data blocks, and podID available.(This code must only be used for start pods!)
 *  2. Upload and check LCD screen is displaying correct Stage description and Time is synced with other pods
 *  3. Functionality: Main Loop() calls writeCard() when card is presented.
 *  4. Functionality: writeCard() calculates epoch time of the swipe and writes
 *                    time to appropriate data block.
 *  5. If an error with block authenticate or data write, then return to main loop().
 *  6. Upon write success, activate buzzer tone  and light up LEDS 
 *  7. writeCard function ends, return to main loop(), halt PICC and stop encryption.
*/

//LIBRARIES
#include <Wire.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>  //Library extends MFRC522.h to support RATS for ISO-14443-4 PICC.
#include <LCD5110_Graph.h> 
#include <DS3232RTC.h>
#include <TimeLib.h>
#include <Streaming.h>      //not sure this is needed? Dan
#include "pitches.h"     //if error message involving pitches.h comes up then make sure pitches.h file is in the same directory as the sketch
#include "iTimeAfricaV2.h"
#include <stdlib.h>
#include <EEPROM.h>

//GLOBAL DEFINITIONS
int tagCount = 0;                                           // Cont that incremetns on eatch tag read
int flashLed = LOW;                                         // LED state to implement flashing
int typeIndex = 0xFF;                                       // Index for pod identity in the pod Type List
podType thisPod ;                                           // Identity configured for tehis pod
unsigned long sqCounter = 0;                                // Couner that increments in intterup routine
unsigned long baseTimeS;                                    // Base time in sec from RTC to withc we sync the ms count
bool battGood = 1;

// RFID COMPONENT Create MFRC522 instance
MFRC522 mfrc522(RFID_CS, LCD_RST);                          // PH pins from PCB header file 
MFRC522::MIFARE_Key key;

// LCD COMPONENT
// LCD5110 myGLCD(4,5,6,A1,7); V2.0 green pcb
// LCD5110 myGLCD(4,5,6,7,8); (v1.0 blue PCB)
LCD5110 myGLCD(LCD_CLK, LCD_DIN , LCD_COM , LCD_RST, LCD_CE);   // PH pins from PCB header file
extern unsigned char TinyFont[];      // for 5110 LCD display
extern unsigned char SmallFont[];     // for 5110 LCD display
extern unsigned char MediumNumbers[]; // for 5110 LCD display
extern unsigned char BigNumbers[];    // for 5110 LCD display

/*
 * BELOW IS STATION/POD IDENTIFICATION!
 * 
 * Uncomment the appropriate line to identify this station/pod (Don't use this code for finish PODS!)
 */
// move to header file
// byte dataBlocks = 8;  String podID = ("Stage 1 Start");       // Stage 1 Start

void setup() 
{
    Wire.begin();                                         // Init I2C buss
    // PH to use status LED serial needs to be disbaled
   // Serial.begin(57600);                               // Initialize serial communications with the PC
   // while (!Serial); {}                                // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)    
    
    myGLCD.InitLCD();                                     // initialise LCD
    initIoPins();
    initMfrc522();
    configrePod();
    initAfricaTimer();
}


int updateCnt =0;
void loop() {
    updateLcdScreen();                      // invoke LCD Screen fucntion
    // when setSyncInterval(interval); runs out the status is changed to "timeNeedsSync"
    if ( timeStatus()  == timeNeedsSync ) {      
        setSyncProvider(RTC.get);           // manual resynch is done
    }   

    if ( ! mfrc522.PICC_IsNewCardPresent()) {    // Look for new cards
        return;
    }
    if ( ! mfrc522.PICC_ReadCardSerial()) {      // Select one of the cards
        return;
    } else {
        writeCard();                          // call writeCard() function
        mfrc522.PICC_HaltA();                 // Halt PICC
        mfrc522.PCD_StopCrypto1();            // Stop encryption on PCD
    }
}


// interupt is triggered on up and down flanks, we only want one flank
ISR (PCINT1_vect) {     // handle pin change interrupt for A0 to A5 here
    if ( digitalRead(SQ_WAVE) == HIGH) {
        sqCounter ++;                 
    }
} 

unsigned long getAfricaTimeMs (){
    unsigned long ms = sqCounter/1.024;  // tic 1/1024s ms  from stable 1042Hz RTC clock
    // more efficent calculation but counter over runs every 80 mins, less efficent way has 50day overun.
    //  unsigned long ms = (sqCounter * 1000) >> 10;  // tic 1/1024s ms  from stable 1042Hz RTC clock
    return ms;
}

void writeCard() {
    unsigned long realMs = getAfricaTimeMs();       // read the ms time from int counter
    unsigned long baseTimeMs = baseTimeS * 1000;    // convert base time to ms
    // Time stamp = base time (ms) + realMS (ms)
    String timestamp     = String(baseTimeMs + realMs);          // converts type unsigned long to type string for getByte function         
  
    // Write the epoch time to block  
    MFRC522::StatusCode status;
    byte buffer[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    timestamp.getBytes(buffer,18);            //converts String to Byte and puts into buffer
    //byte block = dataBlocks[0];
    byte block = thisPod.block;

    //  Serial.println(F("Authenticating using key A for podID write..."));
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return;
    }

    //  Serial.println(F("Writing epoch time to PICC..."));
    status = mfrc522.MIFARE_Write(block, buffer, 16);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Write() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return;
    } else {
        setupAndBeep();
    }
    tagCount ++;
}

unsigned long flashCount = 0;
String getBatteryVoltage() {
    int sensorValue = analogRead(V_MEAS);          // read the input on analog pin A2 for battery voltage reading:
    //Serial.println(String(sensorValue));
    float voltage   = (float)sensorValue / 155;      // Convert the analog reading to a voltage adjusted to Pieters multimeter
    String batMsg   = "Good";
    if (voltage > BATT_GOOD)  {                   // If Voltage higher than 3.8 display "Good" on LCD
        batMsg    = "Good";
        flashLed  = LOW;
        battGood = 1;
    } else if ( voltage > BATT_LOW ) {            // If Voltage lower than 3.8 display "Turn off POD" on LCD and flash lights. POD will work until 3.5V but will stop working after that.
        batMsg = "OK";
        battGood = 1;
        if ( flashCount < sqCounter ){
            flashCount = sqCounter + 2000;            // set toggle time to 5s
            if ( flashLed == LOW ) {                 
              flashLed = HIGH;                        // toggle status led previouse state 
            } else {
               flashLed = LOW;
            }
        }
    } else {
        battGood = 0;
        if ( flashCount < sqCounter ){
            flashCount = sqCounter + 100;            // set toggle time to 1s
            if ( flashLed == LOW ) {                 
              flashLed = HIGH;                        // toggle status led previouse state 
            } else {
              flashLed = LOW;
            }
        }
        batMsg = "PowerOff";             
    }
    char volt[5];
    String messageBattery = dtostrf(voltage,1,2,volt);       // conver float to string
    //Serial.println(messageBattery);
    messageBattery        = messageBattery + "V " + batMsg;  // append status 
    digitalWrite(STATUS_LED, flashLed); 
    return messageBattery;
}

// Displays main Screen Time & Pod station ID
void updateLcdScreen()    {

    myGLCD.invert (false);
    // 1st line of display
    myGLCD.setFont(SmallFont);
    myGLCD.print((thisPod.podID),CENTER,0);             // Displays POD ID 

    if (battGood == 0) {
         myGLCD.setFont(SmallFont);
         myGLCD.print(("Battery Low"),CENTER,10);            

    } else {
    //2nd line  Display
    unsigned long realMs    = getAfricaTimeMs();       // read the ms time from int counter
    unsigned long ms        = realMs % 1000;               // seperate ms from secconds
    unsigned long realTimeS = baseTimeS + realMs/1000; // convert to sec for easy display
//    String msgTime =  "I " + String(hour(realTimeS)) + ":" + String (minute(realTimeS)) + ":" + String(second(realTimeS)) + "." + String(ms);  
//    Serial.println(msgTime);

    myGLCD.setFont(MediumNumbers);                      // Displays Clock in Medium Numbers
    myGLCD.printNumI(  hour(realTimeS),0,10,2,'0');          // Minimum number of characters is 2 with on '0' to full the space if minimum is not reached
    myGLCD.printNumI(minute(realTimeS),29,10,2,'0'); 
    myGLCD.printNumI(second(realTimeS),58,10,2,'0'); 
    
    myGLCD.setFont(MediumNumbers);          // Displays colons
    myGLCD.drawRect(55,15,57,17);
    myGLCD.drawRect(55,20,57,22);
    myGLCD.drawRect(26,15,28,17);
    myGLCD.drawRect(26,20,28,22);
    }
    
    // 3r Line
    myGLCD.setFont(SmallFont);
    int temp    = RTC.temperature();
    int mC      = ((temp*100) % 400 ) /100;
    String msg  = String(temp/4);
    msg         = msg + "." + mC + "C";
    myGLCD.print(msg,LEFT,30);     
    myGLCD.print("Tags: ",RIGHT,30);    
    myGLCD.print( String(tagCount),RIGHT,30);

    // 4th Line
    String messageBattery = getBatteryVoltage();  
    myGLCD.setFont(SmallFont);
    myGLCD.print(messageBattery,LEFT,40);

    myGLCD.update();
    myGLCD.clrScr();            
}


void setupAndBeep(){
   if ( thisPod.role == START_POD ) {
        int length = 4;
        int melody[length] = {NOTE_A7, NOTE_G5, NOTE_E6, NOTE_C8};        // notes in the melody:
        int noteDurations[length] = {8, 8, 8, 8};                        // note durations 4 = quarter note, 8 = eighth note, etc.::
        beepsLights ( length, melody, noteDurations );
    } else {
//        int length = 8;
//        int melody[length] = {NOTE_A7, NOTE_G5, NOTE_E6, NOTE_C8, NOTE_A7, NOTE_G5, NOTE_E6, NOTE_C8};        
//        int noteDurations[length] = {8, 8, 8, 8,8, 8, 8, 8};  
        int length = 4;
        int melody[length] = {NOTE_A7, NOTE_G5, NOTE_E6, NOTE_C8};        // notes in the melody:
        int noteDurations[length] = {8, 8, 8, 8};
        beepsLights ( length, melody, noteDurations );
    } 
}

// Beeps and flash leds for succesfull scan
void beepsLights(int length,int melody[],int noteDurations[]  )    {                                        
    for (int thisNote = 0; thisNote < length; thisNote++) {
        int noteDuration = 1000 / noteDurations[thisNote];      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        tone(BUZZER, melody[thisNote], noteDuration);
        digitalWrite(LED_TOP, LOW);                             // turn  LED on
        digitalWrite(LED_BOT, LOW);                             // turn  LED on

        int pauseBetweenNotes = noteDuration * 1.30;            // the note's duration + 30% seems to work well:
        delay(pauseBetweenNotes);
    
        noTone(BUZZER);                                         // stop the tone playing:
        digitalWrite(LED_TOP, HIGH);
        digitalWrite(LED_BOT, HIGH);
    }
}
/*
// Configuration functions - Only called once duting setup
*/

// These functions are to define the POD role and synck ms with seconds
void configrePod() {
    // read last pod type index from eeprom and select that identiry as default
    typeIndex = EEPROM.read(ADDRESS_POD_TYPE);
    unsigned long ConfigTimeoutMs = millis() + CONFIG_TIME_MS;
    unsigned long msLeft = CONFIG_TIME_MS;
    while ( millis() < ConfigTimeoutMs ) {
        // check for card read and incremetn typeIndex
        if ( mfrc522.PICC_IsNewCardPresent()) {    // Look for new cards
            typeIndex++;
        }
        if (typeIndex >= LIST_SIZE) {
            typeIndex = 0;                          // if uninitilized set to 0
        }
        thisPod = podTypeList[typeIndex];
        // display curretn pod id and start countdown to applying config
        //Serial.println("array size = " + sizeof(thisPod));
        msLeft = ConfigTimeoutMs - millis();
        configDisplay (msLeft );
        delay (100);
    }
    // save pod type in EEPROM for next boot
    EEPROM.write(ADDRESS_POD_TYPE, typeIndex);
}

// Display fucntion during config loop
void configDisplay( unsigned long timeLeftMs)   {
    myGLCD.invert (true);
     // 1st line of display
    myGLCD.setFont(SmallFont);
    String msg = VERSION;
    msg = msg + " Configure ";
    myGLCD.print(msg,LEFT,0);       

   //2nd line  Display
    myGLCD.setFont(SmallFont);          
    myGLCD.print("DONT TAG",RIGHT,10);        

    //3rd line  Display
    myGLCD.setFont(SmallFont);          
    myGLCD.print((thisPod.podID),CENTER,20);        
 
    // 4th Line
    myGLCD.setFont(SmallFont);
    myGLCD.print("Swipe2Change",CENTER,30);

    // 5th Line
    String timeMsg = "Done in ";
    unsigned long ms = (timeLeftMs % 1000)/100;
    timeMsg = timeMsg + (timeLeftMs/1000) + "." + ms + "s";
    myGLCD.setFont(SmallFont);
    myGLCD.print(timeMsg,LEFT,40);

    myGLCD.update();
    myGLCD.clrScr();
}

// All Genral Purpouse IO pin Setup stuffs 
void initIoPins() {
    // Input Pins
    pinMode(SQ_WAVE, INPUT);

    // Output Pins
    pinMode(LED_TOP, OUTPUT);
    pinMode(LED_BOT, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    // set pins start condition: LOW = LED on, HIGH = LED off
    digitalWrite(LED_TOP, HIGH);                       
    digitalWrite(LED_BOT, HIGH);                       
    digitalWrite(STATUS_LED, LOW);                     
}

// All RFID init Stuffs
void initMfrc522() {
    SPI.begin();                                          // Init SPI bus
    mfrc522.PCD_Init();                                   // Init MFRC522 card
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);       // Enhance the MFRC522 Receiver Gain to maximum value of some 48 dB (default is 33dB)

    for (byte i = 0; i < 6; i++) {                // Prepare the key (used both as key A and as key B) using FFFFFFFFFFFFh which is the default at chip delivery from the factory
        key.keyByte[i] = 0xFF;
    }
}

/* set up timer to run odd the interupt driven by the 1024Hz TRC square wave
 * Read a base time in seconds from RTC ( unix time) 
 * thre RTC second has started counting.\, assume ~250ms has passed - initilize counter.
 * from now use the interupt counters time to increment real time and provide a ms
 */
void initAfricaTimer(){
    setSyncProvider(RTC.get);                            // gets RTC time
    RTC.squareWave(SQWAVE_1024_HZ);                      // Set RTC Square wave frequany 

    // Set up interup
    *digitalPinToPCMSK(SQ_WAVE) |= bit (digitalPinToPCMSKbit(SQ_WAVE));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(SQ_WAVE)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(SQ_WAVE)); // enable interrupt for the group
    // Try allign with rtc tick ~ shuold be accurate withing 25ms
    baseTimeS = RTC.get();
    while ( RTC.get() < (baseTimeS +1)) {}; 
    sqCounter = 0;                                  // zero qsuare wave counter 

    // set base time to H:m:s of today
    baseTimeS           = RTC.get();                      
    String msgTime =  "R " + String(hour(baseTimeS)) + ":" + String (minute(baseTimeS)) + ":" + String(second(baseTimeS)) ;  
    Serial.println(msgTime);
    unsigned long secToday        = second(baseTimeS);
    unsigned long minToday        = minute(baseTimeS);
    unsigned long hourToday       = hour(baseTimeS);
    unsigned long today           = weekday(baseTimeS);
    msgTime =  "Today " + String(today) + " - " + String(hourToday) + ":" + String (minToday) + ":" + String(secToday) ;  
    Serial.println(msgTime);

    // Regester our base time now with only D H:m:s; 
    baseTimeS     =  (hourToday * 60 * 60) + (minToday * 60) + secToday;   
    //baseTimeS     =  (today * 24 * 60 * 60 ) + (hourToday * 60 * 60) + (minToday * 60) + secToday;           

    msgTime =  "I " + String(hour(baseTimeS)) + ":" + String (minute(baseTimeS)) + ":" + String(second(baseTimeS)) ;  
    Serial.println(msgTime);
    Serial.println("GO " + String(baseTimeS));
}
