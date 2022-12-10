#include <SPI.h>
#include <MFRC522.h>
#include "MFRC522_func.h"
// #include "SoftwareSerial.h"
// #include "DFRobotDFPlayerMini.h"
#include <FastLED.h>

// logic
#define MY_LEVEL 0
#define EVENT_ID_BIT 3 // burnerot 2022 is bit 3, count starts at 0
byte lastReadLevel = 0;
byte currWriteLevel = 0;

// RFID
#define SS_PIN 10
#define RST_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN); // Instance of the class

MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[4];
byte blockAddr      = 4;
byte buffer[18];
byte size = sizeof(buffer);
byte trailerBlock   = 7;

// sound
// SoftwareSerial mySoftwareSerial(2, 3); // RX, TX
// DFRobotDFPlayerMini myDFPlayer;
// #define VOLUME 15

// LEDs
#define DATA_PIN 3
#define NUM_LEDS 6
#define BRIGHTNESS 50

// Define the array of leds
CRGB leds[NUM_LEDS];

// Switch & Button
#define BUTTON_PIN 4
#define SWITCH_PIN 5
#define DEBOUNCE_DELAY 50
#define READ_MODE LOW
#define WRITE_MODE HIGH
uint8_t switchState = READ_MODE;
uint8_t buttonState = HIGH;
uint8_t lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = DEBOUNCE_DELAY;
uint8_t reading;


void setup() { 

  Serial.begin(115200);

  // RFID
  SPI.begin(); // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522
  //mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  Serial.print("Station level is: "); Serial.println(MY_LEVEL);

  // Sound
  // Serial.println(F("DFRobot DFPlayer Mini Demo"));
  // Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  // mySoftwareSerial.begin(9600);
  // if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
  //   Serial.println(F("Unable to begin DFPlayer Mini:"));
  //   Serial.println(F("1.Please recheck the connection!"));
  //   Serial.println(F("2.Please insert the SD card!"));
  //   Serial.println(F("Continuing with no sound..."));
  // } else {
  //   Serial.println(F("DFPlayer Mini online."));
  //   myDFPlayer.volume(VOLUME);  //Set volume value. From 0 to 30
  // }

  // LEDs
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  Serial.print("LED Brightness set to: "); Serial.print(BRIGHTNESS); Serial.println(" out of 255.");

  // Switch & Button
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.print("Read/Write switch set to pin: "); Serial.println(SWITCH_PIN);
  // Read/Write switch reading
  switchState = digitalRead(SWITCH_PIN);
  Serial.print("Read/Write switch is currently set to: "); 
  if (switchState == READ_MODE) {
    Serial.println("READ_MODE");
  } else {
    Serial.println("WRITE_MODE");
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.print("Level button set to pin: "); Serial.println(BUTTON_PIN);
}
 
void loop() {
  // Read/Write switch reading
  switchState = digitalRead(SWITCH_PIN);

  // Key switch reading with debounce
  reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // Logic for Read mode - ignore button, display last read level
  if (switchState == READ_MODE) {
    FastLED.clear();
    fill_solid(leds, lastReadLevel, CRGB::Blue);
    FastLED.show();
  } else if (switchState == WRITE_MODE) {
    // Logic for Write mode - read button with debounce, display current write level
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
        // only advance current level at one edge of the button press
        if (buttonState == LOW) {
          currWriteLevel++;
          if (currWriteLevel > NUM_LEDS) {
            currWriteLevel = 0;
          }
        }
      }
    }
    FastLED.clear();
    fill_solid(leds, currWriteLevel, CRGB::Green);
    FastLED.show();
  }
  lastButtonState = reading;


  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been read
  if ( ! mfrc522.PICC_ReadCardSerial())
    return;

  // perform authentication to open communication
  bool auth_success = authenticate(trailerBlock, &key, mfrc522);
  if (!auth_success) {
    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
    return;
  }

  // read the tag to get coded information to buffer
  bool read_success = read_block(blockAddr, buffer, size, mfrc522);
  if (!read_success) {
    // Serial.println(F("Initial read failed, closing connection"));
    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
    return;
  }

  byte color = *(buffer + 0); // byte 0 for color encoding
  if(color == 0xff) {
    color = 0x4; // set uninitialized color to 0x4
  }
  byte level = *(buffer + 1); // byte 1 for level encoding
  if(level == 0xff) {
    level = 0;
  }
  byte eventTrack = *(buffer + 15); // byte 15 for event track encoding bit[0] = burnerot2018, bit[1] = contra2019, bit[2] = midburn2022, bit[3] = burnerot2022
  if(eventTrack == 0xff) {
    eventTrack = 0x8; // set uninitialized eventTrack to 0x8, bit[3] = burnerot2022
  }
  Serial.print("Current chip color: "); Serial.println(color);
  Serial.print("Current chip level: "); Serial.println(level);
  Serial.print("Current chip eventTrack: "); Serial.println(eventTrack);

  if(switchState == WRITE_MODE) {
    eventTrack |= (1 << EVENT_ID_BIT);
    byte dataBlock[] = {
      color, currWriteLevel, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 
      0x00, 0x00, 0x00, 0x00, 
      0x00, 0x00, 0x00, eventTrack  // byte 15 for event track bit[0] = burnerot2018, bit[1] = contra2019, bit[2] = Midburn2022, bit[3] = burnerot2022
    };

    bool write_success = write_and_verify(blockAddr, dataBlock, buffer, size, mfrc522);
    if(!write_success) {
      // Failed write light the LEDs RED shortly
      FastLED.clear();
      fill_solid(leds, currWriteLevel, CRGB::Red);
      FastLED.show();
      Serial.println(F("write failed"));
      delay(1000); // hold the fail display for a little
      // Halt PICC
      mfrc522.PICC_HaltA();
      // Stop encryption on PCD
      mfrc522.PCD_StopCrypto1();
      return;
    } else {
      Serial.print("write successful - new chip level: "); Serial.println(currWriteLevel);
    }
  } else if (switchState == READ_MODE) {
    lastReadLevel = level;
  }

  // Halt PICC
  mfrc522.PICC_HaltA();

  // Stop encryption on PCD
  mfrc522.PCD_StopCrypto1();

  // visual indication for a successful operation clear the LEDs so they can light up at next loop
  FastLED.clear();
  FastLED.show();
      
  // hold everything in place for some time so we don't accidentally read the same chip again
  delay(200);
}

