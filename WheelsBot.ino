/*********************************************************************

Robot Control via Bluetooth

RadioShack Make: It Robotics
Adafruit Bluefruit SPI module

*********************************************************************/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <MakeItRobotics.h> 
#include "BlueFruitHelper.h"

// Bluefruit SPI SETTINGS
#define BLUEFRUIT_SPI_CS              10       
#define BLUEFRUIT_SPI_IRQ             9        
#define BLUEFRUIT_SPI_RST             8        
#define BLUEFRUIT_SPI_SCK             13
#define BLUEFRUIT_SPI_MISO            12
#define BLUEFRUIT_SPI_MOSI            11
#define MAX_SPEED                     255

#define VERBOSE_MODE                  true  // If set to 'true' enables debug output

// Define Functions
void stopbot(void);
void initBlueFruit(void);
void error(const __FlashStringHelper*err);

// Define Global Variable
uint8_t currentspeed = 0;               // Current speed of travel
uint8_t directionOfTravel;              // Current direction of travel
uint8_t currentCommand = 0;             // Current Action requested by button press
boolean isButtonPressed;                // Was a button pressed
BLEValues bleData;                      // Data returned from BLE read

// Create Objects
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
MakeItRobotics WheelsBot;             // Declare object from MakeItRobotics.h
SoftwareSerial DebugSerial(4, 5);     // Debug Serial Port


void setup(void) {
  initBlueFruit();
  DebugSerial.println(F("Initialise MakeItRobotics"));
  Serial.begin(10420);            //tell the Arduino to communicate with Make: it PCB
  delay(500);                     //delay 500ms
  WheelsBot.all_stop();               //all motors stop
  DebugSerial.println(F("Starting Main Loop"));

}

void loop(void) {

  memset(bleData.packetbuffer, 0, READ_BUFSIZE);

  /* Wait for new data to arrive */
  bleData = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // Check if any data returned
  if (bleData.replyidx == 0) 
    return;
  else if (bleData.packetbuffer[1] == 'B') {
    // Convert button press to in value
    currentCommand = bleData.packetbuffer[2] - '0';
    // Not if button press of release
    isButtonPressed = bleData.packetbuffer[3] - '0';
  } else {
    DebugSerial.println( "Invalid BLE Command! " );
  }


  /* Adafruit command button map
   5 up
   7 left
   6 down
   8 right
   1 - 4 
   */
   
  /* Motor References
   m1 - Front Left
   m2 - Front Right
   m3 - Rear Left
   m4 - Rear Right  
   */

  if(isButtonPressed == 1) {
    DebugSerial.print(F("New command: "));
    // Check to see if Speed change is requested 
    // If Speed change is requested, update speed and set command to the current 
    // direction of travel
    if (currentCommand == 2) {
      // Speed up
      DebugSerial.print( "Faster " );

      if (currentspeed+25 < MAX_SPEED) {
        currentspeed = currentspeed + 25;
        DebugSerial.println( currentspeed );
        currentCommand = directionOfTravel;

      } else {
        DebugSerial.println( "Max Speed" );
      }

    } else if (currentCommand == 4) {
      // Slow Down
      DebugSerial.print( "Slower " );

      if (currentspeed >= 25) {
        currentspeed = currentspeed - 25;
        DebugSerial.println( currentspeed );
        currentCommand = directionOfTravel;

      } else {
        // Stop the Bot 
        directionOfTravel = 1;

      }
    }


    // Check command to commit motion to requested
    // Start Forward motion
    if (currentCommand == 5 ) {
      if (currentspeed == 0)
        currentspeed = 50;

      WheelsBot.m1_action(FW, currentspeed);
      WheelsBot.m2_action(FW, currentspeed);
      WheelsBot.m3_action(FW, currentspeed);
      WheelsBot.m4_action(FW, currentspeed);

      DebugSerial.print( "Forward: " );
      DebugSerial.println( currentspeed );

      directionOfTravel = 5;

    } else if (currentCommand == 7) {
      // Left Turn
      WheelsBot.m1_action(BW, currentspeed);
      WheelsBot.m2_action(FW, currentspeed);
      WheelsBot.m3_action(BW, currentspeed);
      WheelsBot.m4_action(FW, currentspeed);

      DebugSerial.print( "Left: " );
      DebugSerial.println( currentspeed );

      directionOfTravel = 7;


    } else if (currentCommand == 8) {
      // Right Turn
      WheelsBot.m1_action(FW, currentspeed);
      WheelsBot.m2_action(BW, currentspeed);
      WheelsBot.m3_action(FW, currentspeed);
      WheelsBot.m4_action(BW, currentspeed);

      DebugSerial.print( "Right: " );
      DebugSerial.println( currentspeed );

      directionOfTravel = 8;


    } else if (currentCommand == 6 ) {
     // Backwards
     if (currentspeed == 0)
        currentspeed = 50;
      WheelsBot.m1_action(BW, currentspeed);
      WheelsBot.m2_action(BW, currentspeed);
      WheelsBot.m3_action(BW, currentspeed);
      WheelsBot.m4_action(BW, currentspeed);

      DebugSerial.print( "Backwards: " );
      DebugSerial.println( currentspeed );

      directionOfTravel = 6;

    } else if (currentCommand == 1) {
      stopbot();
      directionOfTravel = 0;

    } else if (currentCommand == 0) {
      DebugSerial.println( "Stopped" );

    } else {
      DebugSerial.println( "Invalid Command! " );
    }
  }
}

void stopbot(void) {
  // All stop
  WheelsBot.all_stop();
  currentspeed = 0;
  DebugSerial.println( "Stop " );
}


void initBlueFruit(void) {
  DebugSerial.begin(115200);
  DebugSerial.println(F("Adafruit Bluefruit App Controller Example"));
  DebugSerial.println(F("-----------------------------------------"));

  /* Initialise the module */
  DebugSerial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));

  DebugSerial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ) {
    /* Perform a factory reset to make sure everything is in a known state */
    DebugSerial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  DebugSerial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  DebugSerial.println(F("Use Adafruit Bluefruit LE app to connect in Controller mode"));
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }
  DebugSerial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    // Change Mode LED Activity
    DebugSerial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  DebugSerial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  DebugSerial.println(F("******************************"));
  DebugSerial.end();
}

void error(const __FlashStringHelper*err) {
  DebugSerial.println(err);
  while (1);
}

