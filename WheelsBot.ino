/*********************************************************************

Robot Control via Bluetooth

RadioShack Make: It Robotics
Adafruit Bluefruit SPI module

*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <MakeItRobotics.h> 
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>

// ***** BlueFruit Defines *****
// ********** Declare Bluetooth object ***************
#define FACTORYRESET_ENABLE           0
#define MINIMUM_FIRMWARE_VERSION      "0.6.6"
#define MODE_LED_BEHAVIOUR            "MODE"
#define BUFSIZE                       128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                  true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT        500   // Timeout in ms waiting to read a response
// Bluefruit SPI SETTINGS
#define BLUEFRUIT_SPI_CS              10       
#define BLUEFRUIT_SPI_IRQ             9        
#define BLUEFRUIT_SPI_RST             8        
#define BLUEFRUIT_SPI_SCK             13
#define BLUEFRUIT_SPI_MISO            12
#define BLUEFRUIT_SPI_MOSI            11
// readPacket function defines
#define PACKET_ACC_LEN                (15)
#define PACKET_GYRO_LEN               (15)
#define PACKET_MAG_LEN                (15)
#define PACKET_QUAT_LEN               (19)
#define PACKET_BUTTON_LEN             (5)
#define PACKET_COLOR_LEN              (6)
#define PACKET_LOCATION_LEN           (15)
#define READ_BUFSIZE                  (20)  // READ_BUFSIZE - Size of the read buffer for incoming packets
// ***** END Bluefruit defines *****

#define MAX_SPEED                     255

// function prototypes 
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
void error(const __FlashStringHelper*err);
void initBlueFruit(void);
void stopbot(void);

/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE+1];

// Variable track current to previous actions
uint8_t currentaction = 0;              // Current action
uint8_t previousaction = 0;             // Previous action
uint8_t currentspeed = 0;               // Declare current speed
boolean pressed;

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
MakeItRobotics robot;             // Declare robot object from MakeItRobotics.h
SoftwareSerial DebugSerial(4, 5); // Debug Serial Port

void setup(void) {
  initBlueFruit();
  DebugSerial.println(F("Change to MakeItRobotics"));
  Serial.begin(10420);            //tell the Arduino to communicate with Make: it PCB
  delay(500);                     //delay 500ms
  robot.all_stop();               //all motors stop
  DebugSerial.println(F("Starting Main Loop"));
}


void loop(void) {
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // Check if any data returned
  if (len == 0) 
    return;
  else if (packetbuffer[1] == 'B') {
    // Convert button press to in value
    currentaction = packetbuffer[2] - '0';
    // Not if button press of release
    pressed = packetbuffer[3] - '0';
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

  if(currentaction != previousaction && pressed == 1) {
    DebugSerial.print(F("New command: "));

    // Start Forward motion
    if (currentaction == 5 ) {
      if (currentspeed == 0)
        currentspeed = 50;

      robot.m1_action(FW, currentspeed);
      robot.m2_action(FW, currentspeed);
      robot.m3_action(FW, currentspeed);
      robot.m4_action(FW, currentspeed);

      DebugSerial.print( "Forward " );
      DebugSerial.println( currentspeed );

    } else if (currentaction == 7) {
      // Left Turn
      robot.m1_action(BW, currentspeed);
      robot.m2_action(FW, currentspeed);
      robot.m3_action(BW, currentspeed);
      robot.m4_action(FW, currentspeed);

      DebugSerial.print( "Left " );
      DebugSerial.println( currentspeed );

    } else if (currentaction == 8) {
      // Right Turn
      robot.m1_action(FW, currentspeed);
      robot.m2_action(BW, currentspeed);
      robot.m3_action(FW, currentspeed);
      robot.m4_action(BW, currentspeed);

      DebugSerial.print( "Right " );
      DebugSerial.println( currentspeed );

    } else if (currentaction == 6 ) {
     // Backwards
     if (currentspeed == 0)
        currentspeed = 50;
      robot.m1_action(BW, currentspeed);
      robot.m2_action(BW, currentspeed);
      robot.m3_action(BW, currentspeed);
      robot.m4_action(BW, currentspeed);

      DebugSerial.print( "Backwards " );
      DebugSerial.println( currentspeed );

    } else if (currentaction == 2) {
      // Speed up
      DebugSerial.print( "Faster " );

      if (currentspeed+25 < MAX_SPEED) {
        currentspeed = currentspeed + 25;
        DebugSerial.println( currentspeed );
      } else {
        DebugSerial.println( "Max Speed" );
      }
      currentaction=0;

    } else if (currentaction == 4) {
      // Slow Down
      DebugSerial.print( "Slower " );

      if (currentspeed >= 25) {
        currentspeed = currentspeed - 25;
        DebugSerial.println( currentspeed );
      }
      else {
        stopbot();
      }
      currentaction=0;

    } else if (currentaction == 1) {
      stopbot();
    } else {
      DebugSerial.println( "Invalid Command! " );
      currentaction=previousaction;
    }
  }
  previousaction=currentaction;
  // delay(50);
}

void stopbot(void) {
  // All stop
  robot.all_stop();
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

  DebugSerial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  DebugSerial.println(F("Then activate game controller"));

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


float parsefloat(uint8_t *buffer) {
  float f;
  memcpy(&f, buffer, 4);
  return f;
}

void printHex(const uint8_t * data, const uint32_t numBytes) {
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    DebugSerial.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
    {
      DebugSerial.print(F("0"));
      DebugSerial.print(data[szPos] & 0xf, HEX);
    }
    else
    {
      DebugSerial.print(data[szPos] & 0xff, HEX);
    }
    // Add a trailing space if appropriate
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      DebugSerial.print(F(" "));
    }
  }
  DebugSerial.println();
}

uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout) {

  uint16_t origtimeout = timeout, replyidx = 0;

  memset(packetbuffer, 0, READ_BUFSIZE);

  while (timeout--) {
    if (replyidx >= 20) break;
    if ((packetbuffer[1] == 'A') && (replyidx == PACKET_ACC_LEN))
      break;
    if ((packetbuffer[1] == 'G') && (replyidx == PACKET_GYRO_LEN))
      break;
    if ((packetbuffer[1] == 'M') && (replyidx == PACKET_MAG_LEN))
      break;
    if ((packetbuffer[1] == 'Q') && (replyidx == PACKET_QUAT_LEN))
      break;
    if ((packetbuffer[1] == 'B') && (replyidx == PACKET_BUTTON_LEN))
      break;
    if ((packetbuffer[1] == 'C') && (replyidx == PACKET_COLOR_LEN))
      break;
    if ((packetbuffer[1] == 'L') && (replyidx == PACKET_LOCATION_LEN))
      break;

    while (ble->available()) {
      char c =  ble->read();
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
      timeout = origtimeout;
    }

    if (timeout == 0) break;
    delay(1);
  }

  packetbuffer[replyidx] = 0;  // null term

  if (!replyidx)  // no data or timeout
    return 0;
  if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;

  // check checksum!
  uint8_t xsum = 0;
  uint8_t checksum = packetbuffer[replyidx-1];

  for (uint8_t i=0; i<replyidx-1; i++) {
    xsum += packetbuffer[i];
  }
  xsum = ~xsum;

  // Throw an error message if the checksum's don't match
  if (xsum != checksum)
  {
    DebugSerial.print("Checksum mismatch in packet : ");
    printHex(packetbuffer, replyidx+1);
    return 0;
  }

  // checksum passed!
  return replyidx;
}
