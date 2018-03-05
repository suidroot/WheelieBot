/*********************************************************************

Robot Control via Bluetooth

RadioShack Make: It Robotics
Adafruit Bluefruit SPI module

*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
// #include "Adafruit_BluefruitLE_UART.h"
#include <MakeItRobotics.h> //include library

#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT         500   // Timeout in ms waiting to read a response

// SPI SETTINGS
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

// Declare robot object from MakeItRobotics.h
MakeItRobotics line_following; 

// ********** Declare Bluetooth object ***************
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
// the packet buffer
extern uint8_t packetbuffer[];

// Declare current speed
int currentspeed = 0;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  Serial.println("Requesting Bluefruit info:");

  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("******************************"));
  Serial.end();
  Serial.println(F("Change to MakeItRobotics"));

  Serial.begin(10420);                     //tell the Arduino to communicate with Make: it PCB
  delay(500);                              //delay 500ms
  line_following.all_stop();               //all motors stop

}


void loop(void)
{

  // Variable track current to previous actions
  static int action1 = 0;                //now action
  static int action2 = 0;                //last action

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  if (len == 0) return;

  // printHex(packetbuffer, len);
  // Convert button press to in value
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    action1 = buttnum;
  }

  // Adafruit command button
  // 5 up
  // 7 left
  // 6 down
  // 8 right
  // 1 - 4 
  if(action1 != action2)
  {
    // Start Forward motion
    if (action1 == 5 ) {
      if (currentspeed == 0)
        currentspeed = 25;
      line_following.go_forward(currentspeed);
    }
    // Left Turn
    if (action1 == 7)
      //line_following.line_following_turn_left(currentspeed);    
      line_following.turn_left(currentspeed);    
    // Right Turn
    if (action1 == 8)
      //line_following.line_following_turn_right(currentspeed);
      line_following.turn_right(currentspeed);
    // Backwards
    if (action1 == 6 ) {
      if (currentspeed == 0)
        currentspeed = 25;
      line_following.go_backward(currentspeed);
    }
      
    // Speed up
    if (action1 == 2)
      if (currentspeed < 250) {
        currentspeed = currentspeed + 25;
      } 
    // Slow Down
    if (action1 == 4)
     if (currentspeed > 10) {
        currentspeed = currentspeed - 25;
      } 
    // All stop
    if (action1 == 1) {
      line_following.move_stop();
      currentspeed = 0;
    }
  }
  action2=action1;  
}

/* ************** Packet Parser (packetParser.cpp) **************
 *
 * from Adafruit_BluefruitLE_nRF51 libraries
 * 
 */

#define PACKET_ACC_LEN                  (15)
#define PACKET_GYRO_LEN                 (15)
#define PACKET_MAG_LEN                  (15)
#define PACKET_QUAT_LEN                 (19)
#define PACKET_BUTTON_LEN               (5)
#define PACKET_COLOR_LEN                (6)
#define PACKET_LOCATION_LEN             (15)

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)


/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE+1];

/**************************************************************************/
/*!
    @brief  Casts the four bytes at the specified address to a float
*/
/**************************************************************************/
float parsefloat(uint8_t *buffer) 
{
  float f;
  memcpy(&f, buffer, 4);
  return f;
}

/**************************************************************************/
/*! 
    @brief  Prints a hexadecimal value in plain characters
    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void printHex(const uint8_t * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++) 
  {
    Serial.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
    {
      Serial.print(F("0"));
      Serial.print(data[szPos] & 0xf, HEX);
    }
    else
    {
      Serial.print(data[szPos] & 0xff, HEX);
    }
    // Add a trailing space if appropriate
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      Serial.print(F(" "));
    }
  }
  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout) 
{
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
    Serial.print("Checksum mismatch in packet : ");
    printHex(packetbuffer, replyidx+1);
    return 0;
  }
  
  // checksum passed!
  return replyidx;
}

