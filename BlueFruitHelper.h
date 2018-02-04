
/*

  Helper functions and static definitaions used when interfacing with data from the 
  Adafruit Bluefruit board

*/
#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include "errorhandler.h"


// Bluetooth interface definitions
#define FACTORYRESET_ENABLE           0
#define MINIMUM_FIRMWARE_VERSION      "0.6.6"
#define MODE_LED_BEHAVIOUR            "MODE"
#define BUFSIZE                       128   // Size of the read buffer for incoming data
#define BLE_READPACKET_TIMEOUT        500   // Timeout in ms waiting to read a response
// readPacket function defines
#define PACKET_ACC_LEN                (15)
#define PACKET_GYRO_LEN               (15)
#define PACKET_MAG_LEN                (15)
#define PACKET_QUAT_LEN               (19)
#define PACKET_BUTTON_LEN             (5)
#define PACKET_COLOR_LEN              (6)
#define PACKET_LOCATION_LEN           (15)
#define READ_BUFSIZE                  (20)  // READ_BUFSIZE - Size of the read buffer for incoming packets

// Type for Data Returned from readPacket function
struct BLEValues {
	uint8_t packetbuffer[READ_BUFSIZE+1];
	uint16_t replyidx;
};

// Function prototypes 
BLEValues readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
