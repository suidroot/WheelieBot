
// #include <string.h>
// #include <Arduino.h>
// #include <SPI.h>
// #include <Adafruit_BLE.h>
// #include <Adafruit_BluefruitLE_SPI.h>
#include "BlueFruitHelper.h"


float parsefloat(uint8_t *buffer) {
  float f;
  memcpy(&f, buffer, 4);
  return f;
}

void printHex(const uint8_t * data, const uint32_t numBytes) {
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    //DebugSerial.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
    {
      //DebugSerial.print(F("0"));
      //DebugSerial.print(data[szPos] & 0xf, HEX);
    }
    else
    {
      //DebugSerial.print(data[szPos] & 0xff, HEX);
    }
    // Add a trailing space if appropriate
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      //DebugSerial.print(F(" "));
    }
  }
  //DebugSerial.println();
}

BLEValues readPacket(Adafruit_BLE *ble, uint16_t timeout) {

  uint16_t origtimeout = timeout, replyidx = 0;
  BLEValues packetValues;
  memset(packetValues.packetbuffer, 0, READ_BUFSIZE);

  while (timeout--) {
    if (replyidx >= 20) break;
    if ((packetValues.packetbuffer[1] == 'A') && (replyidx == PACKET_ACC_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'G') && (replyidx == PACKET_GYRO_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'M') && (replyidx == PACKET_MAG_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'Q') && (replyidx == PACKET_QUAT_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'B') && (replyidx == PACKET_BUTTON_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'C') && (replyidx == PACKET_COLOR_LEN))
      break;
    if ((packetValues.packetbuffer[1] == 'L') && (replyidx == PACKET_LOCATION_LEN))
      break;

    while (ble->available()) {
      char c = ble->read();
      if (c == '!') {
        replyidx = 0;
      }
      packetValues.packetbuffer[replyidx] = c;
      replyidx++;
      timeout = origtimeout;
    }

    if (timeout == 0) break;
    delay(1);
  }

  packetValues.packetbuffer[replyidx] = 0;  // null term

  if (!replyidx) { // no data or timeout
    packetValues.replyidx = 0;
    return packetValues;
  }
  if (packetValues.packetbuffer[0] != '!') { // doesn't start with '!' packet beginning
    packetValues.replyidx = 0;
    return packetValues;
  }
  // check checksum!
  uint8_t xsum = 0;
  uint8_t checksum = packetValues.packetbuffer[replyidx-1];

  for (uint8_t i=0; i<replyidx-1; i++) {
    xsum += packetValues.packetbuffer[i];
  }
  xsum = ~xsum;

  // Throw an error message if the checksum's don't match
  if (xsum != checksum)
  {
    //DebugSerial.print("Checksum mismatch in packet : ");
    printHex(packetValues.packetbuffer, replyidx+1);
    packetValues.replyidx = 0;
    
    return packetValues;
  }

  packetValues.replyidx = replyidx;

  // checksum passed! Return index and data
  return packetValues;
}
