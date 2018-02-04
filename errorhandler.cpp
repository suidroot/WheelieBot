#include "errorhandler.h"


SoftwareSerial DebugSerial(4, 5);     // Debug Serial Port

void initLogging(void) {
	  DebugSerial.begin(115200);
	  logger_char("Logging initalized!");
}

void logger_char(const char * message) {
	  DebugSerial.println(message);
}

void logger_uint8(uint8_t message) {
	  DebugSerial.println(message);
}

void error(const __FlashStringHelper*err) {
  DebugSerial.println(err);
  while (1);
}