#include <SoftwareSerial.h>

void error(const __FlashStringHelper*err);
void logger_char(const char * message);
void logger_uint8(uint8_t message);
void initLogging(void);
