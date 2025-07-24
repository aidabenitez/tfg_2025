#ifndef SERIAL_UTILS_H
#define SERIAL_UTILS_H

#include <Arduino.h>

void setupSerial();
String readMessage();
void sendMessage(String message);
int calculateChecksum(String message);
bool verifyChecksum(String message);

#endif
