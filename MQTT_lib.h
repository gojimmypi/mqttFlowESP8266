// MQTT_lib.h

#ifndef _MQTT_LIB_h
#define _MQTT_LIB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include "MQTT_lib.h"

int lastAdafruitMessageMillis = 0;



void MQTT_connect(Adafruit_MQTT_Client& thisMQTT);
void publishString(Adafruit_MQTT_Client& thisMQTT, char toUserName[128], char thisValue[],     char feedname[] = "history");
void publishValue(Adafruit_MQTT_Client& thisMQTT, char toUserName[128], char thisPinName[16], double thisValue, char feedname[] = "flow");
void publishHistoryValue(uint8_t thisDevice, double thisValue);


#endif

