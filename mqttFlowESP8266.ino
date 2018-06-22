#define ENABLE_MQTT="TRUE"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> #OTA
#include <WiFiUdp.h>     #OTA
#include <ArduinoOTA.h>  #OTA

#ifdef ENABLE_MQTT
	#include <AdafruitIO_Definitions.h>
	#include <Adafruit_MQTT.h>
	#include <Adafruit_MQTT_Client.h>
#endif // ENABLE_MQTT

#include <pins_arduino.h> // be sure to select "NodeMCU Board!" see https://github.com/esp8266/Arduino/tree/master/variants/nodemcu

// https://raw.githubusercontent.com/esp8266/Arduino/master/variants/nodemcu/pins_arduino.h


// My config is stored in myPrivateSettings.h file 
// if you choose not to use such a file, set this to false:
#define USE_myPrivateSettings true

const char* WIFI_SSID = "SSID";
const char* WIFI_PWD = "password";



// #define AIO_SSL_FINGERPRINT "26 96 1C 2A 51 07 FD 15 80 96 93 AE F7 32 CE B9 0D 01 55 C4"
// Note the two possible file name string formats.
#if USE_myPrivateSettings == true 
	#include "c:\workspace\myPrivateSettings.h"
#else
// create your own myPrivateSettings.h, or uncomment the following lines:
const char* WIFI_SSID = "my-wifi-SSID"
const char* WIFI_PWD = "my-WiFi-PASSWORD"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883                   // 8883 for MQTTS
#define AIO_USERNAME    "your-adafruit-username"
#define AIO_KEY         "your-adafrui-key"

// io.adafruit.com SHA1 fingerprint
const char* fingerprint = "your adafruit fingerprint string";
#endif


const int ESP_BUILTIN_LED = 2;

//connect a jumper from GPIO 14 to ground to start test

#include "MQTT_lib.h"

WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);



// https://io.adafruit.com/gojimmypie/mydashboard#
// https://learn.adafruit.com/adafruit-huzzah-esp8266-breakout/using-nodemcu-lua
// https://github.com/adafruit/Adafruit_MQTT_Library
// https://learn.adafruit.com/mqtt-adafruit-io-and-you/arduino-plus-library-setup
// https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/readme.md#client-secure
// https://github.com/esp8266/source-code-examples/blob/master/interrupt_example/user/user_main.c
// https://www.arduino.cc/en/Reference/AttachInterrupt
// https://abzman2k.wordpress.com/2015/09/24/esp8266-flow-meters/
// https://halckemy.s3.amazonaws.com/uploads/image_file/file/99209/screenshot-from-2015-09-07-220915.png 

// I2C http://www.esp8266.com/viewtopic.php?f=29&t=5564

// C:\Users\ [your login] \AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\tools\sdk

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
unsigned long millisBetween(unsigned long millisStart, unsigned long millisEnd);


#define VerboseSerial true
#define GPIO_PIN 14 

// Pin interrupts are supported through attachInterrupt(), detachInterrupt() functions. 
// Interrupts may be attached to any GPIO pin except GPIO16, but since GPIO6-GPIO11 are 
// typically used to interface with the flash memory ICs on most esp8266 modules, 
// applying interrupts to these pins are likely to cause problems. Standard Arduino 
// interrupt types are supported: CHANGE, RISING, FALLING. 
//
// see https://github.com/esp8266/Arduino/tree/master/variants/nodemcu for these definitions:
//
// static const uint8_t SDA = 4;
// static const uint8_t SCL = 5;

// static const uint8_t LED_BUILTIN = 16; // these seem to be wrong!
// static const uint8_t BUILTIN_LED = 16; // these seem to be wrong!
static const uint8_t NODEMCU_LED = 2;     // our LED (on same pin as Tx, is 2)

										  //                     Pin    GPIO
										   //static const uint8_t D0   = 16; // deep sleep wake-up
										   //static const uint8_t D1   = 5;  //
										   //static const uint8_t D2   = 4;  //
										   //static const uint8_t D3   = 0;  // 
										   //static const uint8_t D4   = 2;  // GPIO2 (Pin 4) is LED on our NODEMCU board
										   //static const uint8_t D5   = 14; // SCLK confirmed device #1
										   //static const uint8_t D6   = 12; // MISO
										   //static const uint8_t D7   = 13; // MOSI
										   //static const uint8_t D8   = 15; // SS
										   //static const uint8_t D9   = 3;  // Rx0
										   //static const uint8_t D10  = 1;  // 
										  // do not use GPIO6,7,8,9,10,11 as they are used internally! (SD_DATA)
uint8_t led = HIGH;

static const unsigned long maxUnsignedLongValue = (0 - 1); // just in case platform changes, calculate max unsigned long value

static const uint8_t MAX_DEVICES = 16;
static const uint8_t MAX_HISTORY = 60;

static char pinName[MAX_DEVICES][3]; // an array of pin names (e.g. pinName[D5] = "D5", where perhaps D5 = 14; GPIO #14 having GPIO 14 on board pin D5)

									 //********************************************************************************************************************
									 // volatile variables used in Interrupt Service Routines and main loop.
									 //********************************************************************************************************************
volatile unsigned long isrCountArray[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // count of interrupts (pulses)
volatile unsigned long timeInterruptEvent[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // time of interrupt; expect overflow every 50 days; 0 to 4,294,967,295 (2^32 - 1); see maxUnsignedLongValue
volatile unsigned long timeLastInterruptEvent[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // time of prior interrupt
volatile unsigned long timeDelta[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // time between most recent pulses (Period)
volatile unsigned long timeLastDelta[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // time between most recent pulses  (Prior Seen Period)
																										//

																										//********************************************************************************************************************
																										//
																										//********************************************************************************************************************
unsigned long timeSince[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Per second details
unsigned long timePerSecondRef[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the starting count for 1000ms-period reference
unsigned int  timeSecondDelta[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // keeping tracking of milliseconds in a second
unsigned int  countPerSecondRef[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the starting count for second-period reference
unsigned int  countPerSecond[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // pulses per second

																								// Per minute details                          
unsigned long timePerMinuteRef[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the starting count for Minute-period reference
unsigned int  timeMinuteDelta[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // keeping tracking of milliseconds in a Minute
unsigned int  countPerMinuteRef[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the starting count for Minutend-period reference
unsigned int  countPerMinute[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // pulses per Minute

																								// A single "flow" is the total number of pulses (interrupts) from first seeing a flow, until it stops. (e.g. a 5 minute sprinkler session)
unsigned long countThisFlowRef[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the starting count for total flow for this session
unsigned int  countThisFlow[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the total pulses for this session
unsigned int  countLastFlow[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // the total pulses for previous flow session

																							   // Store the per-second counts in a history table; we'll average over total time when time to display data
int           countHistoryIndex[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int           countFlowHistory[MAX_DEVICES][MAX_HISTORY]; // per second flow rate for all devices

bool deviceIsInitialized[MAX_DEVICES] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
bool deviceActiveFlow[MAX_DEVICES] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
unsigned long countPerMinute10[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long countPerHour[MAX_DEVICES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };




//********************************************************************************************************************
// See http://gammon.com.au/interrupts
//
// Here all of our interrupt services are the same, but apply to different devices
// note how the pin number is also used in the array. perhaps not the most efficient use
// of array space, but quite handy for coding.
//********************************************************************************************************************
void InterruptService(uint8_t thisDevice) {
	timeLastInterruptEvent[thisDevice] = timeInterruptEvent[thisDevice];
	timeInterruptEvent[thisDevice] = millis();

	timeLastDelta[thisDevice] = timeDelta[thisDevice];
	timeDelta[thisDevice] = millisBetween(timeLastInterruptEvent[thisDevice], timeInterruptEvent[thisDevice]);

	isrCountArray[thisDevice] ++;
}

//********************************************************************************************************************
// 
//********************************************************************************************************************
void InterruptServiceD6() {
	InterruptService(D6);
}

//********************************************************************************************************************
// 
//********************************************************************************************************************
void InterruptServiceD5() {
	InterruptService(D5);
}

//********************************************************************************************************************
// for NODEMCU board, blinking the LED will trigger the interrupt! (great for testing, not so great in practice)
//********************************************************************************************************************
void InterruptServiceD4() {
	InterruptService(D4);
}

//********************************************************************************************************************
// 
//********************************************************************************************************************
void InterruptServiceD1() {
	InterruptService(D1);
}

//********************************************************************************************************************
// InterruptServiceD0 for NODEMCU board, this is deep sleep
//********************************************************************************************************************
void InterruptServiceD0() {
	InterruptService(D0);
}

//********************************************************************************************************************
// initDevice
//********************************************************************************************************************
void initDevice(uint8_t thisDevice) {
	cli(); // no interrupts during init 
	timeInterruptEvent[thisDevice] = millis();
	timeLastInterruptEvent[thisDevice] = timeInterruptEvent[thisDevice];

	timePerSecondRef[thisDevice] = timeInterruptEvent[thisDevice];
	timePerMinuteRef[thisDevice] = timeInterruptEvent[thisDevice];

	pinMode(thisDevice, INPUT);
	if (thisDevice == D5) {
		attachInterrupt(digitalPinToInterrupt(D5), InterruptServiceD5, RISING);
		deviceIsInitialized[thisDevice] = true;
	}
	if (thisDevice == D4) {
		attachInterrupt(digitalPinToInterrupt(D4), InterruptServiceD4, RISING);
		deviceIsInitialized[thisDevice] = true;
	}
	if (thisDevice == D1) {
		attachInterrupt(digitalPinToInterrupt(D1), InterruptServiceD1, RISING);
		deviceIsInitialized[thisDevice] = true;
	}
	if (thisDevice == D0) {
		attachInterrupt(digitalPinToInterrupt(D0), InterruptServiceD0, RISING);
		deviceIsInitialized[thisDevice] = true;
	}
	sei(); // enable interrupts
}

//********************************************************************************************************************
//  clearDeviceHistory
//********************************************************************************************************************
void clearDeviceHistory(uint8_t thisDevice) {
	int thisHistoryItem;
	for (thisHistoryItem = 0; thisHistoryItem < MAX_HISTORY; thisHistoryItem++) {
		countFlowHistory[thisDevice][thisHistoryItem] = 0;
	}
}

//********************************************************************************************************************
//  clearHistory
//********************************************************************************************************************
void clearHistory() {
	int thisDevice;
	for (thisDevice = 0; thisDevice < MAX_DEVICES; thisDevice++) {
		clearDeviceHistory(thisDevice);
	}
}

//********************************************************************************************************************
void setLED() {
	//********************************************************************************************************************
	if (isrCountArray[D5] >= 1) {
		digitalWrite(NODEMCU_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
										  // but actually the LED is on; this is because 
										  // it is acive low on the ESP-01)
		delay(200);                      // Wait for a second
		digitalWrite(NODEMCU_LED, HIGH);  // Turn the LED off by making the voltage HIGH
		delay(200);                      // Wait for two seconds (to demonstrate the active low LED)
	}
}

//********************************************************************************************************************
unsigned long millisBetween(unsigned long millisStart, unsigned long millisEnd) {
	// return the number of millisecond values between the start values and end values of values assigned from millis()
	//********************************************************************************************************************
	if (millisStart > millisEnd) { // once every 50 days or so, the unsigned long will overflow back to zero
								   // in this case, return the difference between [the start value and the max] and the new increment from zero [millisEnd]
								   // this code will only execute once in ever 4 billion milliseconds or so!
		return (maxUnsignedLongValue - millisStart) + millisEnd;
	}
	else {
		return  millisEnd - millisStart;
	}
}

//********************************************************************************************************************
//
//********************************************************************************************************************
void SerialMessage(uint8_t thisDevice, const char* msg, unsigned long num) {
	if (VerboseSerial) {
		Serial.print(F("Device "));
		Serial.print(thisDevice, DEC);
		Serial.print(F("; "));
		Serial.print(msg);
		Serial.print(F(" = "));
		Serial.println(num, DEC);
	}
}

//********************************************************************************************************************
//  saveFlowHistory - save the flow history (the number of interrupts per second) in a table for the specified device
//********************************************************************************************************************
void saveFlowHistory(uint8_t thisDevice) {
	countHistoryIndex[thisDevice]++;
	if (countHistoryIndex[thisDevice] > MAX_HISTORY) {
		countHistoryIndex[thisDevice] = 0;
	}
	countFlowHistory[thisDevice][countHistoryIndex[thisDevice]] = countPerSecond[thisDevice];
}

//********************************************************************************************************************
//  averageCountFlow - return the average of the last [n] flow history (interrupts per second) where n=MAX_HISTORY
//                      (n is typically 60)
//********************************************************************************************************************
float averageCountFlow(uint8_t thisDevice) {
	int i;
	if (MAX_HISTORY > 0) {
		float s = 0; // sum of all history count values
		for (i = 0; i < MAX_HISTORY; i++) {
			s += countFlowHistory[thisDevice][i];
		}
		if (s == 0) {
			return 0;
		}
		else {
			return (s / MAX_HISTORY);
		}
	}
	else {
		// when control history depth is zero, simply return current value
		return countFlowHistory[thisDevice][0];
	}

}

//********************************************************************************************************************
//  doDeviceCalc - perform pulse counting and various summaries for a given device
//********************************************************************************************************************
void doDeviceCalc(uint8_t thisDevice) {
	// timeDelta[thisDevice] = millisBetween(last_time_counter[thisDevice], time_counter[thisDevice]);
	timeSince[thisDevice] = millisBetween(timeInterruptEvent[thisDevice], millis());

	// track pulses per second
	timeSecondDelta[thisDevice] = millisBetween(timePerSecondRef[thisDevice], millis());
	if (timeSecondDelta[thisDevice] >= 1000) {                                                  // 1,000 milliseconds is a minute
		countPerSecond[thisDevice] = isrCountArray[thisDevice] - countPerSecondRef[thisDevice]; // this seconds's count of interrupt pulses is the difference between current count and starting reference 
		countPerSecondRef[thisDevice] = isrCountArray[thisDevice];                              // the new reference starting point is the current total count of all pulses since power up
		timePerSecondRef[thisDevice] = millis();

		saveFlowHistory(thisDevice);                                                            // save the flow history every second
	} // endif: a second has passed for this device (1000 millis) 

	  // every calc session, if there's activity during the most recent 1000millis, then update some other things
	if (countPerSecond[thisDevice] > 1) {
		deviceActiveFlow[thisDevice] = true;
		countThisFlow[thisDevice] = isrCountArray[thisDevice] - countThisFlowRef[thisDevice];  // if there IS flow, the count this flow is the difference from last reference
	}
	else {
		cli(); // no interrupts while we reset delta to zero
		timeDelta[thisDevice] = 0;                                                             // with no flow, there's no delta
		sei(); // enable interrupts
	}


	// track pulses per minute
	timeMinuteDelta[thisDevice] = millisBetween(timePerMinuteRef[thisDevice], millis());
	if (timeMinuteDelta[thisDevice] >= 60000) {                                                 // 60,000 milliseconds is a minute
		countPerMinute[thisDevice] = isrCountArray[thisDevice] - countPerMinuteRef[thisDevice]; // this minute's count is the difference between current count and starting reference 
		countPerMinuteRef[thisDevice] = isrCountArray[thisDevice];                              // the new reference count is the current total count
		timePerMinuteRef[thisDevice] = millis();                                                // reset our minute delta timer; new starting reference is current millis value
		timeMinuteDelta[thisDevice] = 0;                                                        // reset our minute delta timer

		if (countPerMinute[thisDevice] < 2) { // when we have less than 2 pulses per minute, we assume end of this flow
			if (deviceActiveFlow[thisDevice]) {
				// if we previously thought there was an active flow, this must be the end
				countLastFlow[thisDevice] = isrCountArray[thisDevice] - countThisFlowRef[thisDevice];
				SerialMessage(thisDevice, "Last Total Flow", countLastFlow[thisDevice]);
				countThisFlow[thisDevice] = 0;                                                  // no flow for current flow (since there is no current flow!)

				char msg[80] = "Stop History=";
				char thisStr[80] = "";
				itoa(countLastFlow[thisDevice], thisStr, 10);
				strncat(msg, thisStr, (unsigned)strlen(thisStr));
#ifdef ENABLE_MQTT
				publishString(mqtt, AIO_USERNAME, msg, "history");
#endif // ENABLE_MQTT


				deviceActiveFlow[thisDevice] = false;                                           // just a flag to indicate that yes, sure enough, no flow (but note we only do this check once per minute)
			}
			SerialMessage(thisDevice, ">> Minute check: no current flow. Last total", countLastFlow[thisDevice]);
			countThisFlowRef[thisDevice] = isrCountArray[thisDevice];                           // if there's no flow, set the potential starting reference count to current total ount
		}
		else {
			if (!deviceActiveFlow[thisDevice]) {
				char msg[80] = "Start History=";
				char thisStr[80] = "";
				itoa(countLastFlow[thisDevice], thisStr, 10);
				strncat(msg, thisStr, (unsigned)strlen(thisStr));

#ifdef ENABLE_MQTT
				publishString(mqtt, AIO_USERNAME, msg, "history");
#endif // ENABLE_MQTT
				deviceActiveFlow[thisDevice] = true; // when we have less than 2 pulses per minute, we assume there's an active flow
			}
			countThisFlow[thisDevice] = isrCountArray[thisDevice] - countThisFlowRef[thisDevice]; // if there IS flow, the count this flow is the difference from last reference
			SerialMessage(thisDevice, ">> Minute check active FLOW!", countThisFlow[thisDevice]);
		}
	} // end of minute delta stuff

	  // if we have flow as indicated by pulses per second, but we do not yet have a minute total, estimate the minute total = [60 * pulses per second]
	if ((deviceActiveFlow[thisDevice] == true) && (countPerMinute[thisDevice] == 0)) {
		countPerMinute[thisDevice] = 60 * countPerSecond[thisDevice];
	}
}

//********************************************************************************************************************
//  doCalc - do all the doDEviceCalc operations for the devices that have been initialized
//********************************************************************************************************************
void doCalc() {
	int thisDevice;
	for (thisDevice = 0; thisDevice < MAX_DEVICES; thisDevice++) {
		if (deviceIsInitialized[thisDevice]) {
			doDeviceCalc(thisDevice);
		}
	}
}

//********************************************************************************************************************
// displayMessage - print some interesting flow data to the serial port
//********************************************************************************************************************
void displayMessage(uint8_t thisDevice) {
	Serial.println();
	Serial.print(F("Device "));
	Serial.print(thisDevice, DEC);
	Serial.print(F("; Count="));
	Serial.print(isrCountArray[thisDevice], DEC);

	Serial.print(F("; PPS="));
	Serial.print(countPerSecond[thisDevice], DEC);

	Serial.print(F("; PPM="));
	Serial.print(countPerMinute[thisDevice], DEC);

	Serial.print(F("; TotalThisFlow="));
	Serial.print(countThisFlow[thisDevice], DEC);

	Serial.print(F("; Time Delta="));
	Serial.print(timeDelta[thisDevice], DEC);
	Serial.print(F("; Time Since="));
	Serial.print(timeSince[thisDevice], DEC);
	if (deviceActiveFlow[thisDevice]) {
		if (timeLastDelta[thisDevice] > timeDelta[thisDevice]) {
			Serial.print(F("; Flow rate decreasing"));
		}
		else if (timeLastDelta[thisDevice] < timeDelta[thisDevice]) {
			Serial.print(F("; Flow rate INCREASING"));
		}
		else {
			Serial.print(F("; Flow rate steady"));
		}
	}
	else {
		Serial.print(F("; No current flow"));
	}
	Serial.print(F("; Avg="));
	Serial.print(averageCountFlow(thisDevice), DEC);


	Serial.println(F("."));
	SerialMessage(thisDevice, "Last Total Flow", countLastFlow[thisDevice]);
}

unsigned long lastMessageMillis = millis();

//********************************************************************************************************************
// initPinNames
//
// initialize our pinName array with the names of pins as printed, not the actual GPIO number.
//
// Everywhere we reference the pin by the label on the PCB (D[x]), not the actual mapped GPIO pin (see pins_arduino.h)
// however even though the data is stored by GPIO number in the value of D[x], we are interested in the *name* of [x].
//
// for example, we use variable D5 in code. "D5" is printed on the circuit board, but actualy GPIO port is 14
// the array references will store values in the 14th position... however when we talk to the MQTT we want to
// use the reference we SEE on the board (e.g. "D5" and not the value of 14). so: pinName[D5] = pinName[14] = "D5"
//
// not all pins are defined for all hardware types
//********************************************************************************************************************
void initPinNames() {
	if (MAX_DEVICES >= D1) { strcpy(pinName[D1], "D1"); };
	if (MAX_DEVICES >= D2) { strcpy(pinName[D2], "D2"); };
	if (MAX_DEVICES >= D3) { strcpy(pinName[D3], "D3");; };
	if (MAX_DEVICES >= D4) { strcpy(pinName[D4], "D4"); };
	if (MAX_DEVICES >= D5) { strcpy(pinName[D5], "D5"); };
	if (MAX_DEVICES >= D6) { strcpy(pinName[D6], "D6"); };
	if (MAX_DEVICES >= D7) { strcpy(pinName[D7], "D7"); };
	if (MAX_DEVICES >= D8) { strcpy(pinName[D8], "D8"); };
	if (MAX_DEVICES >= D9) { strcpy(pinName[D9], "D9"); };
	if (MAX_DEVICES >= D10) { strcpy(pinName[D10], "D10"); };
	//if (MAX_DEVICES >= D11) { strcpy(pinName[D11], "D11"); };
	//if (MAX_DEVICES >= D12) { strcpy(pinName[D12], "D12"); };
	//if (MAX_DEVICES >= D13) { strcpy(pinName[D13], "D13"); };
	//if (MAX_DEVICES >= D14) { strcpy(pinName[D14], "D14"); };
	//if (MAX_DEVICES >= D15) { strcpy(pinName[D15], "D15"); };
	//if (MAX_DEVICES >= D16) { strcpy(pinName[D16], "D16"); };
}


#ifdef ENABLE_MQTT
void verifyFingerprint() {

	const char* host = AIO_SERVER;

	Serial.print("Connecting to ");
	Serial.println(host);

	if (!client.connect(host, AIO_SERVERPORT)) {
		Serial.println("Connection failed. Halting execution.");
		while (1);
	}

	if (client.verify(fingerprint, host)) {
		Serial.println("Connection secure.");
	}
	else {
		Serial.println("Connection insecure! Halting execution.");
		while (1);
	}

}

void mqtt_client_connect(Adafruit_MQTT_Client& thisMQTT) {
	const char* host = AIO_SERVER;
	if (!client.connect(host, AIO_SERVERPORT)) {
		Serial.println("Client connection failure!");
	}
	else {
		Serial.println("SSL Connection Success!");
		MQTT_connect(thisMQTT);
	}
	verifyFingerprint();
}
#endif // ENABLE_MQTT



//********************************************************************************************************************
//********************************************************************************************************************
// setup
//********************************************************************************************************************
//********************************************************************************************************************
void setup() {
	// noInterrupts(); // this is implicit

	Serial.begin(115200);      // open the serial port at 9600 bps:  
	delay(21);
	Serial.println("Startup! Version 0.005");

	WiFi.mode(WIFI_STA);
	WiFi.begin(WIFI_SSID, WIFI_PWD);

	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	clearHistory();          // initialize device flow history to all zero values
	Serial.println("Pre!");
	initPinNames();          // initialize all the pin names (e.g. pinName[D5] = "D5")
	Serial.println("post!");
	Serial.println(pinName[D5]);
	Serial.println("done!");
	initDevice(D5);
	initDevice(D1);



	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());





	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	pinMode(ESP_BUILTIN_LED, OUTPUT);


	pinMode(NODEMCU_LED, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

	setLED();
	//  attachInterrupt(GPIO_PIN, InterruptServiceD5, RISING);

	Serial.println();
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(WIFI_SSID);

#ifdef ENABLE_MQTT
	mqtt_client_connect(mqtt);
	// interrupts(); // this is implicit; allow interrupts only after setup id complete
	publishString(mqtt, AIO_USERNAME, "Startup!", "history");
	publishString(mqtt, AIO_USERNAME, "Ready!", "history");
#endif // ENABLE_MQTT
}



//********************************************************************************************************************
//********************************************************************************************************************
// loop - this is the main app
//********************************************************************************************************************
//********************************************************************************************************************
void loop() {
	ArduinoOTA.handle();

	if ((millis() - lastMessageMillis) > 5000) {
		displayMessage(D5);
		displayMessage(D1);
		lastMessageMillis = millis();
	}

	doCalc();                         // do all the device calculations for all devices that have been initialized in setup

									  // setLED();

	digitalWrite(NODEMCU_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
									  // but actually the LED is on; this is because 
									  // it is acive low on the ESP-01)
									  //delay(1);                        // Wait for a second

	digitalWrite(NODEMCU_LED, HIGH);  // Turn the LED off by making the voltage HIGH




#ifdef ENABLE_MQTT
	if ((millis() - lastAdafruitMessageMillis) > 5000) {
		Serial.println("Start loop!");

		Serial.println(millis(), DEC);
		Serial.println(lastAdafruitMessageMillis, DEC);
		publishString(mqtt, AIO_USERNAME, "loop!", "history");
		Serial.println(millis(), DEC);
		Serial.println(lastAdafruitMessageMillis, DEC);
		Serial.println("End loop!");
		Serial.println("");

		// Ensure the connection to the MQTT server is alive (this will make the first
		// connection and automatically reconnect when disconnected).  See the MQTT_connect
		// function definition further below.

		publishValue(mqtt, AIO_USERNAME, pinName[D1], averageCountFlow(D1));
		publishValue(mqtt, AIO_USERNAME, pinName[D5], averageCountFlow(D5));
		// MQTT_connect();
		//if (!flowD1.publish((int)  averageCountFlow(D1))) {
		// Serial.println(F("Failed"));
		//}
		//else {
		// Serial.println(F("OK!"));
		//}
		lastAdafruitMessageMillis = millis();
#endif // ENABLE_MQTT
	}
}

