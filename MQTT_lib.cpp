// 
// 
// 
//#include <Adafruit_MQTT.h> // install zip library in Arduino from https://github.com/adafruit/Adafruit_MQTT_Library then refresh in Visual Micro  (probably best to restart Visual Studio)
#include <ESP8266WiFi.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>




void MQTT_connect(Adafruit_MQTT_Client& thisMQTT ) {

	// Stop if already connected.
	if (thisMQTT.connected()) {
		Serial.println("MQTT is connected!");
		return;
	}

	Serial.print("Connecting to MQTT... ");
	typedef signed char int8_t;
	int8_t ret;

	uint8_t retries = 3;
	while ((ret = thisMQTT.connect()) != 0) { // connect will return 0 for connected
		Serial.println(thisMQTT.connectErrorString(ret));
		Serial.println("Retrying MQTT connection in 5 seconds...");
		thisMQTT.disconnect();
		delay(5000);  // wait 5 seconds
		retries--;
		if (retries == 0) {
			// basically die and wait for WDT to reset me
			while (1);
		}
	}

	Serial.println("MQTT Connected!");
}



/****************************** Feeds ***************************************/
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish flowD1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/flowD1");
//Adafruit_MQTT_Publish flowD5 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/flowD5");
//Adafruit_MQTT_Publish flowD5_History = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/flowD5_History");


//********************************************************************************************************************
// publishString
//
// be sure feed exists! see https://learn.adafruit.com/adafruit-io-basics-feeds/creating-a-feed
//
//********************************************************************************************************************
void publishString(Adafruit_MQTT_Client& thisMQTT, char toUserName[128], char thisValue[], char feedname[] = "history") {
// 	char publishName[128] = AIO_USERNAME; // "/feeds/history";


	char thisPublishName[128] = "\0";

	strcat(thisPublishName, toUserName);

	strcat(thisPublishName, "/feeds/");

	strncat(thisPublishName, feedname, (unsigned)strlen(feedname));


	// example: Adafruit_MQTT_Publish flowD1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/flowD1");

	Serial.print("Publishing ");
	Serial.print(thisValue);
	Serial.print(" to ");
	Serial.print(thisPublishName);
	Serial.print("; ");

	Adafruit_MQTT_Publish thisPublisher = Adafruit_MQTT_Publish(&thisMQTT, thisPublishName);
	MQTT_connect(thisMQTT);

	if (!thisPublisher.publish(thisValue)) {
		Serial.println(F(" Failed"));
	}
	else {
		Serial.println(F(" Success!"));
	}
}

//********************************************************************************************************************
// publishValue
//
// be sure feed exists! see https://learn.adafruit.com/adafruit-io-basics-feeds/creating-a-feed
//
//********************************************************************************************************************
void publishValue(Adafruit_MQTT_Client& thisMQTT, char toUserName[128], char thisPinName[16], double thisValue, char feedname[] = "flow") {
	// char publishName[128] = AIO_USERNAME;

	char thisPublishName[128] = "\0";

	strcat(thisPublishName, toUserName);

	strcat(thisPublishName, "/feeds/");

	strncat(thisPublishName, feedname, (unsigned)strlen(feedname));

	strncat(thisPublishName, thisPinName, (unsigned)strlen(thisPinName));

	// Adafruit_MQTT_Publish flowD1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/flowD1");

	Serial.print("Publishing to ");
	Serial.print(thisPublishName);
	Serial.print(";  ");

	Adafruit_MQTT_Publish thisPublisher = Adafruit_MQTT_Publish(&thisMQTT, thisPublishName);
	MQTT_connect(thisMQTT);

	if (!thisPublisher.publish(thisValue)) {
		Serial.println(F(" Failed"));
	}
	else {
		Serial.println(F(" Success!"));
	}
}




////********************************************************************************************************************
////  publishValue
////********************************************************************************************************************
//void publishValue(uint8_t thisDevice, double thisValue) {
// // the MQTT path here is [name]/feeds/flow[device] e.g. gojimmypi/feeds/flowD1
// publishValue(thisDevice, thisValue, "flow");
//}

//********************************************************************************************************************
//  publishHistoryValue
//********************************************************************************************************************
//void publishHistoryValue(uint8_t thisDevice, double thisValue) {
//	publishValue(thisDevice, thisValue, "flowHistory");
//}

