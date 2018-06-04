//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

constexpr int BLOCKSIZE = 16; // use fractions of 256

SoftwareSerial loopBack(D5, D6, false, BLOCKSIZE + 2);
unsigned long start;
String effTxTxt("eff. tx: ");
String effRxTxt("eff. rx: ");
int txCount;
int rxCount;
unsigned char expected;
int rxErrors;
constexpr int ReportInterval = 10000;

void setup() {
	Serial.begin(115200);
	//WiFi.mode(WIFI_OFF);
	//WiFi.forceSleepBegin();
	//delay(1);
	loopBack.begin(38400);
	start = micros();
	txCount = 0;
	rxCount = 0;
	expected = -1;
	rxErrors = 0;
}

unsigned char c = 0;

void loop() {
	do {
		loopBack.write(c);
		c = ++c % 256;
		++txCount;
	} while (c % BLOCKSIZE);
	if (loopBack.overflow()) { Serial.println("overflow"); }
	while (loopBack.available()) {
		unsigned char r = loopBack.read();
		if (r == -1) { Serial.println("read() == -1"); }
		if (expected == -1) { expected = r; }
		else {
			expected = ++expected % 256;
		}
		if (r != expected) {
			++rxErrors;
			expected = -1;
		}
		++rxCount;
	}

	if (txCount >= ReportInterval) {
		Serial.println(String("tx/rx: ") + txCount + "/" + rxCount);
		const auto end = micros();
		const unsigned long interval = end - start;
		const long txCps = txCount * (1000000.0 / interval);
		const long rxCps = rxCount * (1000000.0 / interval);
		const long errorCps = rxErrors * (1000000.0 / interval);
		Serial.println(effTxTxt + 10 * txCps + "bps, "
					   + effRxTxt + 10 * rxCps + "bps, "
					   + errorCps + "cps errors (" + 100.0 * rxErrors / rxCount + "%)");
		start = end;
		txCount = 0;
		rxCount = 0;
		rxErrors = 0;
	}
}
