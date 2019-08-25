////////////////////////////////////////
// LIBRARIES
////////////////////////////////////////

#include <Adafruit_BME680.h>
#include "Particle.h"
#include "ConnectionEvents.h"
#include "ConnectionCheck.h"
#include <ThingSpeak.h>

////////////////////////////////////////
// BME680 CONSTANTS
////////////////////////////////////////

#define BME_CS A2
#define BME_SCK A3 
#define BME_MISO A4 
#define BME_MOSI A5 

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

double temperatureInC = 0;
double relativeHumidity = 0;
double pressureHpa = 0;
double gasResistanceKOhms = 0;
double approxAltitudeInM = 0;

int attempts = 0; 
bool BME680check = false;

////////////////////////////////////////
//PLANTOWER CONSTANTS
////////////////////////////////////////

#define setPin B2	
int PM10, PM1, PM2_5 = 0;
int incomingByte = 0; 
const int MAX_FRAME_LEN = 64;
char frameBuf[MAX_FRAME_LEN];
int detectOff = 0;
int frameLen = MAX_FRAME_LEN;
bool inFrame = false;
bool PLANTOWERcheck = false;
char printbuf[256];
uint16_t calcChecksum = 0;
struct PMS7003_framestruct {
	uint8_t  frameHeader[2];
	uint16_t frameLen = MAX_FRAME_LEN;
	uint16_t concPM1_0_CF1;
	uint16_t concPM2_5_CF1;
	uint16_t concPM10_0_CF1;
	uint16_t concPM1_0_amb;
	uint16_t concPM2_5_amb;
	uint16_t concPM10_0_amb;
	uint16_t rawGt0_3um;
	uint16_t rawGt0_5um;
	uint16_t rawGt1_0um;
	uint16_t rawGt2_5um;
	uint16_t rawGt5_0um;
	uint16_t rawGt10_0um;
	uint8_t  version;
	uint8_t  errorCode;
	uint16_t checksum;
} thisFrame;

////////////////////////////////////////
// THING_SPEAK
////////////////////////////////////////

TCPClient client;                                      
unsigned long myChannelNumber = ...;    // INSERT CHANNEL NUMBER 
const char * myWriteAPIKey = "...";     // INSERT WRITE API KEY

////////////////////////////////////////
// STARTUP and SETUP
////////////////////////////////////////

void startup(){
  System.enableFeature(FEATURE_RETAINED_MEMORY);
} 

STARTUP(startup);
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void setup() {
	
	Serial.begin(9600);
	ConnectionCheck conncheck;
	conncheck.withFailureSleepSec(10 * 60);
	Particle.connect();	
	
	pinMode(setPin, OUTPUT);
	digitalWrite(setPin, LOW);
	
	if (!bme.begin()) {
		delay(10000);
		System.reset();
	}	
	else { 
	      // Set up oversampling and filter initialization
	      bme.setTemperatureOversampling(BME680_OS_8X);
	      bme.setHumidityOversampling(BME680_OS_2X);
	      bme.setPressureOversampling(BME680_OS_4X);
	      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	      bme.setGasHeater(320, 150); // 320*C for 150 ms
	      
	}
	
}

////////////////////////////////////////
// PERFORM LOOP
////////////////////////////////////////

void loop() {
	
	// Wait for 30 seconds before reseting (via Deep Sleep)
	if (waitFor (Particle.connected, 30000)) {

   	 ThingSpeak.begin(client);

	// READ THE PLANTOWER
	if (PLANTOWERcheck == false) {
		digitalWrite(setPin, HIGH);
		Serial1.begin(9600);
		delay(1000);
		bool packetReceived = false;
		while (!packetReceived) {
			if (Serial1.available() > 32) {
				int drain = Serial1.available();
				for (int i = drain; i > 0; i--) {
					Serial1.read();
				}
			}
			if (Serial1.available() > 0) {
				incomingByte = Serial1.read();
				if (!inFrame) {
					if (incomingByte == 0x42 && detectOff == 0) {
						frameBuf[detectOff] = incomingByte;
						thisFrame.frameHeader[0] = incomingByte;
						calcChecksum = incomingByte;
						detectOff++;
					}
					else if (incomingByte == 0x4D && detectOff == 1) {
						frameBuf[detectOff] = incomingByte;
						thisFrame.frameHeader[1] = incomingByte;
						calcChecksum += incomingByte;
						inFrame = true;
						detectOff++;
					}
				}
				else {
					frameBuf[detectOff] = incomingByte;
					calcChecksum += incomingByte;
					detectOff++;
					uint16_t val = frameBuf[detectOff - 1] + (frameBuf[detectOff - 2] << 8);
					switch (detectOff) {
					case 4:
						thisFrame.frameLen = val;
						frameLen = val + detectOff;
						break;
					case 6:
						thisFrame.concPM1_0_CF1 = val;
						break;
					case 8:
						thisFrame.concPM2_5_CF1 = val;
						break;
					case 10:
						thisFrame.concPM10_0_CF1 = val;
						break;
					case 12:
						thisFrame.concPM1_0_amb = val;
						break;
					case 14:
						thisFrame.concPM2_5_amb = val;
						break;
					case 16:
						thisFrame.concPM10_0_amb = val;
						break;
					case 18:
						thisFrame.rawGt0_3um = val;
						break;
					case 20:
						thisFrame.rawGt0_5um = val;
						break;
					case 22:
						thisFrame.rawGt1_0um = val;
						break;
					case 24:
						thisFrame.rawGt2_5um = val;
						break;
					case 26:
						thisFrame.rawGt5_0um = val;
						break;
					case 28:
						thisFrame.rawGt10_0um = val;
						break;
					case 29:
						val = frameBuf[detectOff - 1];
						thisFrame.version = val;
						break;
					case 30:
						val = frameBuf[detectOff - 1];
						thisFrame.errorCode = val;
						break;
					case 32:
						thisFrame.checksum = val;
						calcChecksum -= ((val >> 8) + (val & 0xFF));
						break;
					default:
						break;
					}

					if (detectOff >= frameLen) {
						PM1 = thisFrame.concPM1_0_CF1;
						PM2_5 = thisFrame.concPM2_5_CF1;
						PM10 = thisFrame.concPM10_0_CF1;
						digitalWrite(setPin, LOW);
						packetReceived = true;
						detectOff = 0;
						inFrame = false;
						Serial1.end();
						PLANTOWERcheck = true;
					}
				}
			}
		}
	}


	// READ THE BME680
	if (BME680check == false) {	
	if (!bme.performReading()) {
		delay(100);
		System.reset();
		}

		else {
			attempts = 0;
			for (int i = 0; i < 6; i++) {
				bme.performReading();
				delay(50);
				temperatureInC = bme.temperature;
				relativeHumidity = bme.humidity;
				pressureHpa = bme.pressure / 100.0;
				gasResistanceKOhms = bme.gas_resistance / 1000.0;
				delay(20);

				if (gasResistanceKOhms != 0) {
					break;
				}

      				if (i == 5) {
					delay(100);
					break;
				}
				attempts++;
			}
			BME680check = true;
		}	
	}		


	if (PLANTOWERcheck == true && BME680check == true) {

		// Update the ThingSpeak fields with the data
		ThingSpeak.setField(1, (float)temperatureInC);
		ThingSpeak.setField(2, (float)relativeHumidity);
		ThingSpeak.setField(3, (float)pressureHpa);
		ThingSpeak.setField(4, (float)gasResistanceKOhms);
		ThingSpeak.setField(5, PM1);
		ThingSpeak.setField(6, PM2_5);    
		ThingSpeak.setField(7, PM10);

		ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
		delay(5000);  

		PLANTOWERcheck = false;
		BME680check = false;			
		delay(1200);

		// SET SLEEP INTERVAL BETWEEN READINGS
		System.sleep(SLEEP_MODE_DEEP, 15*60); 	// CURRENT INTERVAL = 15 minutes

	}	
		
}

else {
	System.sleep(SLEEP_MODE_DEEP, 5);
}

delay(1000);

}
