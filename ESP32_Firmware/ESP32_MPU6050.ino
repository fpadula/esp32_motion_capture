#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include <WiFi.h>
#include <MPU6050.h>
#include <Preferences.h>

MPU6050 mpu;

#define INTERRUPT_PIN 23  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2		 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SDA 21		 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SCL 22		 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;	// return status after each device operation (0 = success,
					  // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

#define SENSOR_IDENTIFIER 'b'
uint8_t unityPacket[9] = {0, 0, 0, 0, 0, 0, 0, 0, SENSOR_IDENTIFIER};


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt =
	false;  // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

const char* ssid = "padula-note";
const char* password = "hotspotforimu";

const uint16_t port = 8090;
const char* host = "padula-note.local";

const int PushButton = 18;

WiFiClient client;
Preferences preferences;

int16_t *GetActiveOffsets(MPU6050 *mpu){
	uint8_t AOffsetRegister =
		(mpu->getDeviceID() < 0x38) ? MPU6050_RA_XA_OFFS_H : 0x77;
	int16_t *ret;

	ret = (int16_t *)malloc(sizeof(int16_t)*6);
	int16_t Data[3];
	//Serial.print(F("Offset Register 0x"));
	//Serial.print(AOffsetRegister>>4,HEX);Serial.print(AOffsetRegister&0x0F,HEX);
	// Serial.print(F("\n//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n//OFFSETS   "));
	if(AOffsetRegister == 0x06)
		I2Cdev::readWords(MPU6050_DEFAULT_ADDRESS, AOffsetRegister, 3,
						  (uint16_t *)Data);
	else {
		I2Cdev::readWords(MPU6050_DEFAULT_ADDRESS, AOffsetRegister, 1,
						  (uint16_t *)Data);
		I2Cdev::readWords(MPU6050_DEFAULT_ADDRESS, AOffsetRegister + 3, 1,
						  (uint16_t *)Data + 1);
		I2Cdev::readWords(MPU6050_DEFAULT_ADDRESS, AOffsetRegister + 6, 1,
						  (uint16_t *)Data + 2);
	}
	//	A_OFFSET_H_READ_A_OFFS(Data);
	// printfloatx("", Data[0], 5, 0, ",  ");
	// printfloatx("", Data[1], 5, 0, ",  ");
	// printfloatx("", Data[2], 5, 0, ",  ");
    ret[0] = Data[0];
    ret[1] = Data[1];
    ret[2] = Data[2];
	I2Cdev::readWords(MPU6050_DEFAULT_ADDRESS, 0x13, 3, (uint16_t *)Data);
	//	XG_OFFSET_H_READ_OFFS_USR(Data);
	ret[3] = Data[0];
	ret[4] = Data[1];
	ret[5] = Data[2];
    return ret;
}

void setup() {
    int16_t *offsets, read_offsets[6], buffer;            
    int i, addr;

	pinMode(LED_PIN, OUTPUT);

	Wire.begin();
	Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
							// compilation difficulties
	Serial.begin(115200);

	pinMode(PushButton, INPUT);
	digitalWrite(LED_PIN, true);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
										: F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
    preferences.begin("IMUWS", false);   
	if (devStatus == 0) {
		// Calibration Time: generate offsets and calibrate our MPU6050
        if (digitalRead(PushButton) == HIGH) {

			Serial.println(F("Calibration button pushed. Setting offsets and calibrating..."));
            blinkState = true;
            for (i = 0; i < 10; i++){
			    digitalWrite(LED_PIN, blinkState);
                blinkState = !blinkState;
                delay(500);
            }
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            offsets = GetActiveOffsets(&mpu);
            mpu.setXAccelOffset(offsets[0]);
            mpu.setYAccelOffset(offsets[1]);
            mpu.setZAccelOffset(offsets[2]);
            mpu.setXGyroOffset(offsets[3]);
            mpu.setYGyroOffset(offsets[4]);
            mpu.setZGyroOffset(offsets[5]);

            preferences.putBytes("offsets", offsets, sizeof(int16_t) * 6);
            
            Serial.print(F("\n//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n//OFFSETS SAVED ON EPROM   "));
            #define printfloatx(Name,Variable,Spaces,Precision,EndTxt) { Serial.print(F(Name)); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(F(EndTxt)); }//Name,Variable,Spaces,Precision,EndTxt
			printfloatx("", offsets[0], 5, 0, ",  ");
			printfloatx("", offsets[1], 5, 0, ",  ");
			printfloatx("", offsets[2], 5, 0, ",  ");
			printfloatx("", offsets[3], 5, 0, ",  ");
			printfloatx("", offsets[4], 5, 0, ",  ");
			printfloatx("", offsets[5], 5, 0, ",  ");
            // EEPROM.commit();
            free(offsets);
		}
        else{
            preferences.getBytes("offsets", read_offsets, sizeof(int16_t) * 6);
            Serial.println();
            Serial.print(F("\n//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n//OFFSETS READ FROM EPROM   "));
            #define printfloatx(Name,Variable,Spaces,Precision,EndTxt) { Serial.print(F(Name)); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(F(EndTxt)); }//Name,Variable,Spaces,Precision,EndTxt
			printfloatx("", read_offsets[0], 5, 0, ",  ");
			printfloatx("", read_offsets[1], 5, 0, ",  ");
			printfloatx("", read_offsets[2], 5, 0, ",  ");
			printfloatx("", read_offsets[3], 5, 0, ",  ");
			printfloatx("", read_offsets[4], 5, 0, ",  ");
			printfloatx("", read_offsets[5], 5, 0, ",  ");
            mpu.setXAccelOffset(read_offsets[0]);
            mpu.setYAccelOffset(read_offsets[1]);
            mpu.setZAccelOffset(read_offsets[2]);
            mpu.setXGyroOffset(read_offsets[3]);
            mpu.setYGyroOffset(read_offsets[4]);
            mpu.setZGyroOffset(read_offsets[5]);
        }
    // #ifdef CALIBRATE
    // #endif
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(
			F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady,
						RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to
		// use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	

	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.println("...");
	}

	Serial.print("WiFi connected with IP: ");
	Serial.println(WiFi.localIP());
	digitalWrite(LED_PIN, false);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {    
      
	if (!client.connected()) {
		if (!client.connect(host, port)) {
			Serial.println("Connection to host failed");

			delay(1000);
			return;
		} else {
			Serial.println("Connected to server successful!");
		}
	}

	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		if (mpuInterrupt && fifoCount < packetSize) {
			// try to get out of the infinite loop
			fifoCount = mpu.getFIFOCount();
		}
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too
	// inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) ||
		fifoCount >= 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen
		// frequently)
	} 
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		unityPacket[0] = fifoBuffer[0];
		unityPacket[1] = fifoBuffer[1];
		unityPacket[2] = fifoBuffer[4];
		unityPacket[3] = fifoBuffer[5];
		unityPacket[4] = fifoBuffer[8];
		unityPacket[5] = fifoBuffer[9];
		unityPacket[6] = fifoBuffer[12];
		unityPacket[7] = fifoBuffer[13];
		client.write(unityPacket, 9);
		
	}
}