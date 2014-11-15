/*
KDS Reader - for reading data from the Kawasaki Diagnostic System (KDS) Port

KDS Packet format is:

0x8? - Start Addressed Packet, ? = 1, one byte packet. ? = 0, packet size below.
0x?? - Target Address (ECU = 0x11)
0x?? - Source Address (GiPro = 0xF1)
0x?? - Single byte command/response if first byte is 0x?1, otherwise number of bytes (n).
.... - (n) bytes of command/response data
0x?? - Checksum = sum of all previous bytes & 0xFF

Commands/Responses:
0x81				- Start Communication Request
0xC1 0xEA 0x8F		- Start Communication Accepted
0x10 0x80			- Start Diagnostic Session Request
0x50 0x80			- Start Diagnostic Session accepted
0x21 0x??			- Request register 0x?? value
0x61 0x?? 0x## ...	- Register response for register 0x??, value(s) 0x##
0x7F 0x21 0x##		- Negative response for register, error code 0x##

2013 Z1000SX (Ninja 1000)
Registers (byte responses are: a, b, c...):
00 (4 bytes):	?
01 (1 byte):	?
02 (1 byte):	?
04 (2 bytes):	Throttle Position Sensor: 0% = 0x00 0xD8, 100% = 0x03 0x7F /// TODO: VERIFY
05 (2 bytes):	Air Pressure = ??
06 (1 byte):	Engine Coolant Temperature = (a - 48) / 1.6
07 (1 bytes):	Intake Air Temperature
08 (2 bytes):	Abs Pressure(?)
09 (2 bytes):	Engine RPM = (a * 100) + b ... 
0A (1 byte):	?
0B (1 byte):	Gear Position = x
0C (2 bytes):	Speed = (a << 8 + b) / 2
20 (4 bytes):	?
27 (1 byte):	?
28 (1 byte):	?
29 (1 byte):	?
2A (1 byte):	?
2E (1 byte):	?
31 (1 byte):	?
32 (1 byte):	?
33 (1 byte):	?
3C (1 byte):	?
3D (1 byte):	?
3E (1 byte):	?
3F (1 byte):	?
40 (4 bytes):	?
44 (4 bytes):	?
54 (2 bytes):	?
56 (1 byte):	?
5B (1 byte):	?
5C (1 byte):	?
5D (1 byte):	?
5E (1 byte):	?
5F (1 byte):	?
60 (4 bytes):	?
61 (1 byte):	?
62 (2 bytes):	?
63 (1 byte):	?
64 (1 byte):	?
65 (1 byte):	?
66 (1 byte):	?
67 (1 byte):	?
68 (1 byte):	?
6E (1 byte):	?
6F (1 byte):	?
80 (4 bytes):	?
9B (1 byte):	?
A0 (4 bytes):	?
B4 (1 byte):	?

From ISO14230-2:
Time (ms)	Sequence
5-20		Inter byte time in tester request
0-20		Inter byte timing in ECU response
25-50		Time between end of tester request and start of ECU response or between ECU responses
25-5000		Extended mode for "rspPending"
55-5000		Time between end of ECU response and start of new tester request, or time between end of tester
				request and start of new request if ECU doesn't respond

*/

#include <Adafruit_NeoPixel.h>

#define K_OUT 1 // K Output Line - TX on Arduino
#define K_IN 0 // K Input Line - RX on Arduino
#define SERIAL_ON 3

#define MAXSENDTIME 5000 // 5 second timeout on KDS comms.

// Animation settings
#define MAX_RPM 10000

// LED settings
#define N_PIXELS 60
#define LED_PIN 6
#define GREEN_LED 5
#define RED_LED 4
#define BOARD_LED 13

const uint32_t ISORequestByteDelay = 10;
const uint32_t ISORequestDelay = 20; // Time between requests.

const bool debugKDS = true;
const bool debugPkt = true;
const bool reqAllReg = false;

// Connect rolePin to Ground for slave unit (connected to KDS port).
const uint8_t rolePin = 8;

const uint8_t ECUaddr = 0x11;
const uint8_t myAddr = 0xF2;

const uint8_t validRegs[] = { 0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08,
	0x09, 0x0A, 0x0B, 0x0C, 0x20, 0x27, 0x28, 0x29, 0x2A, 0x2E, 0x31, 0x32,
	0x33, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x44, 0x54, 0x56, 0x5B, 0x5C, 0x5D,
	0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x6E,
	0x6F, 0x80, 0x9B, 0xA0, 0xB4 };

const uint8_t numValidRegs = (uint8_t)(sizeof(validRegs));


Adafruit_NeoPixel
	strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


bool ECUconnected = false;
uint32_t rpms = 0;

void setup()
{
	char rBuf[32] = "";

	pinMode(K_OUT, OUTPUT);
	pinMode(K_IN, INPUT);
        pinMode(SERIAL_ON, OUTPUT);
        pinMode(BOARD_LED, OUTPUT);
        pinMode(RED_LED, OUTPUT);
        pinMode(GREEN_LED, OUTPUT);

        digitalWrite(BOARD_LED, LOW);

	strip.begin();

        for (uint8_t i = 0; i < N_PIXELS; i++) {
          strip.setPixelColor(i, 255, 255, 255);
        }
        
        strip.show();
        
        delay(2000);
        
        strip.clear();
        strip.show();
}

void loop()
{
        digitalWrite(SERIAL_ON, HIGH);
        digitalWrite(BOARD_LED, HIGH);
        delay(300);
        digitalWrite(BOARD_LED, LOW);
        delay(300);
        digitalWrite(BOARD_LED, HIGH);
        delay(300);
        digitalWrite(BOARD_LED, LOW);
        
	char strBuf[21];
	uint8_t cmdSize;
	uint8_t cmdBuf[6];
	uint8_t respSize;
	uint8_t respBuf[12];
	uint8_t ect;

	if (!ECUconnected) {
		// Start KDS comms
		ECUconnected = initPulse();

		if (ECUconnected) {
                  digitalWrite(BOARD_LED, HIGH);
		} else {
                  digitalWrite(BOARD_LED, LOW);
                  digitalWrite(SERIAL_ON, LOW);
		}
	}
	// Endless loop.
        boolean redOn = true;
	while (ECUconnected) {
  
  		strip.clear();
  
		// Send register requests
		cmdSize = 2; // each request is a 2 byte packet.
		cmdBuf[0] = 0x21; // Register request cmd
		// Response to a register request is either:
		// 0x61 - Register read OK
		// 0x?? - Register requested
		// 0x?? - Value byte 1
		// ...  - (if more than 1 byte value - remainder of values)
		// ___ or:
		// 0x7F - Error response
		// 0x21 - command (0x21 = Read register)
		// 0x?? - error code (0x10 = General Reject: The service is rejected
		//			but the server does not specify the reason of the rejection

		for (uint8_t i = 0; i < 4; i++) respBuf[i] = 0;
		// Request Coolant Temp is register: 0x06
		
		// This bit isn't working - seems to be printing a 2 byte response
		// when the type is only 1 byte - and the formula doesn't work.
		
		//cmdBuf[1] = 0x06;
		//respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
		//if (respSize == 3) {
		//	ect = (respBuf[2] - 48) / 1.6;
		//	//              "                    "
		//	sprintf(strBuf, "TEMPC:  %3d    [%02hhX]", ect, respBuf[2]);
		//	lcd.setCursor(0,2);
		//	lcd.print(strBuf);
		//	if (SDCardPresent) {
		//		dataFile.println(strBuf);
		//	}
		//}
		//delay(ISORequestDelay);
				
		//for (uint8_t i = 0; i < 4; i++) respBuf[i] = 0;
		//// Request GEAR is register: 0x0B
		//cmdBuf[1] = 0x0B;
		//respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
		//if (respSize == 3) {
		//	//              "                    "
		//	sprintf(strBuf, "GEAR#:   %02hhX    [%02hhX]", respBuf[2], respBuf[2]);
		//	lcd.setCursor(0, 1);
		//	lcd.print(strBuf);
		//	if (SDCardPresent) {
		//		dataFile.println(strBuf);
		//	}
		//}
		//delay(ISORequestDelay);

		for (uint8_t i = 0; i < 5; i++) respBuf[i] = 0;
		// Request RPM is register: 0x09
		cmdBuf[1] = 0x09;
		respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
		if (respSize == 4) {
			//              "                    "
			sprintf(strBuf, "RPM's: %4d [%02hhX|%02hhX]", respBuf[2] * 100 + respBuf[3], respBuf[2], respBuf[3]);
                        rpms = respBuf[2] * 100 + respBuf[3];
                        rpms = max(min(rpms, MAX_RPM), 0);
                        if (rpms > 2000) {
                          digitalWrite(GREEN_LED, HIGH);
                        }
                        else {
                          digitalWrite(GREEN_LED, LOW);
                        }
                        if (redOn) {
                          digitalWrite(RED_LED, HIGH);
                        }
                        else {
                          digitalWrite(RED_LED, LOW);
                        }
                        redOn = !redOn;

		}
		delay(ISORequestDelay);

		//for (uint8_t i = 0; i < 5; i++) respBuf[i] = 0;
		// Request Speed is register: 0x0C
		//cmdBuf[1] = 0x0C;
		//respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
		//if (respSize == 4) {
		//	//              "                    "
		//	sprintf(strBuf, "SPEED:  %3d [%02hhX|%02hhX]", ((respBuf[2] << 8) + respBuf[3]) / 2, respBuf[2], respBuf[3]);
		//}
		//delay(ISORequestDelay);

		strip.show();
	}
	delay(5000);
}


void updateLeds() {
  strip.clear();
  uint8_t leds = (uint8_t)((rpms * N_PIXELS) / MAX_RPM);
  for (uint8_t k = 0; k < leds; k++) {
    strip.setPixelColor(k, 255, 255, 255);
  }
  strip.show();
}


bool initPulse() {
	uint8_t rLen;
	uint8_t req[2];
	uint8_t resp[3];

	Serial.end();
	
	// This is the ISO 14230-2 "Fast Init" sequence.
	digitalWrite(K_OUT, HIGH);
	delay(300);
	digitalWrite(K_OUT, LOW);
	delay(25);
	digitalWrite(K_OUT, HIGH);
	delay(25);

	Serial.begin(10400);

	// Start Communication is a single byte "0x81" packet.
	req[0] = 0x81;
	rLen = sendRequest(req, resp, 1, 3);

	delay(ISORequestDelay);
	// Response should be 3 bytes: 0xC1 0xEA 0x8F
	if ((rLen == 3) && (resp[0] == 0xC1) && (resp[1] == 0xEA) && (resp[2] == 0x8F)) {
		// Success, so send the Start Diag frame
		// 2 bytes: 0x10 0x80
		req[0] = 0x10;
		req[1] = 0x80;
		rLen = sendRequest(req, resp, 2, 3);
		
		// OK Response should be 2 bytes: 0x50 0x80
		if ((rLen == 2) && (resp[0] == 0x50) && (resp[1] == 0x80)) {
			return true;
		}
	}
	// Otherwise, we failed to init.
	return false;
}

// send a request to the ESC and wait for the response
// request = buffer to send
// response = buffer to hold the response
// reqLen = length of request
// maxLen = maximum size of response buffer
//
// Returns: number of bytes of response returned.
uint8_t sendRequest(const uint8_t *request, uint8_t *response, uint8_t reqLen, uint8_t maxLen) {
	uint8_t buf[16], rbuf[16];
	uint8_t bytesToSend;
	uint8_t bytesSent = 0;
	uint8_t bytesToRcv = 0;
	uint8_t bytesRcvd = 0;
	uint8_t rCnt = 0;
	uint8_t c, z;
	bool forMe = false;
	char radioBuf[32];
	uint32_t startTime;
	
	for (uint8_t i = 0; i < 16; i++) {
		buf[i] = 0;
	}
	// Zero the response buffer up to maxLen
	for (uint8_t i = 0; i < maxLen; i++) {
		response[i] = 0;
	}

	// Form the request:
	if (reqLen == 1) {
		buf[0] = 0x81;
	} else {
		buf[0] = 0x80;
	}
	buf[1] = ECUaddr;
	buf[2] = myAddr;

	if (reqLen == 1) {
		buf[3] = request[0];
		buf[4] = calcChecksum(buf, 4);
		bytesToSend = 5;
	} else {
		buf[3] = reqLen;
		for (z = 0; z < reqLen; z++) {
			buf[4 + z] = request[z];
		}
		buf[4 + z] = calcChecksum(buf, 4 + z);
		bytesToSend = 5 + z;
	}
	
	// Now send the command...
	serial_rx_off();
	for (uint8_t i = 0; i < bytesToSend; i++) {
		bytesSent += Serial.write(buf[i]);
		delay(ISORequestByteDelay);
	}
	serial_rx_on();
	
	// Wait required time for response.
	delay(ISORequestDelay);

	startTime = millis();

	// Wait for and deal with the reply
	while ((bytesRcvd <= maxLen) && ((millis() - startTime) < MAXSENDTIME)) {
		if (Serial.available()) {
			c = Serial.read();
			startTime = millis(); // reset the timer on each byte received

			delay(ISORequestByteDelay);

			rbuf[rCnt] = c;
			switch (rCnt) {
			case 0:
				// should be an addr packet either 0x80 or 0x81
				if (c == 0x81) {
					bytesToRcv = 1;
				} else if (c == 0x80) {
					bytesToRcv = 0;
				}
				rCnt++;
				break;
			case 1:
				// should be the target address
				if (c == myAddr) {
					forMe = true;
				}
				rCnt++;
				break;
			case 2:
				// should be the sender address
				if (c == ECUaddr) {
					forMe = true;
				} else if (c == myAddr) {
					forMe = false; // ignore the packet if it came from us!
				}
				rCnt++;
				break;
			case 3:
				// should be the number of bytes, or the response if its a single byte packet.
				if (bytesToRcv == 1) {
					bytesRcvd++;
					if (forMe) {
						response[0] = c; // single byte response so store it.
					}
				} else {
					bytesToRcv = c; // number of bytes of data in the packet.
				}
				rCnt++;
				break;
			default:
				if (bytesToRcv == bytesRcvd) {
					// must be at the checksum...
					if (forMe) {
						// Only check the checksum if it was for us - don't care otherwise!
						if (calcChecksum(rbuf, rCnt) == rbuf[rCnt]) {
							// Checksum OK.
							return(bytesRcvd);
						} else {
							// Checksum Error.
							return(0);
						}
					}
					// Reset the counters
					rCnt = 0;
					bytesRcvd = 0;
					
					// ISO 14230 specifies a delay between ECU responses.
					delay(ISORequestDelay);
				} else {
					// must be data, so put it in the response buffer
					// rCnt must be >= 4 to be here.
					if (forMe) {
						response[bytesRcvd] = c;
					}
					bytesRcvd++;
					rCnt++;
				}
				break;
			}
		}
	}

	return false;

}


// Checksum is simply the sum of all data bytes modulo 0xFF
// (same as being truncated to one byte)
uint8_t calcChecksum(uint8_t *data, uint8_t len) {
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crc + data[i];
	}
	return crc;
}


void serial_rx_on() {
	//UCSR0B |= (_BV(RXEN0));  //disable UART RX
	//Serial.begin(10400);		//setting enable bit didn't work, so do beginSerial
}

void serial_rx_off() {
	//UCSR0B &= ~(_BV(RXEN0));  //disable UART RX
}

void serial_tx_off() {
	//UCSR0B &= ~(_BV(TXEN0));  //disable UART TX
	//delay(20);                 //allow time for buffers to flush
}
