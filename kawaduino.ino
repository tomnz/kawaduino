/*
KAWADUINO

Reads data from a Kawasaki ECU Diagnostic Port, and updates an LED string based on RPMs
read from the ECU. This code is adapted from code originally written by Greebo on page 2
of the following thread:

http://ecuhacking.activeboard.com/t56234221/kds-protocol/

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
#include <EEPROM.h>
#include "kawaduino.h"

#define K_OUT 1 // K Output Line - TX on Arduino
#define K_IN 0 // K Input Line - RX on Arduino
#define SERIAL_ON 3

// Animation settings
#define REFRESH_MICROS 30000
// Mode 1
#define MAX_RPM 6000
#define MIN_COL 160
#define MAX_COL 255
#define MIN_BRIGHT 20
#define MAX_BRIGHT 255
// Mode 2
#define MAX_MPH2 160
#define MPH_DOTS 3
#define MPH_DOT_SIZE 3
#define MIN_MPH2 4
#define BACKGROUND_MAX 0.4
#define BACKGROUND_INCREASE 0.1
#define BACKGROUND_DECREASE 0.7
#define BACKGROUND_COLOR 180
#define PROGRESS_MPH2_MULT 1
#define DAMPED_MPH2_FACTOR 10

// LED settings
#define N_PIXELS 60
#define LED_PIN 6
#define DIAG_LED1 4
#define DIAG_LED2 5
#define BOARD_LED 13

// Modes
#define N_MODES 2
#define MODE_ADDR 0
#define BTN_PIN 7

// Startup
#define AVG_CYCLES 50

// Timings
#define MAXSENDTIME 2000 // 2 second timeout on KDS comms.
const uint32_t ISORequestByteDelay = 10;
const uint32_t ISORequestDelay = 40; // Time between requests.

// Addresses
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

// Animation variables
unsigned long lastFrameTime = 0;
// Mode 1
uint32_t rpms = 0;
uint32_t dampedRpms = 0;
// Mode 2
uint32_t mph2 = 0;
float dampedMph2 = 0;
float backgroundLevel = 0;
float progress = 0;

// Modes
uint8_t mode = 0;
boolean btnPressed = false;

void setup() {
  // Setup pins
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
#ifdef SERIAL_ON
  pinMode(SERIAL_ON, OUTPUT);
#endif
#ifdef DIAG_LED1
  pinMode(DIAG_LED1, OUTPUT);
#endif
#ifdef DIAG_LED2
  pinMode(DIAG_LED2, OUTPUT);
#endif
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LOW);

  // Show startup routine
  strip.begin();

  startupLeds();
  
  // Read mode
  mode = EEPROM.read(MODE_ADDR);
  
  lastFrameTime = micros();
  
  // Determine duration of updateLeds()
  determineAverage();
  
  strip.clear();
  strip.show();
}

void loop() {
  // Init blink
#ifdef SERIAL_ON
  digitalWrite(SERIAL_ON, HIGH);
#endif
  digitalWrite(BOARD_LED, HIGH);
  delay(200);
  digitalWrite(BOARD_LED, LOW);
  delay(200);
  digitalWrite(BOARD_LED, HIGH);
  delay(200);
  digitalWrite(BOARD_LED, LOW);
        
  uint8_t cmdSize;
  uint8_t cmdBuf[6];
  uint8_t respSize;
  uint8_t respBuf[12];
  uint8_t ect;

  if (!ECUconnected) {
    // Start KDS comms
    ECUconnected = initPulse();

    if (ECUconnected) {
      // Show we're connected
      digitalWrite(BOARD_LED, HIGH);
    } else {
    }
  }
  
  // Endless loop.
  boolean diag1On = true;
  while (ECUconnected) {
    doButton();
    
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
    //      but the server does not specify the reason of the rejection

    // Grab RPMs
    if (mode == 1) {
      for (uint8_t i = 0; i < 5; i++) respBuf[i] = 0;
      
      // Request RPM is register: 0x09
      cmdBuf[1] = 0x09;
      respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
      if (respSize == 4) {
        // Formula for RPMs from response
        rpms = respBuf[2] * 100 + respBuf[3];
        
        // Conform RPMs
        rpms = max(min(rpms, MAX_RPM), 0);
  
  #ifdef DIAG_LED1
        // Diagnostic blink to show update rate
        if (diag1On) {
          digitalWrite(DIAG_LED1, HIGH);
        }
        else {
          digitalWrite(DIAG_LED1, LOW);
        }
        diag1On = !diag1On;
  #endif
      }
      else if (respSize == 0) {
        ECUconnected = false;
        break;
      }
      delayLeds(ISORequestDelay, true);
    }
    
    // Gram Speed
    if (mode == 2) {
      for (uint8_t i = 0; i < 5; i++) respBuf[i] = 0;

      // Request Speed is register: 0x0C
      cmdBuf[1] = 0x0C;
      respSize = sendRequest(cmdBuf, respBuf, cmdSize, 12);
      if (respSize == 4) {
        // NOTE: Actual MPH is this value halved, but we want to
        // keep full available resolution
        mph2 = (respBuf[2] << 8) + respBuf[3];

  #ifdef DIAG_LED1
        // Diagnostic blink to show update rate
        if (diag1On) {
          digitalWrite(DIAG_LED1, HIGH);
        }
        else {
          digitalWrite(DIAG_LED1, LOW);
        }
        diag1On = !diag1On;
  #endif
      }
      else if (respSize == 0) {
        ECUconnected = false;
        break;
      }
      delayLeds(ISORequestDelay, true);
    }

  }

  // Housekeeping
  digitalWrite(BOARD_LED, LOW);
#ifdef SERIAL_ON
  digitalWrite(SERIAL_ON, LOW);
#endif
  strip.clear();
  strip.show();

  delay(5000);
}


unsigned long avg = 0;
#ifdef DIAG_LED2
boolean diag2On = false;
#endif

// One-time function to measure average runtime of
// updateLeds() - called at startup
void determineAverage() {
  boolean oldECUconnected = ECUconnected;
  uint32_t oldRpms = rpms;
  uint32_t oldMph2 = mph2;
  ECUconnected = true;
  unsigned long start = micros();
  
  for (int i = 0; i < AVG_CYCLES; i++) {
    // Update these values each time to make sure the function
    // is working as hard as possible
    rpms = map(i, 0, AVG_CYCLES, 0, MAX_RPM);
    dampedRpms = rpms;
    mph2 = map(i, 0, AVG_CYCLES, 0, MAX_MPH2);
    dampedMph2 = mph2;
    updateLeds();
  }
  avg = (micros() - start) / AVG_CYCLES;
  
  rpms = oldRpms;
  dampedRpms = oldRpms;
  mph2 = oldMph2;
  dampedMph2 = oldMph2;
  ECUconnected = oldECUconnected;
}


boolean doButton() {
#ifdef BTN_PIN
  // Check the button
  if (digitalRead(BTN_PIN) == HIGH) {
    if (!btnPressed) {
      // This is the first time we're seeing the press
      btnPressed = true;
      
      // Increment the mode
      mode++;
      if (mode > N_MODES) {
        mode = 1;
      }
      
      // Save it
      EEPROM.write(MODE_ADDR, mode);
      
      // Calculate the average again
      determineAverage();
      
      // Reset
      resetLeds();
      
      return true;
    }
  } else {
    btnPressed = false;
  }
#endif
  return false;
}

// Custom delay routine that updates LEDs while idle
void delayLeds(unsigned long ms, boolean safe) {
  unsigned long last = micros();
  unsigned long lastUpdate = 0;
  unsigned long first = last;

  // Run as long as we haven't exceeded given ms
  while ((last - first) < ms * 1000) {
    unsigned long curr = micros();
    
    // Refresh the lights if we go over a given interval, and we'll have time
    // Note that this conservatively will NOT run updateLeds if it doesn't look
    // like there will be enough time to complete
    boolean changed = false;
    if (curr - lastUpdate > REFRESH_MICROS && ((curr - first) + avg*4 < ms * 1000)) {
      if (!safe) {
        changed = doButton();
      }
      updateLeds();
      
      last = micros();
      lastUpdate = last;
      if (!changed) {
        if (avg == 0) {
          avg = last - curr;
        } else {
          avg = (avg * 15 + (last - curr)) >> 4;
        }
      }
    } else {
      last = curr;
    }
  }
}


void resetLeds() {
  strip.clear();
  strip.setBrightness(255);
  strip.show();
  backgroundLevel = 0;
}


// Show the next frame on the LEDs
void updateLeds() {
  unsigned long currTime = micros();
  unsigned long frameTime = currTime - lastFrameTime;
  lastFrameTime = currTime;
  
  if (!ECUconnected) {
    return;
  }
  
#ifdef DIAG_LED2
  // Diagnostic blink
  if (diag2On) {
    digitalWrite(DIAG_LED2, HIGH);
  }
  else {
    digitalWrite(DIAG_LED2, LOW);
  }
  
  diag2On = !diag2On;
#endif

  switch(mode) {
    case 1:
      doMode1(frameTime);
      break;
    case 2:
      doMode2(frameTime);
      break;
  }
}


// Show frame for mode 1
void doMode1(unsigned long frameTime) {
  // Uncomment for test RPMs
  //rpms += 100;
  //if (rpms > MAX_RPM) {
  //  rpms = 0;
  //}

  // Update rpms
  dampedRpms = (dampedRpms * 7 + rpms) >> 3;
  
  // Set brightness
  strip.setBrightness(map(dampedRpms, 0, MAX_RPM, MIN_BRIGHT, MAX_BRIGHT));
  
  // Grab color for RPM
  Color col = wheel(map(dampedRpms, 0, MAX_RPM, MIN_COL, MAX_COL));

  // Display
  strip.clear();
  for (uint8_t k = 0; k < N_PIXELS; k++) {
    strip.setPixelColor(k, col.r, col.g, col.b);
  }
  
  // Random pixel for visual refresh representation
  //strip.setPixelColor(random(N_PIXELS), 255, 255, 255);
  
  strip.show();
}

// Show frame for mode 2
void doMode2(unsigned long frameTime) {
  strip.clear();
  strip.setBrightness(255);
  float frameSecs = (float)frameTime / 1000000;
  
  // Update mph
  dampedMph2 = (dampedMph2 * DAMPED_MPH2_FACTOR + mph2) / (DAMPED_MPH2_FACTOR + 1);

  // Update the background
  if (mph2 > MIN_MPH2) {
    backgroundLevel -= (float)BACKGROUND_DECREASE * frameSecs;
  } else {
    backgroundLevel += (float)BACKGROUND_INCREASE * frameSecs;
  }
  backgroundLevel = max(0, min(backgroundLevel, BACKGROUND_MAX));

  // Show background
  if (backgroundLevel > 0) {
    Color bgBase = wheel(BACKGROUND_COLOR);
    uint32_t bg = Color((float)bgBase.r * backgroundLevel, (float)bgBase.g * backgroundLevel, (float)bgBase.b * backgroundLevel).toUint32();
    
    for (int i = 0; i < N_PIXELS; i++) {
      strip.setPixelColor(i, bg);
    }
  }
  
  // Increase progress
  progress += dampedMph2 * (float)PROGRESS_MPH2_MULT * frameSecs;
  while (progress > N_PIXELS - 1) {
    progress -= N_PIXELS;
  }
  
  // Show mph points
  if (dampedMph2 > MIN_MPH2) {
    // Get the color for the dots
    uint32_t col = wheel(map(dampedMph2, 0, MAX_MPH2, MIN_COL, MAX_COL)).toUint32();
    for (int p = 0; p < MPH_DOTS; p++) {
      // Get the dot position
      int pos = (int)progress + N_PIXELS * p / MPH_DOTS;
      if (pos > N_PIXELS - 1) {
        pos -= N_PIXELS;
      }

      for (int i = 0; i < MPH_DOT_SIZE; i++) {
        int n = pos + i;
        if (n > N_PIXELS - 1) {
          n -= N_PIXELS;
        }
        
        strip.setPixelColor(n, col);
      }
    }
  }
  
  strip.show();
}


// Run startup routine on LEDs (purely cosmetic!)
void startupLeds() {
  strip.clear();
  strip.setBrightness(255);
  
  // Show
  for (uint8_t i = 0; i < N_PIXELS; i++) {
    strip.setPixelColor(i, wheel(i * 255 / N_PIXELS).toUint32());
    strip.show();
    delay(1000 / N_PIXELS);
  }

  // Hide
  for (uint8_t i = 0; i < N_PIXELS; i++) {
    strip.setPixelColor(i, 0);
    strip.show();
    delay(1000 / N_PIXELS);
  }
        
  strip.clear();
  strip.show();
}


// Initialize connection to ECU
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


// Send a request to the ECU and wait for the response
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
  for (uint8_t i = 0; i < bytesToSend; i++) {
    bytesSent += Serial.write(buf[i]);
    delay(ISORequestByteDelay);
  }
  
  // Wait required time for response.
  delayLeds(ISORequestDelay, false);
  
  startTime = millis();
  
  // Wait for and deal with the reply
  while ((bytesRcvd <= maxLen) && ((millis() - startTime) < MAXSENDTIME)) {
    if (Serial.available()) {
      c = Serial.read();
      startTime = millis(); // reset the timer on each byte received

      delayLeds(ISORequestByteDelay, true);

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
          delayLeds(ISORequestDelay, true);
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


// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
Color wheel(byte wheelPos) {
  if(wheelPos < 85) {
    return Color(255 - wheelPos * 3, 0, wheelPos * 3);
  } else if(wheelPos < 170) {
    wheelPos -= 85;
    return Color(0, wheelPos * 3, 255 - wheelPos * 3);
  } else {
    wheelPos -= 170;
    return Color(wheelPos * 3, 255 - wheelPos * 3, 0);
  }
}

