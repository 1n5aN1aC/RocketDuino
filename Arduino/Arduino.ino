//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
#include "Arduino.h"
#include <Wire.h>
#include <BMP280_DEV.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>

//LoRa Hardware parameters
#define E32_TTL_1W
#define FREQUENCY_433
#include "LoRa_E32.h"

// Pin designations
#define BUZZER 17 //Buzzer Pin
#define CHUTE1 8  //Output for Chute Channel 1
#define CHUTE2 7  //Output for Chute Channel 1
#define LED    13 //Teensy Onboard LED
#define LED1   4  //PCB LED 1 (Green)
#define LED2   3  //PCD LED 2 (Yellow)


// Barometer Variables
BMP280_DEV bmp280; // Define barometer object. Basic I2C SCL & SDA
SimpleKalmanFilter altitudeFilterObject(4, 1, 0.015); //Measurement Uncertainty, Estimation Uncertainty, Noise
float    temporaryAltitude       = 0;
float    currentFilteredAltitude = 0;
elapsedMillis lastBarometerRead;

// Mode variables
#define  FREQUENCY_UPDATE_MODE   250
uint8_t  currentMode             = 0;
elapsedMillis lastModeUpdate;

// Buzzer Variables
#define  FREQUENCY_UPDATE_BUZZER 3000
elapsedMillis lastBuzzerUpdate;

// Serial Variables
#define  LORABAUDRATE            57600
#define  FREQUENCY_TX_LORA       350
elapsedMillis lastLoraTX;
LoRa_E32 e32ttl100(&Serial1);

// GPS Variables
TinyGPSPlus   gps_object;
TinyGPSCustom gpsStatus(gps_object, "GPGGA", 6);
#define  GPSBAUDRATE             57600
bool     GPSforward              = false;
elapsedMillis lastGPSReceive;

typedef struct __attribute__((packed)) packet_normal { //Maximum 51 / 58 Bytes
  int16_t  currentAlt;        //2 bytes
  int16_t  maxAlt;            //2 bytes
  int16_t  startingAlt;       //2 bytes
  uint8_t  currentMode;       //1 byte
  float    latitude;          //4 bytes
  float    longitude;         //4 bytes
  uint16_t gpsAlt;            //2 bytes
  uint16_t gpsSpeedMPS;       //2 bytes
  uint8_t  sats;              //1 byte
  uint8_t  gps_mode;          //1 byte (only 2 bits needed)
  uint8_t  gps_recent;        //1 byte (only 1 bit needed)
};

// Saved Values
int16_t  maxAltitudeSeen    = -32768;   //Highest we've been to
int16_t  startingAltitude   = 0;   //what elevation we consider 'ground'
uint16_t failsafeAltitude   = 100; //minimum alt we need to reach to arm
uint8_t  extraDeploymentAlt = 25;  //Extra meters we have to fall below max alt before deployment (to be safe from jiggle)

//
// Run Initial setup Tasks
//
void setup() {
  // Initialize variables & objects
  pinMode(BUZZER, OUTPUT);
  pinMode(CHUTE1, OUTPUT);
  pinMode(CHUTE2, OUTPUT);
  pinMode(LED,    OUTPUT);
  pinMode(LED1,   OUTPUT);
  pinMode(LED2,   OUTPUT);
  digitalWrite(LED, HIGH);  //LED on startup
  
	//Initialize both serial ports
  Serial.println(57600); //USB
  Serial1.begin(LORABAUDRATE);
  Serial2.begin(GPSBAUDRATE);

  delay(500); //Wait for stuff to init

  //Init LoRa                                             // https://www.mischianti.org/2019/10/21/lora-e32-device-for-arduino-esp32-or-esp8266-library-part-2/
  e32ttl100.begin();                                      // Startup all pins and UART
  ResponseStructContainer c;                              // C stores the config
  c = e32ttl100.getConfiguration();                       // Load config from module
  Configuration configuration = *(Configuration*) c.data; // Make local config object
  //configuration.ADDL = 0x0;                                           // 0x0; (def: 00H /00H-FFH) // First part of address
  //configuration.ADDH = 0x1;                                           // 0x1; (def: 00H /00H-FFH) // Second part of address
  configuration.CHAN = 0x17;                                            // Communication channel (def 17H == 23d == 433MHz / 410 M + CHAN*1M)
  configuration.OPTION.fec = FEC_1_ON;                                  // Forward Error Correction?
  configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION; // Broadcast transmissions
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;     // Enable pull-ups on serial
  configuration.OPTION.transmissionPower = POWER_21;                    // dBm transmission power (POWER_30 POWER_27 POWER_24 POWER_21)
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;                // Wait time for wake up
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;                // Air data rate (2.4k)
  configuration.SPED.uartBaudRate = UART_BPS_57600;                     // Communication baud rate - 9600bps (default)
  configuration.SPED.uartParity = MODE_00_8N1;                          // Serial UART Parity

  e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE); // Save config permanently
  c.close();                                                         // Must close after opening
  
  delay(500); //Wait for stuff to init

  //Init BMP
  bmp280.begin(SLEEP_MODE, BMP280_I2C_ALT_ADDR);  // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X2);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setTempOversampling(OVERSAMPLING_X1);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setIIRFilter(IIR_FILTER_OFF);            // Options are IIR_FILTER_OFF, _2, _4, _8, _16
  bmp280.setTimeStandby(TIME_STANDBY_62MS);       // Options are TIME_STANDBY_05MS, _62MS, _125MS, _250MS, _500MS, _1000MS, 2000MS, 4000MS
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

  delay(500); //Wait for stuff to init
  digitalWrite(LED, LOW);  //LED off as startup finished
}

//
// Main Loop Tasks
//
void loop() {
	//Gather new (baro) data
  update_barometer();
  
	//Calculate on data (flight modes)
  calculate_mode();
  
  //Update Serial / wireless
  update_lora_send();
  
  //Handle incoming gps data
  update_gps_receive();

  //Handle incoming lora packets
  update_lora_receive();
  
  //Update buzzer / LEDs
  update_buzzer();
}

//
// Queries the barometer for elevation, stores it, and updates the running average
void update_barometer() {
  if (bmp280.getAltitude(temporaryAltitude)) {   // Check if the measurement is complete
    currentFilteredAltitude = altitudeFilterObject.updateEstimate(temporaryAltitude);
    
    //Update max altitude if needed and out of startup mode
    if (currentMode > 0 && currentFilteredAltitude > maxAltitudeSeen) {
      maxAltitudeSeen = currentFilteredAltitude;
    }

    //Take note that we've sucessfully read the altitude:
    lastBarometerRead = 0;
  }
}

//
// Mode 0 = Boot
// Mode 1 = Below Failsafe Alt
// Mode 2 = Flying
// Mode 3 = Eject!
//
void calculate_mode() {
  if (lastModeUpdate > FREQUENCY_UPDATE_MODE) {
    lastModeUpdate = lastModeUpdate - FREQUENCY_UPDATE_MODE;
    
    // Mode 0 = Boot
    if (currentMode == 0) {
      //Once 20 seconds have passed on boot, lets call that the starting height.
      if (millis() > 30000) {
        startingAltitude = currentFilteredAltitude;
        currentMode = 1;
        Serial.println("Mode 0->1");
        digitalWrite(LED, LOW); //Onboard LED
      }
      //Also, if we haven't read the barometer recently, abort:
      if (millis() > 5000 && lastBarometerRead > 1000) {
        tone(BUZZER, 4000);
        delay(90000);
      }
      return;
    }
    
    //Mode 1 = Below Failsafe Alt
    if (currentMode == 1) {
      //If we are above the failsafe height, go to flight mode.
      if (currentFilteredAltitude > startingAltitude + failsafeAltitude) {
        currentMode = 2;
        Serial.println("Mode 1->2");
      }
      return;
    }
    
    //Mode 2 = Flying
    if (currentMode == 2) {
      //If we're more than the safety distance below max alt, go to eject mode
      if (maxAltitudeSeen > currentFilteredAltitude + extraDeploymentAlt) {
        //WAIT!! If we're below the failsafe altitude, don't.
        if (currentFilteredAltitude < startingAltitude + failsafeAltitude) {
          return;
        }
        currentMode = 3;
        Serial.println("Mode 2->3");
        digitalWrite(CHUTE1, HIGH);
        digitalWrite(CHUTE2, HIGH);
        delay(500);
        digitalWrite(CHUTE1, LOW);
        digitalWrite(CHUTE2, LOW);
      }
      return;
    }
    
    //Mode 3 = Eject & Recovery
    if (currentMode == 3) {
      //We're already done, so do nothing I guess?
    }
  }
}

//Process any incoming bytes from the GPS
void update_gps_receive() {
  while (Serial2.available() > 0) {
    byte b = Serial2.read();
    gps_object.encode(b);
    lastGPSReceive = 0;
    
    //If GPSforward is enabled, then pass all GPSdata through lora to the basestation
    if (GPSforward) {
      Serial1.write(b);
    }
  }
}

// Buzzes depending on current mode
// Updates GPSmode led
// Updates flight mode LED
void update_buzzer() {
  if (lastBuzzerUpdate > FREQUENCY_UPDATE_BUZZER) {
    //Update buzzer
    lastBuzzerUpdate = lastBuzzerUpdate - FREQUENCY_UPDATE_BUZZER;
    if (currentMode == 0) {
      digitalWrite(LED1, HIGH); // Status LED on for duration
      tone(BUZZER, 2000, 75);   // Send 2KHz sound signal for 75ms
      delay(125);
      tone(BUZZER, 4000, 75);   // Send 4KHz sound signal for 75ms
      delay(125);
      tone(BUZZER, 2000, 25);   // Send 2KHz sound signal for 25ms
      digitalWrite(LED1, LOW);
    } else if (currentMode == 1) {
      digitalWrite(LED1, HIGH); // Status LED on
      tone(BUZZER, 4000, 25);   // Send 4KHz sound signal for 25ms
    } else if (currentMode == 3) {
      digitalWrite(LED1, LOW);  // Status LED off
      tone(BUZZER, 4000, 100);  // Send 4KHz sound signal for 100ms
      delay(200);
      tone(BUZZER, 4000, 100);  // Send 4KHz sound signal for 100ms
    }

    //Update GPSmode LED
    uint8_t gps_mode;
    if (gps_object.location.age() > 1500) {
      gps_mode = 0;
    } else {
      gps_mode = atoi(gpsStatus.value());
    }
    if (gps_mode == 2) {        //DGPS
      digitalWrite(LED2, HIGH);
    } else if (gps_mode == 1) { //2D-3D Fix
      bool current = digitalRead(LED2);
      if (current) {
        digitalWrite(LED2, LOW);
      } else {
        digitalWrite(LED2, HIGH);
      }
    } else {                    //No fix
      digitalWrite(LED2, LOW);
    }
  }
}

void update_lora_receive() {
  while (Serial1.available() > 0) {
    byte b = Serial2.read();
  }
}

void update_lora_send() {
  if (lastLoraTX > FREQUENCY_TX_LORA) {
    lastLoraTX = lastLoraTX - FREQUENCY_TX_LORA;

    send_packet_normal();
  }
}

void send_packet_normal() {
  //When was last gps data?  If it was valid, but longer than a couple seconds, override the gps info pkt:
  uint8_t gps_mode;
  if (atoi(gpsStatus.value()) != 0 && gps_object.location.age() > 1500) {
    gps_mode = 3;
  } else {
    gps_mode = atoi(gpsStatus.value());
  }

  //When was the last time we got GPS packets?
  uint8_t gps_recent = 0;
  if (lastGPSReceive < 1000) {
    gps_recent = 1;
  }
  
  packet_normal packet = {static_cast<int16_t>(currentFilteredAltitude), maxAltitudeSeen, startingAltitude, currentMode, gps_object.location.lat(), gps_object.location.lng(), gps_object.altitude.meters(), gps_object.speed.mps(), gps_object.satellites.value(), gps_mode, gps_recent};
  Serial1.write(reinterpret_cast<char*>(&packet), sizeof packet);
}
