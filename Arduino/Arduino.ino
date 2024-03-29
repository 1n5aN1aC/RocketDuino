//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all logic for the RocketDuino
//
// TODO:
// 1. Increase transmission power to maximum.  (is min right now)
// 2. Mode for Serial Passthrough to GPS.  (To upload sat info, etc.)
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
#define CHUTE1 6  //Output for Chute Channel 1
#define CHUTE2 5  //Output for Chute Channel 2
#define LED    13 //Teensy Onboard LED
#define LED1   4  //PCB LED 1 (Green)
#define LED2   3  //PCD LED 2 (Yellow)
#define LORARX 9  //Lora_RX  (E32TX)
#define LORATX 10 //Lora_TX  (E32RX)
#define LORAUX 8  //Lora_AUX (Status)
#define LORAM0 12 //Lora_M0  (Mode Set)
#define LORAM1 11 //Lora_M1  (Mode Set)
//GPS  = Serial1
//Lora = Serial2

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
#define  FREQUENCY_TX_LORA       1000
elapsedMillis lastLoraTX;
LoRa_E32 e32ttl100(LORARX, LORATX, LORAUX, LORAM0, LORAM1, UART_BPS_RATE_9600);

// GPS Variables
TinyGPSPlus   gps;
TinyGPSCustom gpsStatus(gps, "GPGGA", 6);
#define  GPSBAUDRATE             115200
unsigned char extraSerialBuffer  [300];   //extra memory to dedocate to gps rx
int sizeOfExtraSerialBuffer      = 300;
bool     GPSforward              = false;
elapsedMillis lastGPSReceive;

struct __attribute__((packed)) packet_normal {
  uint32_t timeStamp;         //4 bytes
  uint8_t  currentMode;       //1 byte
  int16_t  currentAlt;        //2 bytes
  int16_t  startingAlt;       //2 bytes
  int16_t  maxAlt;            //2 bytes
  
  double   latitude;          //8 bytes
  double   longitude;         //8 bytes
  uint16_t gpsAlt;            //2 bytes
  uint16_t gpsSpeedMPS;       //2 bytes
  uint8_t  gpsSats;           //1 byte
  uint16_t gpsHDOP;           //2 bytes
  uint8_t  gps_mode;          //1 byte (only 2 bits needed)
  uint8_t  gps_valid;         //1 byte (only 1 bit needed)
  uint16_t gps_receivedSents; //2 bytes
  uint8_t  gps_checkFails;    //1 byte
  uint8_t  checksum;          //1 byte
};                   //TOTAL = 40 bytes, maximum 51 / 58 Bytes   (58 on-air) [Must wait 3-byte time between packets if <58bytes]

// Init Values
int16_t  maxAltitudeSeen    = -32768;   //Highest we've been to
int16_t  startingAltitude   = 0;   //what elevation we consider 'ground'
uint16_t failsafeAltitude   = 100; //minimum alt we need to reach to arm
uint8_t  extraDeploymentAlt = 25;  //Extra meters we have to fall below max alt before deployment (to be safe from jiggle)

//
// Run Initial setup Tasks
//
void setup() {
  // Init variables & objects
  pinMode(BUZZER, OUTPUT);
  pinMode(CHUTE1, OUTPUT);
  pinMode(CHUTE2, OUTPUT);
  pinMode(LED,    OUTPUT);
  pinMode(LED1,   OUTPUT);
  pinMode(LED2,   OUTPUT);
  pinMode(LORAM0, OUTPUT);
  pinMode(LORAM1, OUTPUT);
  digitalWrite(LED,    HIGH);  //LED on startup

  //Init USB serial port
  Serial.begin(57600); //USB
  Serial.println(F("[SER]  (USB)    Debug serial began.  (57600)"));

  //Init GPS serial port
  Serial1.begin(GPSBAUDRATE);
  Serial.println(F("[SER]  (GPS)    Gps serial began.") + String(GPSBAUDRATE) + ")");
  //We need to dedicate extra memory to the GPS receive buffer:
  Serial1.addMemoryForRead(extraSerialBuffer, sizeOfExtraSerialBuffer);

  //Init lora transmitter
  setup_LoRa();

  //Init Baro
  Serial.println(F("[BARO] (Config) Configuring Altimiter..."));
  bmp280.begin(SLEEP_MODE, BMP280_I2C_ALT_ADDR);  // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X2);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setTempOversampling(OVERSAMPLING_X1);    // Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16
  bmp280.setIIRFilter(IIR_FILTER_OFF);            // Options are IIR_FILTER_OFF, _2, _4, _8, _16
  bmp280.setTimeStandby(TIME_STANDBY_62MS);       // Options are TIME_STANDBY_05MS, _62MS, _125MS, _250MS, _500MS, _1000MS, 2000MS, 4000MS
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

  delay(500); //Wait for stuff to init
  digitalWrite(LED, LOW);  //LED off as startup finished
  Serial.println("[Info]          Start up Complete.");
}

void setup_LoRa() {
  //Serial2.begin(LORABAUDRATE);
  //Serial.println("LORA Serial Started" + String(LORABAUDRATE) + ")");

  //Connect to module
  Serial.println(F("[LoRa] (Config) Connecting to module..."));  // https://www.mischianti.org/2019/10/21/lora-e32-device-for-arduino-esp32-or-esp8266-library-part-2/
  e32ttl100.begin();                                             // Startup all pins and UART
  delay(200);

  //Fetch LoRa module information
  ResponseStructContainer cMi;
  cMi = e32ttl100.getModuleInformation();
  ModuleInformation mi = *(ModuleInformation*)cMi.data; // It's important get information pointer before all other operation
  cMi.close();
  Serial.print(F("[LoRa] (Config) Version:  "));
  Serial.println(mi.version, HEX);
  Serial.print(F("[LoRa] (Config) Features: "));
  Serial.println(mi.features, HEX);

  Serial.println(F("[LoRa] (Config) Fetching current config..."));
  ResponseStructContainer c;                                     // C stores the config
  c = e32ttl100.getConfiguration();                              // Load config from module
  Configuration configuration = *(Configuration*) c.data;        // Make local config object
  int numChanges = 0;

  //Communication channel (def 17H == 23d == 433MHz / 410 M + CHAN*1M)
  Serial.print(F("[LoRa] (Config) Communication channel: "));
  Serial.print(configuration.CHAN, HEX);
  Serial.print(F(" Wants: "));
  Serial.print(0x17, HEX);
  if (configuration.CHAN != 0x17) {
    configuration.CHAN = 0x17;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Forward Error Correction
  Serial.print(F("[LoRa] (Config) FEC: "));
  Serial.print(configuration.OPTION.fec);
  Serial.print(F(" Wants: "));
  Serial.print(FEC_1_ON);
  if (configuration.OPTION.fec != FEC_1_ON) {
    configuration.OPTION.fec = FEC_1_ON;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Broadcast/directed setting
  Serial.print(F("[LoRa] (Config) Fixed/Transparent: "));
  Serial.print(configuration.OPTION.fixedTransmission);
  Serial.print(F(" Wants: "));
  Serial.print(FT_FIXED_TRANSMISSION);
  if (configuration.OPTION.fixedTransmission != FT_FIXED_TRANSMISSION) {
    configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Pull ups enabled or not
  Serial.print(F("[LoRa] (Config) Pull-ups: "));
  Serial.print(configuration.OPTION.ioDriveMode);
  Serial.print(F(" Wants: "));
  Serial.print(IO_D_MODE_PUSH_PULLS_PULL_UPS);
  if (configuration.OPTION.ioDriveMode != IO_D_MODE_PUSH_PULLS_PULL_UPS) {
    configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //dBm transmission power (POWER_30 POWER_27 POWER_24 POWER_21)
  Serial.print(F("[LoRa] (Config) TX Power: "));
  Serial.print(configuration.OPTION.transmissionPower);
  Serial.print(F(" Wants: "));
  Serial.print(POWER_21);
  if (configuration.OPTION.transmissionPower != POWER_21) {
    configuration.OPTION.transmissionPower = POWER_21;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Wait time for wake up
  Serial.print(F("[LoRa] (Config) Wake time: "));
  Serial.print(configuration.OPTION.wirelessWakeupTime);
  Serial.print(F(" Wants: "));
  Serial.print(WAKE_UP_250);
  if (configuration.OPTION.wirelessWakeupTime != WAKE_UP_250) {
    configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Air data rate (2.4k)
  Serial.print(F("[LoRa] (Config) Air data rate: "));
  Serial.print(configuration.SPED.airDataRate);
  Serial.print(F(" Wants: "));
  Serial.print(AIR_DATA_RATE_010_24);
  if (configuration.SPED.airDataRate != AIR_DATA_RATE_010_24) {
    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Serial baud rate
  Serial.print(F("[LoRa] (Config) Serial Baud: "));
  Serial.print(configuration.SPED.uartBaudRate);
  Serial.print(F(" Wants: "));
  Serial.print(UART_BPS_9600);
  if (configuration.SPED.uartBaudRate != UART_BPS_9600) {
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Serial UART Parity
  Serial.print(F("[LoRa] (Config) Serial Parity: "));
  Serial.print(configuration.SPED.uartParity);
  Serial.print(F(" Wants: "));
  Serial.print(MODE_00_8N1);
  if (configuration.SPED.uartParity != MODE_00_8N1) {
    configuration.SPED.uartParity = MODE_00_8N1;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Address(high)
  Serial.print(F("[LoRa] (Config) Address (high): "));
  Serial.print(configuration.ADDH);
  Serial.print(F(" Wants: "));
  Serial.print(13);
  if (configuration.ADDH != 13) {
    configuration.ADDH = 13;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  //Address(low)
  Serial.print(F("[LoRa] (Config) Address (low): "));
  Serial.print(configuration.ADDL);
  Serial.print(F(" Wants: "));
  Serial.print(13);
  if (configuration.ADDL != 13) {
    configuration.ADDL = 13;
    numChanges++;
    Serial.print(F(" OPTION UPDATED."));
  }
  Serial.println();

  if (numChanges > 0) {
    Serial.println(F("[LoRa] (Config) Setting new config..."));
    e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE); // Save config permanently
    delay(500); //Wait for stuff to save and such
  } else {
    Serial.println(F("[LoRa] (Config) No config changes to save..."));
  }
  c.close();                                                         // Must close after opening

  delay(300); //Wait for stuff to init
  Serial.println(F("[LoRa] (Config) Complete."));
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
    lastModeUpdate = 0;

    // Mode 0 = Boot
    if (currentMode == 0) {
      //Once 20 seconds have passed on boot, lets call that the starting height.
      if (millis() > 30000) {
        startingAltitude = currentFilteredAltitude;
        currentMode = 1;
        Serial.println(F("[MODE] (Change) Mode 0->1"));
        digitalWrite(LED, LOW); //Onboard LED
      }
      //Also, if we haven't read the barometer recently, abort:
      if (millis() > 5000 && lastBarometerRead > 5000) {
        Serial.println(F("[ERROR]         More than 5000ms since last baro read!"));
        tone(BUZZER, 4000);
        digitalWrite(LED, HIGH);  //LED on error
        delay(90000);
      }
      return;
    }

    //Mode 1 = Below Failsafe Alt
    if (currentMode == 1) {
      //If we are above the failsafe height, go to flight mode.
      if (currentFilteredAltitude > startingAltitude + failsafeAltitude) {
        currentMode = 2;
        Serial.println(F("[MODE] (Change) Mode 1->2"));
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
        Serial.println(F("[MODE] (Change) Mode 2->3"));
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
  while (Serial1.available()) {
    gps.encode(Serial1.read());
    lastGPSReceive = 0;
  }
}

// Buzzes depending on current mode
// Updates GPSmode led
// Updates flight mode LED
void update_buzzer() {
  if (lastBuzzerUpdate > FREQUENCY_UPDATE_BUZZER) {
    //Update buzzer
    lastBuzzerUpdate = 0;
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
    if (gps.location.age() > 1500) {
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
  while (Serial2.available()) {
    byte b = Serial2.read();
  }
}

void update_lora_send() {
  if (lastLoraTX > FREQUENCY_TX_LORA) {
    lastLoraTX = 0;

    send_packet_normal();
  }
}

void send_packet_normal() {
  //When was the last time we got GPS packets?
  uint8_t gps_valid = 0;
  if (gps.altitude.age() > 1050) {
    gps_valid += 1;    //TinyGPS++ says it wasn't updated within last second.
  }
  if (lastGPSReceive > 1000) {
    gps_valid += 2;    //We haven't received any gps serial to even pass it on to TinyGPS++.
  }
  if (!gps.altitude.isValid()) {
    gps_valid += 4;    //Something wrong with the altitude fix?
  }

//  Serial.print(F("[DEBUG] "));
//  Serial.print(static_cast<int16_t>(currentFilteredAltitude));
//    Serial.print(",");
//  Serial.print(maxAltitudeSeen);
//    Serial.print(",");
//  Serial.print(startingAltitude);
//    Serial.print(",");
//  Serial.print(currentMode);
//    Serial.print(",");
//  Serial.print(gps.location.lat());
//    Serial.print(",");
//  Serial.print(gps.location.lng());
//    Serial.print(",");
//  Serial.print(gps.altitude.meters());
//    Serial.print(",");
//  Serial.print(gps.speed.mps());
//    Serial.print(",");
//  Serial.print(gps.satellites.value());
//    Serial.print(",");
//  Serial.print(static_cast<int16_t>(gps.hdop.value()));
//    Serial.print(",");
//  Serial.print(gpsStatus.value());
//    Serial.print(",");
//  Serial.print(gps_valid);
//  Serial.println();

  packet_normal packet = {
    millis(),
    currentMode,
    static_cast<int16_t>(currentFilteredAltitude),
    startingAltitude,
    maxAltitudeSeen,
    gps.location.lat(),
    gps.location.lng(),
    static_cast<uint16_t>(gps.altitude.meters()),
    static_cast<uint16_t>(gps.speed.mps()),
    static_cast<uint8_t> (gps.satellites.value()),
    static_cast<uint16_t>(gps.hdop.value()),
    static_cast<uint8_t> (String(gpsStatus.value()).toInt()),
    gps_valid,
    static_cast<uint16_t>(gps.passedChecksum()),
    static_cast<uint8_t> (gps.failedChecksum()),
    0 //checksum
  };
  //Serial.println(sizeof(packet));

  //Calculate checksum and add to packet
  uint8_t sum = 0;
  uint8_t *p = (uint8_t *)&packet;
  for (uint8_t i=0; i<sizeof(packet); i++) {
      sum += p[i];
  }
  packet.checksum = sum;

  //Serial.write(reinterpret_cast<char*>(&packet), sizeof packet);
  //Serial2.write(reinterpret_cast<char*>(&packet), sizeof packet);
  e32ttl100.sendFixedMessage(0,3,0x17,&packet, sizeof(packet));
}