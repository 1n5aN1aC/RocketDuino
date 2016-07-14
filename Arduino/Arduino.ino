#include <TinyGPS++.h>
#define ROCKETDUINO_VERSION "0.1.2"

// Baud rate settings
#define GPS_BAUD 9600
#define TX_BAUD 57600
#define ACTUAL_TX_BAUD 1200
#define TARGET_TX_BAUD 1000

// Frequency of transmissions
#define TELEMETRY_FREQUENCY 500
#define STATUS_FREQUENCY 1000

// Pin designations
#define VSENSE_PIN 12


// The TinyGPS++ object(s)
TinyGPSPlus gps;
TinyGPSCustom VDOP   (gps, "GPGSA", 18); // Vertical dillution Of Presicion

//Time counters
unsigned long lastTelemetryTX = 0UL;
unsigned long lastStatusTX    = 0UL;
unsigned long lastRX          = 0UL;

//counters for last packet statistics send
unsigned int lastBadSum  = 0;
unsigned int lastGoodSum = 0;
unsigned int lastWithFix = 0;


//
// Run Initial setup Tasks
//
// 0. Initialize serial buffers
// 1. Send Version information
//
void setup() {
  // initialize both serial ports:
  Serial.begin(TX_BAUD);
  Serial1.begin(GPS_BAUD);

  Serial.println();
  Serial.print(F("RocketDuino v."));
  Serial.print(ROCKETDUINO_VERSION);
  Serial.print(F(" & TinyGPS++ v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
}


//
// Main Loop Tasks
//
// 1. Recieve & Process GPS data
// 2. Recieve & Process Ground Control Packets
// 3. Send Telemetry if needed
// 4. Send Status information if needed.
//
void loop() {
  //If there is any new GPS information, read & process it
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  //If we have received a character from ground control, process it.
  if (Serial.available() )
  {
     ch = Serial.read();
	 process_command(ch);
  }

  //If we need to send a new telemetry packet, do so.
  if (millis() - lastTelemetryTX > TELEMETRY_FREQUENCY)
    telemetryTX();

  //If we need to send a new status packet, do so.
  if (millis() - lastStatusTX > STATUS_FREQUENCY)
    statusTX();
}


//
// Sends telemetry information
// 
// (LAT, LONG, ALT, SOG, COG, Pressure)
//
void telemetryTX() {
  Serial.print("T,");
  
  Serial.print(gps.location.lat(), 6);
  Serial.print(",");
  Serial.print(gps.location.lng(), 6);

  Serial.print(",");
  Serial.print(gps.altitude.meters());

  Serial.print(",");
  Serial.print(gps.speed.kmph());

  Serial.print(",");
  Serial.print(gps.course.deg());

  Serial.print(",");
  //Put pressure here?

  //Finally, update last Packet variable
  Serial.println();
  lastTelemetryTX = millis();
}


//
// Status update Packet:
// 
// S,21594500,000000,6,165,165,532,2/1/0
// S = This is a status packet
//   21594500 = GPS time
//            0 = Age of last ping from ground control
//             0 = Time validity byte (0-9 = seconds since update.  X=invalid / >10 seconds)
//              0 = Location validity byte
//               0 = Altitude validity byte
//                0 = Speed validity byte
//                 0 = Course validity byte
//                   6 = Satellites used for fix
//                     165 = HDOP (Horrizontal dillution of precision)
//                         165 = VDOP (Vertical dillution of precision)
//                             532 = battery input voltage.  In centivolts (5.32v) (After diode, before everything else)
//                                 2/1/0 = 2 new packets with valid fix / 1 new packet with no fix / 0 packets with bad checksums
//
void statusTX() {
  //Static Packet 
  Serial.print("S,");

  //Current Time
  Serial.print(gps.time.value());
  Serial.print (",");

  //Uplink Status
  timeSince = millis() - lastRX;
  if (timeSince < 10000) {
    Serial.print( timeSince / 1000 );
  } else {
    Serial.print("X");
  }

  //Time Status
  if (gps.time.isValid() && gps.time.age() < 10000) {
    Serial.print( gps.time.age() / 1000 );
  } else {
    Serial.print("X");
  }

  //Location Status
  if (gps.location.isValid() && gps.location.age() < 10000) {
    Serial.print( gps.location.age() / 1000 );
  } else {
    Serial.print("X");
  }

  //Altitude Status
  if (gps.altitude.isValid() && gps.altitude.age() < 10000) {
    Serial.print( gps.altitude.age() / 1000 );
  } else {
    Serial.print("X");
  }

  //Speed Status
  if (gps.speed.isValid() && gps.speed.age() < 10000) {
    Serial.print( gps.speed.age() / 1000 );
  } else {
    Serial.print("X");
  }

  //Course Status
  if (gps.course.isValid() && gps.course.age() < 10000) {
    Serial.print( gps.course.age() / 1000 );
  } else {
    Serial.print("X");
  }

  //Number of satellites
  Serial.print(",");
  if (gps.satellites.age() < 10000)
    Serial.print(gps.satellites.value());
  else
    Serial.print(0);

  //Horizontal Dillution of Precision
  Serial.print(",");
  Serial.print(gps.hdop.value());

  //Vertical dillution of Precision
  Serial.print(",");
  Serial.print(VDOP.value());

  //Battery Volatage
  Serial.print(",");
  float sample = analogRead(VSENSE_PIN);
  float voltage = (sample * 5.015) / 1024.0 * 5.7 * 100;   //5.7 = voltage divider ratio
  Serial.print( (int) voltage );

  //Number of (new) good packets with fixes
  Serial.print(",");
  Serial.print( (gps.sentencesWithFix() - lastWithFix) );

  //Number of (new) good packets with NO fix
  Serial.print("/");
  Serial.print( (gps.passedChecksum() - lastGoodSum) - (gps.sentencesWithFix() - lastWithFix) );

  //Number of (new) bad packets
  Serial.print("/");
  Serial.print( gps.failedChecksum() - lastBadSum );

  //Now update the 'lasts'
  lastGoodSum = gps.passedChecksum();
  lastBadSum = gps.failedChecksum();
  lastWithFix = gps.sentencesWithFix();

  //Finally, update last Packet variable
  Serial.println();
  lastStatusTX = millis();
}

//
// Processes a command from ground control.
//
// Valid packets:
// P = 'ping' Keeps track of last RX from Ground Control
//
void process_command(Byte ch) {
  if (ch == 'P') {
    lastRX = millis();
  }
  else {
    //something went wrong?  Bad packet.
  }
}
