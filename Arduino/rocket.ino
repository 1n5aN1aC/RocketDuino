#include <TinyGPS++.h>

#define ROCKETDUINO_VERSION "0.1.1"

#define GPS_BAUD 9600
#define TX_BAUD 38400
#define ACTUAL_TX_BAUD 1200

#define TELEMETRY_FREQUENCY 500
#define STATUS_FREQUENCY 1000

#define ERR_CHECK_FREQUENCY 5000


// The TinyGPS++ object(s)
TinyGPSPlus gps;
TinyGPSCustom VDOP   (gps, "GPGSA", 18); // Vertical dillution Of Presicion

//Time counters
unsigned long lastTelemetryTX = 0UL;
unsigned long lastStatusTX    = 0UL;

//counters for last packet statistics send
unsigned int lastBadSum  = 0;
unsigned int lastGoodSum = 0;
unsigned int lastWithFix = 0;

//
// 
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
// 
//
void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (millis() - lastTelemetryTX > TELEMETRY_FREQUENCY)
    telemetryTX();

  if (millis() - lastStatusTX > STATUS_FREQUENCY)
    statusTX();
}

//
// Sends telemetry information
// 
// (TIME, LAT, LONG, ALT, SOG, COG, Pressure)
//
void telemetryTX() {
  Serial.print(F("T "));
  
  Serial.print(F("Lat="));
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(" Long="));
  Serial.print(gps.location.lng(), 6);

  Serial.print(F(" ALT="));
  Serial.print(gps.altitude.meters());

  Serial.print(F(" KPH="));
  Serial.print(gps.speed.kmph());

  Serial.print(F(" Course="));
  Serial.print(gps.course.deg());

  //Pressure here?
  
  Serial.println();

  lastTelemetryTX = millis();
}


//
// Status update Packet:
// 
// S,21594500,00000,6,165,165,2/1/0
// S = This is a status packet
//   21594500 = GPS time
//            0 = Time validity byte (0-9 = seconds since update.  X=invalid / >10 seconds)
//             0 = Location validity byte
//              0 = Altitude validity byte
//               0 = Speed validity byte
//                0 = Course validity byte
//                  6 = Satellites used for fix
//                    165 = HDOP (Horrizontal dillution of precision)
//                        165 = VDOP (Vertical dillution of precision)
//                            2/1/0 = 2 new packets with valid fix / 1 new packet with no fix / 0 packets with bad checksums
//
void statusTX() {
  //Static Packet 
  Serial.print(F("S,"));

  //Current Time
  Serial.print(gps.time.value());
  Serial.print (",");

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

  //Horizontal dillution of Precision
  Serial.print(",");
  Serial.print(gps.hdop.value());

  //Vertical dillution of Precision
  Serial.print(",");
  Serial.print(VDOP.value());

  //Number of good packets with fixes
  Serial.print(",");
  Serial.print( (gps.sentencesWithFix() - lastWithFix) );

  //Number of good packets with no fix
  Serial.print("/");
  Serial.print( (gps.passedChecksum() - lastGoodSum) - (gps.sentencesWithFix() - lastWithFix) );

  //Number of bad packets
  Serial.print("/");
  Serial.print( gps.failedChecksum() - lastBadSum );

  //Now update the 'lasts'
  lastGoodSum = gps.passedChecksum();
  lastBadSum = gps.failedChecksum();
  lastWithFix = gps.sentencesWithFix();

  Serial.println();
  lastStatusTX = millis();
}
