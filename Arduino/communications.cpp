//
// File:    communications.cpp
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles all communication between the RocketDuino and the ground station.
//


//
// Telemetry (Location) Update Packet
//
// T,44.982719,-123.337142,98.5,1.17,___
// T = this is a telemetry packet
//   44.982719 = Lattitude
//             -123.337142 = Longitude
//                         98.5 = Elevation in Meters
//                              1.17 = Speed in Kph
//
void telemetryTX() {
  Serial.print("T,");

  Serial.print(gps.location.lat(), 6);
  Serial.print(",");
  Serial.print(gps.location.lng(), 6);

  Serial.print(",");
  Serial.print(gps.altitude.meters());
  //maybe use avgAltitde?

  Serial.print(",");
  Serial.print(gps.speed.kmph());
  //maybe use avgSpeed?

  //Finally, update last Packet variable
  Serial.println();
  lastTelemetryTX = millis();
}


//
// Status (Integrity) update Packet:
//
// S,21594500,000000,6,165,532,2/1/0
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
//                         532 = battery input voltage.  In centivolts (5.32v) (After diode, before everything else)
//                             2/1/0 = 2 new (GPS) packets with valid fix / 1 new packet with no fix / 0 packets with bad checksums
//
void statusTX() {
  //Static Packet
  Serial.print("S,");

  //Current Time
  Serial.print(gps.time.value());
  Serial.print (",");

  //Uplink Status
  unsigned long timeSince = millis() - lastRX;
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
  Serial.print(gps.satellites.value());

  //Horizontal Dillution of Precision
  Serial.print(",");
  Serial.print(gps.hdop.value());

  //Battery Volatage
  Serial.print(",");
  Serial.print(avgBattery);

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
// A = arm the flight controller for takeoff
//
void process_command(byte ch) {
  if (ch == 'P') {
    lastRX = millis();
  }
  if (ch == 'A' && currMode < 0) {
    currMode = 0;
  }
  else {
    //something went wrong?  Bad packet.
  }
}
