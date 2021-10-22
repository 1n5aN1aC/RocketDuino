//
// File:    main.ino
// Author:  1n5aN1aC (Joshua Villwock)
// Purpose: Handles primary logic for the RocketDuino
//
#include "Arduino.h"

//My Classes
#include "DataManager.h"
#include "CommunicationManager.h"

#define ROCKETDUINO_VERSION "0.3.2"

//Manager classes
DataManager          data_manager;
CommunicationManager communication_manager(data_manager);

//
// Run Initial setup Tasks
//
// 1. Initialize serial buffers
// 2. Send Version information
// 3. Setup barometer
//
void setup()
{
  communication_manager.setup_connections();
  
  Serial.println();
  Serial.print(F("RocketDuino v."));
  Serial.print(ROCKETDUINO_VERSION);
  Serial.print(F(" & TinyGPS++ v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  data_manager.setup_baro();
}

//
// Main Loop Tasks
//
// 1. Recieve & Process GPS data
// 2. Recieve & Process Ground Control Packets
// 3. Process data & calculate averages (& Update Modes)
// 4. Send Telemetry if needed
// 5. Send Status information if needed.
// 6. Update LEDs
//
void loop()
{
  // 1. Recieve & Process GPS data
  // 2. Recieve & Process Ground Control Packets
  communication_manager.process_gps_serial();
  communication_manager.process_gc_serial();

  // 3. Process data & calculate averages (& Update Modes)
  data_manager.update_battery();
  data_manager.update_barometer();
  data_manager.update_gps_altitude();
  data_manager.calculate_location();

  // 4. Send Telemetry if needed
  // 5. Send Status information if needed.
  communication_manager.transmit_telemetry();
  communication_manager.transmit_status();

  // 6. Update LEDs
  communication_manager.update_leds();
}
