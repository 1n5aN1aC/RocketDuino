#ifndef HEADER_COMMUNICATIONMANAGER
  #define HEADER_COMMUNICATIONMANAGER

  #include "DataManager.h"
  
  class CommunicationManager {
    private:
      unsigned long lastTelemetryTX;
      unsigned long lastStatusTX;
      unsigned long lastRX;
      DataManager data_manager;
      
    public:
      CommunicationManager(DataManager data_manager);

      void setup_connections();

      void transmit_telemetry();
      void transmit_status();
      
      void process_gps_serial();
      void process_gc_serial();

      void update_leds();
  };
   
#endif
