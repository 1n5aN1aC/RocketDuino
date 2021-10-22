#ifndef HEADER_DATAMANAGER
  #define HEADER_DATAMANAGER

  #include <TinyGPS++.h>
  #include "runningAverage.h"

  class DataManager {
    private:
      unsigned long lastBatteryUpdate;
      unsigned long lastBarometerUpdate;
      unsigned long lastGpsHeightUpdate;
      unsigned long lastLocationCalculation;

      float last_altitude_gps;
      float last_altitude_baro;

    public:
      DataManager();

      TinyGPSPlus gps_object;    //The TinyGPS++ object

      int   get_average_battery();
      float get_average_altitude();
      float get_average_speed();
      float get_average_acceleration();
      float get_average_temperature();
      
      void setup_baro();
      void update_battery();
      void update_barometer();
      void update_gps_altitude();
      void calculate_location();
  };
   
#endif
