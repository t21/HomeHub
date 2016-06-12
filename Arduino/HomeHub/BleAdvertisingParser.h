
#ifndef BleAdvertisingParser_h
#define BleAdvertisingParser_h

#include "Arduino.h"

typedef enum {
    SENSOR_ID_NONE                  = 0,
    SENSOR_ID_TEMPERATURE           = 1,
    SENSOR_ID_HUMIDITY              = 2,
    SENSOR_ID_AMBIENT_LIGHT         = 3,
    SENSOR_ID_PIR                   = 8,
    SENSOR_ID_BAROMETRIC_PRESSURE   = 9,
    SENSOR_ID_BATTERY_CAPACITY      = 10,
    SENSOR_ID_CO2                   = 12,
} sensor_id_t;


class BleAdvertisingParser {
  
  public:
    BleAdvertisingParser();
    static float getSensorValue(String str, unsigned int sensorId);
    static boolean isValidSensorId(unsigned int sensorId);

  private:
    void convert();
      
};

#endif
