
#ifndef BleDevice_h
#define BleDevice_h

#include "Arduino.h"

typedef enum {
    BD_OP_MODE_NONE,
    BD_OP_MODE_ADV,
    BD_OP_MODE_ESS,
    BD_OP_MODE_AIO,
} ble_device_op_mode_t;


class BleDevice {
  
  public:
    BleDevice();
    BleDevice(ble_device_op_mode_t op_mode,
              unsigned long thingSpeakChannelNumber);
              
//    int begin(Stream &serial, const int rstPin);
//    int startScan();
//    int stopScan();
//    int setScanParameters();
//    int addDevice();

  private:
    ble_device_op_mode_t _op_mode;

    unsigned long _thingSpeakChannelNumber;
      
};

#endif
