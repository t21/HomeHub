
#ifndef BluetoothDeviceAddress_h
#define BluetoothDeviceAddress_h

#include "Arduino.h"


class BluetoothDeviceAddress {
  
  public:
    BluetoothDeviceAddress();
    BluetoothDeviceAddress(uint8_t ad5, uint8_t ad4, uint8_t ad3, uint8_t ad2, uint8_t ad1, uint8_t ad0);

    String toString();


  private:
    uint8_t _address[6];
      
};

#endif
