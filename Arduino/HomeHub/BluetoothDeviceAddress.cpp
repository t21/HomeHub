#include "BluetoothDeviceAddress.h"

#define ENABLE_DEBUG_PRINT

BluetoothDeviceAddress::BluetoothDeviceAddress() {

}

BluetoothDeviceAddress::BluetoothDeviceAddress(uint8_t ad5, uint8_t ad4, uint8_t ad3, uint8_t ad2, uint8_t ad1, uint8_t ad0) {
    _address[0] = ad0;  
    _address[1] = ad1;  
    _address[2] = ad2;  
    _address[3] = ad3;  
    _address[4] = ad4;  
    _address[5] = ad5;  
}


String BluetoothDeviceAddress::toString() {
    char str[18] = {0};
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", _address[5], _address[4], _address[3], _address[2], _address[1], _address[0]);
    return str;
}


