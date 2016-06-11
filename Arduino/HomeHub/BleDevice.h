
#ifndef BleDevice_h
#define BleDevice_h

#include "Arduino.h"
#include "BluetoothDeviceAddress.h"

typedef enum {
    BD_OP_MODE_NONE,
    BD_OP_MODE_ADV,
    BD_OP_MODE_ESS,
    BD_OP_MODE_AIO,
} ble_device_op_mode_t;


class BleDevice {
  
  public:
    BleDevice();
    BleDevice(BluetoothDeviceAddress bleAddress,
              ble_device_op_mode_t op_mode,
              unsigned long channelNumber,
              String writeAPIKey,
              unsigned int sensorIdField1,
              unsigned int sensorIdField2,
              unsigned int sensorIdField3,
              unsigned int sensorIdField4,
              unsigned int sensorIdField5,
              unsigned int sensorIdField6,
              unsigned int sensorIdField7,
              unsigned int sensorIdField8
              );

    unsigned long getThingSpeakChannelNumber();
    String getThingSpeakWriteAPIKey();
    unsigned int getSensorIdField1();
    unsigned int getSensorIdField2();
    unsigned int getSensorIdField3();
    unsigned int getSensorIdField4();
    unsigned int getSensorIdField5();
    unsigned int getSensorIdField6();
    unsigned int getSensorIdField7();
    unsigned int getSensorIdField8();
    BluetoothDeviceAddress getBluetoothDeviceAddress();

  private:
    BluetoothDeviceAddress _bluetoothDeviceAddress;
    ble_device_op_mode_t _op_mode;
    unsigned long _channelNumber;
    String _writeAPIKey;
    unsigned int _sensorIdField1;
    unsigned int _sensorIdField2;
    unsigned int _sensorIdField3;
    unsigned int _sensorIdField4;
    unsigned int _sensorIdField5;
    unsigned int _sensorIdField6;
    unsigned int _sensorIdField7;
    unsigned int _sensorIdField8;
      
};

#endif
