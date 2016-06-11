#include "BleDevice.h"

#define PRINT_DEBUG_MESSAGES

BleDevice::BleDevice() {
  
}

BleDevice::BleDevice(BluetoothDeviceAddress bluetoothDeviceAddress,
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
                    ) 
{
    _bluetoothDeviceAddress = bluetoothDeviceAddress;
    _channelNumber = channelNumber;  
    _writeAPIKey = writeAPIKey;
    _sensorIdField1 = sensorIdField1;
    _sensorIdField2 = sensorIdField2;
    _sensorIdField3 = sensorIdField3;
    _sensorIdField4 = sensorIdField4;
    _sensorIdField5 = sensorIdField5;
    _sensorIdField6 = sensorIdField6;
    _sensorIdField7 = sensorIdField7;
    _sensorIdField8 = sensorIdField8;
}


unsigned long BleDevice::getThingSpeakChannelNumber() {
    return _channelNumber;
}

String BleDevice::getThingSpeakWriteAPIKey() {
    return _writeAPIKey;
}

unsigned int BleDevice::getSensorIdField1() {
    return _sensorIdField1;
}

unsigned int BleDevice::getSensorIdField2() {
    return _sensorIdField2;
}

unsigned int BleDevice::getSensorIdField3() {
    return _sensorIdField3;
}

unsigned int BleDevice::getSensorIdField4() {
    return _sensorIdField4;
}

unsigned int BleDevice::getSensorIdField5() {
    return _sensorIdField5;
}

unsigned int BleDevice::getSensorIdField6() {
    return _sensorIdField6;
}

unsigned int BleDevice::getSensorIdField7() {
    return _sensorIdField7;
}

unsigned int BleDevice::getSensorIdField8() {
    return _sensorIdField8;
}


BluetoothDeviceAddress BleDevice::getBluetoothDeviceAddress()
{
    return _bluetoothDeviceAddress;
}

