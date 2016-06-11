
#ifndef WalnutCentral_h
#define WalnutCentral_h

#include "Arduino.h"
#include "BluetoothDeviceAddress.h"

class WalnutCentral {
  
  public:
    WalnutCentral();
    int begin(Stream &serial, const int rstPin);
    int startScan();
    int stopScan();
    int setScanParameters();
    int addDevice(BluetoothDeviceAddress address);

  private:
    Stream* _serial;
    int _rstPin;
    int sendString(char *tx, int tx_len);
    int receiveString(char *rx, int *rx_len);
  
};

#endif
