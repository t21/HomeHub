#include "WalnutCentral.h"

#define PRINT_DEBUG_MESSAGES

#define MAX_BUF_SIZE 100
#define UART_TIMEOUT 2000

WalnutCentral::WalnutCentral() {
    // Empty constructor  
}


int WalnutCentral::begin(Stream &serial, const int rstPin) {
    _serial = &serial;
    _rstPin = rstPin;

    // Release Reset
    digitalWrite(rstPin, 0);
    delay(100);
    digitalWrite(rstPin, 1);
    // Wait for boot finished
    delay(1000);
    // TODO: Implement command or character to be received when booting up

    // Send ATZ
    char tx[MAX_BUF_SIZE];
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  
    strcpy(tx, "ATZ");
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }
  
    // Check revision
    // TODO: Implement command on Walnut
  
}


int WalnutCentral::startScan() {
    char tx[MAX_BUF_SIZE];
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  
    strcpy(tx, "AT+SCANSTART");
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }

    if (strcmp(rx, "OK") == 0) {
        return 0;
    } else {
        return 1;
    }
}


int WalnutCentral::stopScan() {
    char tx[MAX_BUF_SIZE];
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  
    strcpy(tx, "AT+SCANSTOP");
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }

    if (strcmp(rx, "OK") == 0) {
        return 0;
    } else {
        return 1;
    }
}


int WalnutCentral::setScanParameters() {
    char tx[MAX_BUF_SIZE];
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  
    strcpy(tx, "AT+SCANSETP=1,0,1760,1680,0");
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }

    if (strcmp(rx, "OK") == 0) {
        return 0;
    } else {
        return 1;
    }
}


int WalnutCentral::addDevice(BluetoothDeviceAddress address) {
    char tx[MAX_BUF_SIZE];
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  

    String tempStr = "AT+DEVADD=" + address.toString();
    tempStr.toCharArray(tx, MAX_BUF_SIZE);
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.println(tx);
    #endif
    
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }

    if (strcmp(rx, "OK") == 0) {
        return 0;
    } else {
        return 1;
    }
}


int WalnutCentral::getAdvertisingMessage(uint8_t deviceIndex) {
    char tx[MAX_BUF_SIZE] = {0};
    char rx[MAX_BUF_SIZE];
    int rx_len = 0;
  
    sprintf(tx, "AT+ADV=%02d", deviceIndex);
    
    int err_code = sendString(tx, strlen(tx));
    if (err_code != 0) {
        return err_code;
    }
  
    err_code = receiveString(rx, &rx_len);
    if (err_code != 0) {
        return err_code;
    }

    if (strcmp(rx, "OK") == 0) {
        return 0;
    } else {
        return 1;
    }
}


int WalnutCentral::sendString(char *tx, int tx_len) 
{
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.println(tx);
    #endif
    
    int nbr_of_bytes_sent = 0;

    for (int i = 0; i < tx_len; i++) {
        #ifdef USE_BLE_HW_HANDSHAKE
            while (!digitalRead(UART_CTS_PIN) {
                delayMicroseconds(1);
            }
        #endif
        _serial->print(tx[i]);
        nbr_of_bytes_sent++;
        #ifndef USE_BLE_HW_HANDSHAKE
            delayMicroseconds(50);      // 45us failed @ 460kbit/s, 46us passes @ 460kbit/s
        #endif
    }
    _serial->print('\r');
    nbr_of_bytes_sent++;
    delayMicroseconds(50);
    _serial->print('\n');
    nbr_of_bytes_sent++;
  
    if (nbr_of_bytes_sent == (tx_len + 2)) {
        return 0;  
    } else {
        return 1;
    }
}


int WalnutCentral::receiveString(char *rx, int *rx_len) 
{
    memset(rx, 0, MAX_BUF_SIZE);

    _serial->setTimeout(1000);
    *rx_len = _serial->readBytesUntil('\n', rx, MAX_BUF_SIZE);
    if (*rx_len > 1) {
        rx[*rx_len-1] = 0;
        *rx_len -= 1;
    }

    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print("receiveString: -");
        Serial.print(rx);
        Serial.print("- nbrOfBytes: ");
        Serial.println(*rx_len);
    #endif
  
    if (*rx_len == 0) {
        return 1; // Timeout
    }

    return 0;
}

