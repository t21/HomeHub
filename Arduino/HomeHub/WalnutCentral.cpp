#include "WalnutCentral.h"

#define ENABLE_DEBUG_PRINT

#define MAX_BUF_SIZE 50
#define UART_TIMEOUT 2000

WalnutCentral::WalnutCentral() {
  
}


int WalnutCentral::begin(Stream &serial, const int rstPin) {
  _serial = &serial;
  _rstPin = rstPin;

  // Release Reset
  // TODO
  digitalWrite(rstPin, 0);
  delay(100);
  digitalWrite(rstPin, 1);
  // Wait for boot finished
  delay(1000);

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
  //Serial.println(rx);

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
  //Serial.println(rx);

  if (strcmp(rx, "OK") == 0) {
    return 0;
  } else {
    return 1;
  }
}


int WalnutCentral::addDevice() {
  char tx[MAX_BUF_SIZE];
  char rx[MAX_BUF_SIZE];
  int rx_len = 0;
  
  strcpy(tx, "AT+DEVADD=D5:A9:E3:C1:1B:A4");
  
  int err_code = sendString(tx, strlen(tx));
  if (err_code != 0) {
    return err_code;
  }
  
  err_code = receiveString(rx, &rx_len);
  if (err_code != 0) {
    return err_code;
  }
  //Serial.println(rx);

  if (strcmp(rx, "OK") == 0) {
    return 0;
  } else {
    return 1;
  }
}


int WalnutCentral::sendString(char *tx, int tx_len) 
{
  Serial.println(tx);
  int nbr_of_bytes_sent = _serial->println(tx);
  
  if (nbr_of_bytes_sent == (tx_len + 2)) {
    return 0;  
  } else {
    return 1;
  }
}


int WalnutCentral::receiveString(char *rx, int *rx_len) 
{
  memset(rx, 0, MAX_BUF_SIZE);

  _serial->setTimeout(2000);
  *rx_len = _serial->readBytesUntil('\n', rx, MAX_BUF_SIZE);
//  *rx_len = _serial->readBytes(rx, MAX_BUF_SIZE);
  if (*rx_len > 1) {
    rx[*rx_len-1] = 0;
    //rx[*rx_len-2] = 0;
    *rx_len -= 1;
  }

  Serial.print("receiveString: .");
  Serial.print(rx);
  Serial.print(". nbrOfBytes: ");
  Serial.println(*rx_len);
  
  if (*rx_len == 0) {
    return 1; // Timeout
  }

  return 0;
}

