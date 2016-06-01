#include <WiFi101.h>
#include <WiFiClient.h>
#include <WiFiMDNSResponder.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>

#include "WalnutCentral.h"

WalnutCentral ble;
const int BLE_RST_PIN = 7;
int i = 0;


void setup() {
  int err_code;

  // Initialize reset pin for the BLE-module
  pinMode(BLE_RST_PIN, OUTPUT);
  digitalWrite(BLE_RST_PIN, LOW);
  
  delay(2000);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
    delay(100);
  }
  Serial.println("Booting up HomeHubNew ...");

  // initialize digital pin 13 as an output.
  pinMode(PIN_LED, OUTPUT);

  // Initialize the BLE module
  Serial.println("Initializing BLE-module ...");
  Serial1.begin(115200);
  Serial1.begin(921600);
  delay(100);
  err_code = ble.begin(Serial1, BLE_RST_PIN);
  if (err_code != 0) {
    Serial.println("Initializing BLE-module FAILED! Halting ...");
    exit(0);
  }

  delay(10);
  Serial.println("Stopping BLE scan ...");
  err_code = ble.stopScan();
  if (err_code != 0) {
    Serial.println("Stopping BLE scan FAILED!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(i++);
  digitalWrite(PIN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  ble.startScan();
  digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  ble.stopScan();
}

