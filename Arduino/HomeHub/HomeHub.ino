/*
  HomeHub
  BLE to Wifi bridge for IoT cloud applications.

  by Thomas Berg
 */

#define DEBUG_MODE
//#define JOBB

#include "WalnutCentral.h"
#include "BleDevice.h"
#include "ThingSpeak.h"

#ifdef ARDUINO_SAMD_FEATHER_M0
    #include <Adafruit_WINC1500.h>
  
    // Define the WINC1500 board connections.
    #define WINC_CS   8     // Chip select (CS) to WINC1500
    #define WINC_IRQ  7     // Interrupt (IRQ) from WINC1500
    #define WINC_RST  4     // Reset (RST) to WINC1500
    #define WINC_EN   2     // Enable (EN) to WINC1500

    // Setup the WINC1500 connection with the pins above and the default hardware SPI.
    Adafruit_WINC1500 WiFi(WINC_CS, WINC_IRQ, WINC_RST);    
    
    // Initialize the Wifi client library
    Adafruit_WINC1500Client wifiClient;
    
    const int BLE_RST_PIN = 7;  // TODO: Check correct pin
#elif defined ARDUINO_SAMD_MKR1000
    #include <WiFi101.h>

    // Initialize the Wifi client library
    WiFiClient wifiClient;
  
    const int BLE_RST_PIN = 7;
#else
    #error "Board not recognized"
#endif

#define NBR_OF_BLE_DEVICES 10

WalnutCentral ble;
BleDevice bleDeviceList[NBR_OF_BLE_DEVICES];

#ifdef JOBB
    char ssid[] = "Sigma-PDA";      //  your network SSID (name)
    char pass[] = "sigma2013!";   // your network password
#else
    char ssid[] = "Simpsons";      //  your network SSID (name)
    char pass[] = "6DskwPNYqtkm4V";   // your network password
#endif

int status = WL_IDLE_STATUS;


// server address:
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(141,101,112,175);  // numeric IP for test page (no DNS)
char server[] = "www.adafruit.com";    // domain name for test page (using DNS)
#define webpage "/testwifi/index.html"  // path to test page

unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds

const unsigned long BLINK_INTERVAL_MS = 1000;
unsigned long lastBlinkTime;
byte blinkLedStatus = 0;

const unsigned long WIFI_CHECK_INTERVAL_MS = 30000;
unsigned long lastWifiCheckTime;


/**
 * Run once initialization
 * 
 */
void setup() 
{
    initGPIO();
    initDebugUart();
    initWifi();
    initThingspeak();
    getBleDevices();
    initBle();
    addBleDevices();
    setBleScanParameters();
    startBleScan();
    lastBlinkTime = millis();
    lastWifiCheckTime = millis();
}


/**
 * Main loop
 */
void loop() 
{
    // Check if there is incoming data from BLE-module
    if (Serial1.available()) {
        handleBleData();
    }

    // TODO: Check if there is incoming data from cloud?

    // Toggle blink-LED every 1 second
    if ((millis() - lastBlinkTime) > BLINK_INTERVAL_MS) {
        blinkLedStatus = !blinkLedStatus;
        digitalWrite(LED_BUILTIN, blinkLedStatus);
        lastBlinkTime = millis();
    }    

    // Check Wifi-status
    if ((millis() - lastWifiCheckTime) > WIFI_CHECK_INTERVAL_MS) {
        printWifiStatus();
        lastWifiCheckTime = millis();
    }    
}


/**
 * Function that initializes the GPIOs
 */
void initGPIO() 
{
    // Initialize reset pin for the BLE-module
    pinMode(BLE_RST_PIN, OUTPUT);
    digitalWrite(BLE_RST_PIN, LOW);
  
    // Initialize the blink-LED pin
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, blinkLedStatus);
}


/**
 * Function that initializes the debug UART aka Serial
 */
void initDebugUart() 
{
    delay(2000);
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
        delay(100);
    }
    Serial.println("Booting up HomeHub ...");
}


void getBleDevices()
{
    bleDeviceList[0] = BleDevice(BD_OP_MODE_ADV,
                                 0);
}


void initBle() 
{
    int err_code;
  
    // Initialize the BLE module
    Serial.println("Initializing BLE-module ...");
    Serial1.begin(115200);
    delay(100);
    err_code = ble.begin(Serial1, BLE_RST_PIN);
    if (err_code != 0) {
        Serial.println("BLE-module initialization FAILED! Halting ...");
        //exit(0);
    }
}


void initWifi()
{
    Serial.println("Initializing Wifi-module ...");

#ifdef WINC_EN
    pinMode(WINC_EN, OUTPUT);
    digitalWrite(WINC_EN, HIGH);
#endif
    
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
        // don't continue:
        exit(0);
    }

    // attempt to connect to Wifi network:
    while ( status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 1 second for connection:
        delay(1000);
        Serial.print('.');
    }
    
    // you're connected now, so print out the status:
    printWifiStatus();
}


void initThingspeak()
{
    ThingSpeak.begin(wifiClient,"api.thingspeak.com", 80);
}


/**
 * Function that adds the BLE devices to scan for
 * 
 */
void addBleDevices()
{
    int err_code;
  
    delay(100);
    Serial.println("Adding device to scanlist ...");
    err_code = ble.addDevice();
    if (err_code != 0) {
        Serial.println("Adding BLE device FAILED!");
    }   
}


/**
 * Function that sets BLE scanning parameters
 */
void setBleScanParameters()
{
    int err_code;
  
    delay(100);
    Serial.println("Setting BLE scan parameters...");
    err_code = ble.setScanParameters();
    if (err_code != 0) {
        Serial.println("Setting BLE scan parameters FAILED!");
    }    
}


/**
 * Function that starts scanning for BLE devices
 */
void startBleScan()
{
    int err_code;
  
    delay(100);
    Serial.println("Starting BLE scan ...");
    err_code = ble.startScan();
    if (err_code != 0) {
        Serial.println("Starting BLE scan FAILED!");
    }    
}


/**
 * Function that handles incoming sensor data from the BLE-module
 * and passes it on to the cloud.
 */
void handleBleData() 
{
    String rxStr;
    
    Serial1.setTimeout(1000);
    rxStr = Serial1.readStringUntil('\n');
    if (rxStr.length() == 0) {
        // Timeout occured, ignore received data
    } else {
        Serial.println(rxStr);
        // TODO: Send data or add to send buffer
    }
}


// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  wifiClient.stop();

  // if there's a successful connection:
  if (wifiClient.connect(server, 80)) {
    Serial.println("connecting...");
    // Make a HTTP request:
    wifiClient.print("GET ");
    wifiClient.print(webpage);
    wifiClient.println(" HTTP/1.1");
    wifiClient.print("Host: "); wifiClient.println(server);
    wifiClient.println("Connection: close");
    wifiClient.println();

    // note the time that the connection was made:
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}


void printWifiStatus() {
    Serial.print("Wifi status: ");
    switch (WiFi.status()) {
        case WL_CONNECTED:
            Serial.println("WL_CONNECTED");
            break;
            
        case WL_NO_SHIELD:
            Serial.println("WL_NO_SHIELD");
            break;
            
        case WL_IDLE_STATUS:
            Serial.println("WL_IDLE_STATUS");
            break;
            
        case WL_NO_SSID_AVAIL:
            Serial.println("WL_NO_SSID_AVAIL");
            break;
            
        case WL_SCAN_COMPLETED:
            Serial.println("WL_SCAN_COMPLETED");
            break;
            
        case WL_CONNECT_FAILED:
            Serial.println("WL_CONNECT_FAILED");
            break;
            
        case WL_CONNECTION_LOST:
            Serial.println("WL_CONNECTION_LOST");
            break;
            
        case WL_DISCONNECTED:
            Serial.println("WL_DISCONNECTED");
            break;
    }

    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    Serial.println();
}

