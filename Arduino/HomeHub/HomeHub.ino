/*
  HomeHub
  BLE to Wifi bridge for IoT cloud applications.

  by Thomas Berg
 */

#include "WalnutCentral.h"
#include "BleDevice.h"
#include "BleAdvertisingParser.h"
#include "ThingSpeak.h"

//#define PRINT_DEBUG_MESSAGES
//#define JOBB

#if defined(ARDUINO_SAMD_FEATHER_M0)

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
    
#elif defined(ARDUINO_SAMD_MKR1000)

    #include <WiFi101.h>

    // Initialize the Wifi client library
    WiFiClient wifiClient;
  
    const int BLE_RST_PIN = 7;
    
#else
    #error "Unsupported board?!?"
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


unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds

const unsigned long BLINK_INTERVAL_MS = 1000;
unsigned long lastBlinkTime;
byte blinkLedStatus = 0;

const unsigned long WIFI_CHECK_INTERVAL_MS = 30000;
unsigned long lastWifiCheckTime;

const unsigned long THINGSPEAK_TEST_INTERVAL_MS = 60000;
unsigned long lastThingSpeakTestTime;
int thingSpeakTestCounter = 0;

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
    setupBleDevices();
    initBle();
    addBleDevices();
    setBleScanParameters();
    startBleScan();
    lastBlinkTime = millis();
    lastWifiCheckTime = millis();
    lastThingSpeakTestTime = millis();
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
        printWifiConnectionStatus();
        lastWifiCheckTime = millis();
    }    

    // Testing ThingSpeak API
//    if ((millis() - lastThingSpeakTestTime) > THINGSPEAK_TEST_INTERVAL_MS) {
//        //printWifiStatus();
//        ThingSpeak.setField(1, thingSpeakTestCounter);
//        Serial.print("ThingSpeak channel number: ");
//        Serial.println(bleDeviceList[0].getThingSpeakChannelNumber());
//        Serial.print("ThingSpeak write API key: ");
//        Serial.println(bleDeviceList[0].getThingSpeakWriteAPIKey());
//        String t = bleDeviceList[0].getThingSpeakWriteAPIKey();
//        char t2[50];
//        t.toCharArray(t2, 50);
//        // TODO: Add writeFields function with String
//        //ThingSpeak.writeFields(bleDeviceList[0].getThingSpeakChannelNumber(), bleDeviceList[0].getThingSpeakWriteAPIKey());
//        ThingSpeak.writeFields(bleDeviceList[0].getThingSpeakChannelNumber(), t2);
//        lastThingSpeakTestTime = millis();
//        thingSpeakTestCounter++;
//    }    
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
    Serial.println("*** Booting up HomeHub ... ***");
}


/**
 * Function that initializes the Wifi
 */
void initWifi()
{
    Serial.println("Initializing Wifi-module ...");

    #ifdef WINC_EN
        pinMode(WINC_EN, OUTPUT);
        digitalWrite(WINC_EN, HIGH);
    #endif
    
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present, halting execution");
        // don't continue:
        exit(0);
    }

    // attempt to connect to Wifi network:
    WiFi.begin(ssid, pass);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while ( WiFi.status() != WL_CONNECTED) {
        // wait for connection:
        delay(500);
        Serial.print('.');
    }
    //Serial.println();
//    // attempt to connect to Wifi network:
//    while ( status != WL_CONNECTED) {
//        Serial.print("Attempting to connect to SSID: ");
//        Serial.println(ssid);
//        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
//        status = WiFi.begin(ssid, pass);
//
//        // wait 1 second for connection:
//        delay(1000);
//        Serial.print('.');
//    }
//    Serial.println();
    
    // you're connected now, so print out the status:
    printWifiStatus();
}


void initThingspeak()
{
    ThingSpeak.begin(wifiClient,"api.thingspeak.com", 80);
}


void setupBleDevices()
{
    bleDeviceList[0] = BleDevice(BD_OP_MODE_ADV,            // BLE operating mode
                                 123470,                    // ThingSpeak channel number
                                 "0TLG1W8504G1IODA",        // ThingSpeak write API Key
                                 SENSOR_ID_TEMPERATURE, SENSOR_ID_HUMIDITY, SENSOR_ID_AMBIENT_LIGHT, SENSOR_ID_PIR, 0, 0, 0, 0     // SensorId <-> ThingSpeak Field # mapping
                                 );
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
        Serial.println("BLE-module initialization FAILED! Halting execution ...");
        //exit(0);
    }
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
    
    Serial1.setTimeout(2000);
    rxStr = Serial1.readStringUntil('\n');
    rxStr.trim();
    if (rxStr.length() == 0) {
        // Timeout occured, ignore received data
    } else {
//        #ifdef PRINT_DEBUG_MESSAGES
            Serial.println(rxStr);
//        #endif
        int device_index = ((rxStr[5] - '0') * 10) + (rxStr[6] - '0');
        #ifdef PRINT_DEBUG_MESSAGES
            Serial.print("Device index:"); Serial.println(device_index);
        #endif
        if (device_index >= NBR_OF_BLE_DEVICES) {
            Serial.println("Wrong device index found ...");
            return; 
        }

        boolean sensorValuesUpdated = false;
        
        unsigned int sensorId = bleDeviceList[device_index].getSensorIdField1();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(1, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField2();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(2, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField3();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(3, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField4();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(4, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField5();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(5, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField6();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(6, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField7();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(7, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }
        
        sensorId = bleDeviceList[device_index].getSensorIdField8();
        if (BleAdvertisingParser::isValidSensorId(sensorId)) {
            float sensorValue = BleAdvertisingParser::getSensorValue(rxStr, sensorId);
            ThingSpeak.setField(8, sensorValue);
            sensorValuesUpdated = true;
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("SensorId:"); Serial.println(sensorId);
                Serial.print("SensorValue:"); Serial.println(sensorValue);
            #endif
        }

        if (sensorValuesUpdated) {
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.println("Updating");
            #endif
            String t = bleDeviceList[0].getThingSpeakWriteAPIKey();
            char t2[50];
            t.toCharArray(t2, 50);
            ThingSpeak.writeFields(bleDeviceList[0].getThingSpeakChannelNumber(), t2);
            lastThingSpeakTestTime = millis();
        }
    }
}


void printWifiStatus() {
    printWifiConnectionStatus();

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


void printWifiConnectionStatus()
{
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
}

