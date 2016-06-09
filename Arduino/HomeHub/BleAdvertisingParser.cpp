#include "BleAdvertisingParser.h"

#define ENABLE_DEBUG_PRINT

BleAdvertisingParser::BleAdvertisingParser() {
  
}


float BleAdvertisingParser::getSensorValue(String str, unsigned int sensorId) {
    unsigned int nbrOfBytes = 15;   // TODO: Read from str
    unsigned int offset = 7;

    if (nbrOfBytes > 0) {
        unsigned short a[62] = {0};
        unsigned int index = 0;
        char temp[3];
        for (int i = offset; i < (2*nbrOfBytes+offset); i+=2) {
//            Serial.print(str[i]);
//            Serial.print(str[i+1]);
            temp[0] = str[i];
            temp[1] = str[i+1];
            temp[2] = 0;
            a[index++] = (int)strtol(temp, NULL, 16);
        }
//        Serial.println();

        index = 0;
        unsigned int sId;
        unsigned int sVal;
        while (index < nbrOfBytes) {
            sId = (a[index++] << 8) | a[index++];
            switch (sId) {
                case SENSOR_ID_TEMPERATURE:
                    sVal = (a[index++] << 8) | a[index++];
                    if (sId == sensorId) {
                        return sVal / 10.0;
                    }
                    break;
                    
                case SENSOR_ID_HUMIDITY:
                    sVal = (a[index++] << 8) | a[index++];
                    if (sId == sensorId) {
                        return sVal / 10.0;
                    }
                    break;
                    
                case SENSOR_ID_AMBIENT_LIGHT:
                    sVal = (a[index++] << 8) | a[index++];
                    if (sId == sensorId) {
                        return sVal;
                    }
                    break;
                    
                case SENSOR_ID_PIR:
                    sVal = a[index++];
                    if (sId == sensorId) {
                        return sVal;
                    }
                    break;
                    
            }
        }
        
    }
}


void BleAdvertisingParser::convert() {
    
}

