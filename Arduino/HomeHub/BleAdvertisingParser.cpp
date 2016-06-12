#include "BleAdvertisingParser.h"

//#define PRINT_DEBUG_MESSAGES

BleAdvertisingParser::BleAdvertisingParser() {
  
}


float BleAdvertisingParser::getSensorValue(String str, unsigned int sensorId) {
    uint8_t advData[62] = {0};
    uint8_t advDataLen;
    
    unsigned int offset = 11;
    char tempStr[3];

    tempStr[0] = str[8];
    tempStr[1] = str[9];
    tempStr[2] = 0;
    advDataLen = (int)strtol(tempStr, NULL, 16);

    unsigned int index = offset;
    if (advDataLen > 0) {

        // Convert string to uint8_t array
        unsigned int index = 0;
        for (int i = offset; i < str.length(); i+=2) {
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print(str[i]);
                Serial.print(str[i+1]);
            #endif
            tempStr[0] = str[i];
            tempStr[1] = str[i+1];
            tempStr[2] = 0;
            advData[index++] = (int)strtol(tempStr, NULL, 16);
        }
        #ifdef PRINT_DEBUG_MESSAGES
            Serial.println();
        #endif

        index = 0;
        while (index < advDataLen) {
            uint8_t fieldLength = advData[index];
            uint8_t fieldType   = advData[index+1];
            #ifdef PRINT_DEBUG_MESSAGES
                Serial.print("field length:"); Serial.print(fieldLength); Serial.print(" field type:"); Serial.println(fieldType);
            #endif

            if (fieldType == 0x16) {
                uint8_t k = index + 2;
                uint16_t uuid = (advData[k+1] << 8) | advData[k];
                if ((uuid == 0x180F) && (sensorId == SENSOR_ID_BATTERY_CAPACITY)) {
                    #ifdef PRINT_DEBUG_MESSAGES
                        Serial.print("Battery:"); Serial.print(advData[k+2]); Serial.println("%");
                    #endif
                    return advData[k+2];
                }
            }

            if (fieldType == 0xFF) {
                #ifdef PRINT_DEBUG_MESSAGES
                    Serial.println("Manufacturer data found");
                #endif

                uint8_t k = index + 4;
                unsigned int sId;
                float sVal;
                
                while (k < (index + fieldLength)) {
                    sId = (advData[k++] << 8) | advData[k++];
                    
                    switch (sId) {
                        case SENSOR_ID_TEMPERATURE:
                            sVal = (int16_t)((advData[k++] << 8) | advData[k++]) / 10.0;
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        case SENSOR_ID_HUMIDITY:
                            sVal = (uint16_t)((advData[k++] << 8) | advData[k++]) / 10.0;
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        case SENSOR_ID_AMBIENT_LIGHT:
                            sVal = (uint16_t)((advData[k++] << 8) | advData[k++]);
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        case SENSOR_ID_PIR:
                            sVal = advData[k++];
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        case SENSOR_ID_BAROMETRIC_PRESSURE:
                            sVal = (uint16_t)((advData[k++] << 8) | advData[k++]);
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        case SENSOR_ID_CO2:
                            sVal = (uint16_t)((advData[k++] << 8) | advData[k++]);
                            if (sId == sensorId) {
                                #ifdef PRINT_DEBUG_MESSAGES
                                    Serial.print("Sensor id:"); Serial.println(sId);
                                    Serial.print("Sensor value:"); Serial.println(sVal);
                                #endif
                                return sVal;
                            }
                            break;
                    
                        default:
                            // Sensor Id unknown
                            return 0;
                    }
                }
            }
            index += fieldLength + 1;
        }
    }

    return 0;
}


boolean BleAdvertisingParser::isValidSensorId(unsigned int sensorId) {
//    boolean valid = false;
//
//    switch(sensorId) {
//        case SENSOR_ID_NONE:
//            break;
//        case SENSOR_ID_TEMPERATURE:
//        case SENSOR_ID_HUMIDITY:
//        case SENSOR_ID_AMBIENT_LIGHT:
//        case SENSOR_ID_PIR:
//            valid = true;
//    };

    if (sensorId == SENSOR_ID_NONE) {
        return false;
    } else {
        return true;
    }

//    return valid;
}


