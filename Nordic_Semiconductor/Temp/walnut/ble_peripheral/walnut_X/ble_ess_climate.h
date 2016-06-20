/* This file was generated by plugin 'Nordic Semiconductor nRF5x v.1.2.2' (BDS version 1.0.2095.0) */

#ifndef BLE_ENVIRONMENTAL_SENSING_SERVICE_H__
#define BLE_ENVIRONMENTAL_SENSING_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_bds.h"



/**@brief ENVIRONMENTAL SENSING SERVICE event type. */
typedef enum
{ 
    BLE_ENVIRONMENTAL_SENSING_SERVICE_TEMPERATURE_EVT_NOTIFICATION_ENABLED,  /**< Temperature value notification enabled event. */
    BLE_ENVIRONMENTAL_SENSING_SERVICE_TEMPERATURE_EVT_NOTIFICATION_DISABLED, /**< Temperature value notification disabled event. */
    BLE_ENVIRONMENTAL_SENSING_SERVICE_TEMPERATURE_EVT_CCCD_WRITE, /**< Temperature CCCD write event. */
} ble_environmental_sensing_service_evt_type_t;

// Forward declaration of the ble_environmental_sensing_service_t type.
typedef struct ble_environmental_sensing_service_s ble_environmental_sensing_service_t;








/**@brief Temperature structure. */
typedef struct
{
    int16_t temperature;
} ble_environmental_sensing_service_temperature_t;

/**@brief ENVIRONMENTAL SENSING SERVICE Service event. */
typedef struct
{
    ble_environmental_sensing_service_evt_type_t evt_type;    /**< Type of event. */
    union {
        uint16_t cccd_value; /**< Holds decoded data in Notify and Indicate event handler. */
    } params;
} ble_environmental_sensing_service_evt_t;

/**@brief ENVIRONMENTAL SENSING SERVICE Service event handler type. */
typedef void (*ble_environmental_sensing_service_evt_handler_t) (ble_environmental_sensing_service_t * p_environmental_sensing_service, ble_environmental_sensing_service_evt_t * p_evt);

/**@brief ENVIRONMENTAL SENSING SERVICE Service init structure. This contains all options and data needed for initialization of the service */
typedef struct
{
    ble_environmental_sensing_service_evt_handler_t     evt_handler; /**< Event handler to be called for handling events in the ENVIRONMENTAL SENSING SERVICE Service. */
    bool is_temperature_notify_supported;    /**< TRUE if notification of Temperature is supported. */
    ble_environmental_sensing_service_temperature_t ble_environmental_sensing_service_temperature_initial_value; /**< If not NULL, initial value of the Temperature characteristic. */ 
} ble_environmental_sensing_service_init_t;

/**@brief ENVIRONMENTAL SENSING SERVICE Service structure. This contains various status information for the service.*/
struct ble_environmental_sensing_service_s
{
    ble_environmental_sensing_service_evt_handler_t evt_handler; /**< Event handler to be called for handling events in the ENVIRONMENTAL SENSING SERVICE Service. */
    uint16_t service_handle; /**< Handle of ENVIRONMENTAL SENSING SERVICE Service (as provided by the BLE stack). */
    bool is_temperature_notify_supported;    /**< TRUE if notification of Temperature is supported. */
    ble_gatts_char_handles_t temperature_handles; /**< Handles related to the Temperature characteristic. */
    uint16_t conn_handle; /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};

/**@brief Function for initializing the ENVIRONMENTAL SENSING SERVICE.
 *
 * @param[out]  p_environmental_sensing_service       ENVIRONMENTAL SENSING SERVICE Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_environmental_sensing_service_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_environmental_sensing_service_init(ble_environmental_sensing_service_t * p_environmental_sensing_service, const ble_environmental_sensing_service_init_t * p_environmental_sensing_service_init);

/**@brief Function for handling the Application's BLE Stack events.*/
void ble_environmental_sensing_service_on_ble_evt(ble_environmental_sensing_service_t * p_environmental_sensing_service, ble_evt_t * p_ble_evt);

/**@brief Function for setting the Temperature.
 *
 * @details Sets a new value of the Temperature characteristic. The new value will be sent
 *          to the client the next time the client reads the Temperature characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_environmental_sensing_service                 ENVIRONMENTAL SENSING SERVICE Service structure.
 * @param[in]   p_temperature  New Temperature.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_environmental_sensing_service_temperature_set(ble_environmental_sensing_service_t * p_environmental_sensing_service, ble_environmental_sensing_service_temperature_t * p_temperature);

/**@brief Function for sending the Temperature.
 *
 * @details The application calls this function after having performed a temperature.
 *          The temperature data is encoded and sent to the client.
 *          This function is only generated if the characteristic's Notify or Indicate property is not 'Excluded'.
 *
 * @param[in]   p_environmental_sensing_service                    ENVIRONMENTAL SENSING SERVICE Service structure.
 * @param[in]   p_temperature               New temperature.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_environmental_sensing_service_temperature_send(ble_environmental_sensing_service_t * p_environmental_sensing_service, ble_environmental_sensing_service_temperature_t * p_temperature);

#endif //_BLE_ENVIRONMENTAL_SENSING_SERVICE_H__
