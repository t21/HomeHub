/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/** 
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
//#include "boards.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "device_manager.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "app_util.h"
#include "app_timer.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "app_uart.h"
#include "nrf_drv_spis.h"
#include "ble_radio_notification.h"
#include "nrf_delay.h"

#define CENTRAL_LINK_COUNT         1                                  /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT      0                                  /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define STRING_BUFFER_LEN          50
#define BOND_DELETE_ALL_BUTTON_ID  0                                  /**< Button used for deleting all bonded centrals during startup. */

#define APP_TIMER_PRESCALER        0                                  /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE    2                                  /**< Size of timer operation queues. */

#define APPL_LOG                   NRF_LOG_PRINTF                     /**< Logger macro that will be used in this file to do logging over UART or RTT based on nrf_log configuration. */
#define APPL_LOG_DEBUG             NRF_LOG_PRINTF_DEBUG               /**< Debug logger macro that will be used in this file to do logging of debug information over UART or RTT based on nrf_log configuration. This will only work if DEBUG is defined*/

#define SEC_PARAM_BOND             1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM             1                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC             0                                  /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS         0                                  /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB              0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                 /**< Maximum encryption key size. */

#define DEFAULT_SCAN_INTERVAL      0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define DEFAULT_SCAN_WINDOW        0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY              0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID                0x181A                             /**< Target device name that application is looking for. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                           /**< UART RX buffer size. */

#define UART_RX_PIN_NUMBER 11
#define UART_TX_PIN_NUMBER 9
#define UART_RTS_PIN_NUMBER 10
#define UART_CTS_PIN_NUMBER 8
#define LED_1 21
#define BUTTON_1 17

// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif
                                 
                                 
/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                             /**< Pointer to data. */
    uint16_t      data_len;                                           /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,                                                     /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                              /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                   /**< Fast advertising running. */
} ble_scan_mode_t;

static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
static ble_hrs_c_t                  m_ble_hrs_c;                         /**< Structure used to identify the heart rate client module. */
static ble_bas_c_t                  m_ble_bas_c;                         /**< Structure used to identify the Battery Service client module. */
static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
//static ble_scan_mode_t              m_scan_mode = BLE_FAST_SCAN;         /**< Scan mode used by application. */
//static uint16_t                     m_conn_handle;                       /**< Current connection handle. */
static volatile bool                m_whitelist_temporarily_disabled = false; /**< True if whitelist has been temporarily disabled. */

static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

#define MAX_CHAR_RX 50  // Max length of strings received via the UART

typedef enum {
    BD_OP_MODE_NONE,
    BD_OP_MODE_ADV,
    BD_OP_MODE_ESS,
    BD_OP_MODE_AIO
} ble_op_mode_t;

#define MAX_NBR_DEVICES 10
typedef struct {
    ble_gap_addr_t  peer_addr;
    ble_op_mode_t   op_mode;
    uint32_t        time_between_reports;
    uint8_t         adv_data[31];
    uint8_t         adv_data_len;
    uint8_t         scan_resp[31];
    uint8_t         scan_resp_len;
    uint32_t        time_last_seen;
} device_t;
static device_t m_device_list[MAX_NBR_DEVICES];

//static const nrf_drv_spis_t m_spi_slave_1 = NRF_DRV_SPIS_INSTANCE(1);
//static volatile uint8_t m_slave_tx_buffer[50];
//static volatile uint8_t m_slave_rx_buffer[50];

APP_TIMER_DEF(m_timer_id);

/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static void scan_start(void);
static uint32_t uart_write_str(uint8_t *str, uint8_t len);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hrs_on_db_disc_evt(&m_ble_hrs_c, p_evt);
    ble_bas_on_db_disc_evt(&m_ble_bas_c, p_evt);
}


/**@brief Callback handling device manager events.
 *
 * @details This function is called to notify the application of device manager events.
 *
 * @param[in]   p_handle      Device Manager Handle. For link related events, this parameter
 *                            identifies the peer.
 * @param[in]   p_event       Pointer to the device manager event.
 * @param[in]   event_status  Status of the event.
 */
static ret_code_t device_manager_event_handler(const dm_handle_t    * p_handle,
                                                 const dm_event_t     * p_event,
                                                 const ret_code_t     event_result)
{
    uint32_t err_code;

    switch (p_event->event_id)
    {
        case DM_EVT_CONNECTION:
        {
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_CONNECTION\r\n");
#ifdef ENABLE_DEBUG_LOG_SUPPORT
            ble_gap_addr_t * peer_addr;
            peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;
            APPL_LOG_DEBUG("[APPL]:[%02X %02X %02X %02X %02X %02X]: Connection Established\r\n",
                                peer_addr->addr[0], peer_addr->addr[1], peer_addr->addr[2],
                                peer_addr->addr[3], peer_addr->addr[4], peer_addr->addr[5]);
#endif // ENABLE_DEBUG_LOG_SUPPORT
            
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);

//            m_conn_handle = p_event->event_param.p_gap_param->conn_handle;

            m_dm_device_handle = (*p_handle);

            // Initiate bonding.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);

            m_peer_count++;

            if (m_peer_count < CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_CONNECTION\r\n");
            break;
        }

        case DM_EVT_DISCONNECTION:
        {
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DISCONNECTION\r\n");
            memset(&m_ble_db_discovery, 0 , sizeof (m_ble_db_discovery));

//            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//            APP_ERROR_CHECK(err_code);

            if (m_peer_count == CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            m_peer_count--;
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DISCONNECTION\r\n");
            break;
        }

        case DM_EVT_SECURITY_SETUP:
        {
            APPL_LOG_DEBUG("[APPL]:[0x%02X] >> DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            // Slave securtiy request received from peer, if from a non bonded device, 
            // initiate security setup, else, wait for encryption to complete.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG_DEBUG("[APPL]:[0x%02X] << DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            break;
        }

        case DM_EVT_SECURITY_SETUP_COMPLETE:
        {
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            break;
        }

        case DM_EVT_LINK_SECURED:
            APPL_LOG_DEBUG("[APPL]: >> DM_LINK_SECURED_IND\r\n");
            // Discover peer's services. 
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_event->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG_DEBUG("[APPL]: << DM_LINK_SECURED_IND\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_LOADED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_LINK_SECURED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_LOADED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_STORED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_DELETED:
            APPL_LOG_DEBUG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG_DEBUG("[APPL]: << DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
//static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
//{
//    uint32_t  index = 0;
//    uint8_t * p_data;

//    p_data = p_advdata->p_data;

//    while (index < p_advdata->data_len)
//    {
//        uint8_t field_length = p_data[index];
//        uint8_t field_type   = p_data[index+1];

//        if (field_type == type)
//        {
//            p_typedata->p_data   = &p_data[index+2];
//            p_typedata->data_len = field_length-1;
//            return NRF_SUCCESS;
//        }
//        index += field_length + 1;
//    }
//    return NRF_ERROR_NOT_FOUND;
//}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
//static void sleep_mode_enter(void)
//{
//    uint32_t err_code;
////    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
////    APP_ERROR_CHECK(err_code);

//    // Prepare wakeup buttons.
////    err_code = bsp_btn_ble_sleep_mode_prepare();
////    APP_ERROR_CHECK(err_code);

//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
//}


static void device_adv_print(ble_gap_evt_adv_report_t adv_report)
{
    APPL_LOG("[APPL]: Device found\r\n");
    for (int i=5; i > 0; i--) {
        APPL_LOG("%02X:", adv_report.peer_addr.addr[i]);
    }
    APPL_LOG("%02X\r\n", adv_report.peer_addr.addr[0]);
    APPL_LOG("rssi: %d\r\n", adv_report.rssi);    
    APPL_LOG("\r\n");    
}


//static void extract_sensor_data(device_t *device, char *sensorData, uint8_t len)
//{
//    uint8_t adv_message[62] = {0};
//    
//    // Add both adv_data and scan_resp to one array
//    for (int i = 0; i < device->adv_data_len; i++) {
//        adv_message[i] = device->adv_data[i];
//    }
//    for (int i = 0; i < device->scan_resp_len; i++) {
//        adv_message[i + device->adv_data_len] = device->scan_resp[i];
//    }
//    
//    for (int i=0; i < (device->adv_data_len + device->scan_resp_len); i++) {
//        APPL_LOG("%02x ", adv_message[i]);
//    }
//    APPL_LOG("\r\n");

//    uint32_t  index = 0;

//    while (index < (device->adv_data_len + device->scan_resp_len)) {
//        uint8_t field_length = adv_message[index];
//        uint8_t field_type   = adv_message[index+1];
//        APPL_LOG("Field length:%d\r\n", field_length);
//        APPL_LOG("Field type:%02x\r\n", field_type);

//        if (field_type == 0xFF) {
//            APPL_LOG("Field type 0xFF found\r\n");
//            sensorData[0] = field_length - 2;
//            for (int i = 0; i < field_length; i++) {
//                sensorData[i + 1] = adv_message[i + index + 4];
//            }
//            break;
//        }
//        index += field_length + 1;
//    }

//    for (int i=0; i < len; i++) {
//        APPL_LOG("%02x ", sensorData[i], sensorData[i]);
//    }
//    APPL_LOG("\r\n");
//        
////    index = 0;
////    
////    while (index 
////    return NRF_ERROR_NOT_FOUND;
//}

void get_battery(uint8_t device_index, uint8_t *battery_level)
{
    uint8_t index = 0;
    
    *battery_level = 0;
    
    while (index < m_device_list[device_index].adv_data_len) {
        uint8_t fieldLength = m_device_list[device_index].adv_data[index];
        uint8_t fieldType   = m_device_list[device_index].adv_data[index+1];

        if (fieldType == 0x16) {
            uint8_t k = index + 2;
            uint16_t uuid = (m_device_list[device_index].adv_data[k+1] << 8) | m_device_list[device_index].adv_data[k];
                if (uuid == 0x180F) {
                    *battery_level = m_device_list[device_index].adv_data[k+2];
                    break;
                }
        }

        index += fieldLength + 1;
    }    
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;
    //int8_t index = -1;
    uint8_t device_index = 0;
    uint32_t time_now;
    uint32_t time_diff;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
//            data_t type_data;
            
            // Initialize advertisement report for parsing.
            adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;

//            device_adv_print(p_gap_evt->params.adv_report);
            
            bool device_found = false;
            
            // Check if sdvertising from device in device list
            for (int i = 0; i < MAX_NBR_DEVICES; i++) {
                if ((p_gap_evt->params.adv_report.peer_addr.addr[0] == m_device_list[i].peer_addr.addr[0]) &&
                    (p_gap_evt->params.adv_report.peer_addr.addr[1] == m_device_list[i].peer_addr.addr[1]) &&
                    (p_gap_evt->params.adv_report.peer_addr.addr[2] == m_device_list[i].peer_addr.addr[2]) &&
                    (p_gap_evt->params.adv_report.peer_addr.addr[3] == m_device_list[i].peer_addr.addr[3]) &&
                    (p_gap_evt->params.adv_report.peer_addr.addr[4] == m_device_list[i].peer_addr.addr[4]) &&
                    (p_gap_evt->params.adv_report.peer_addr.addr[5] == m_device_list[i].peer_addr.addr[5]))
                {
                        device_found = true;
                        device_index = i;
                        break;
                }
            }
            
            if (device_found) {
//                APPL_LOG("[APPL]: Device found: %d\r\n", device_index);
                
                // Store advertising data for device in device list
                if (p_gap_evt->params.adv_report.scan_rsp == 0) {
                    for (int i = 0; i < adv_data.data_len; i++) {
                        m_device_list[device_index].adv_data[i] = adv_data.p_data[i];
                    }
                    m_device_list[device_index].adv_data_len = adv_data.data_len;
                } else {
                    for (int i = 0; i < adv_data.data_len; i++) {
                        m_device_list[device_index].scan_resp[i] = adv_data.p_data[i];
                    }
                    m_device_list[device_index].scan_resp_len = adv_data.data_len;
                }
                
                err_code = app_timer_cnt_get(&time_now);
                APP_ERROR_CHECK(err_code);
//                APPL_LOG("[APPL]: Last seen: %d\r\n", m_device_list[device_index].time_last_seen);
//                APPL_LOG("[APPL]: Time Now: %d\r\n", time_now);
                err_code = app_timer_cnt_diff_compute(time_now, m_device_list[device_index].time_last_seen, &time_diff);
                
                if (time_diff > (60 * 32768)) {
                    m_device_list[device_index].time_last_seen = time_now;
                    
                    uint8_t var = 2;
                    if (var == 0) {
                    if ((m_device_list[device_index].adv_data_len > 0) && (m_device_list[device_index].scan_resp_len > 0)) {
                        char adv_str[150] = {0};
                        char tempStr[10] = {0};
                        
                        sprintf(adv_str, "+ADV:%02d,%02X,", device_index, (m_device_list[device_index].adv_data_len+m_device_list[device_index].scan_resp_len));
                        for (int i = 0; i < m_device_list[device_index].adv_data_len; i++) {
                            sprintf(tempStr, "%02X", m_device_list[device_index].adv_data[i]);
                            strcat(adv_str, tempStr);
                        }
                        for (int i = 0; i < m_device_list[device_index].scan_resp_len; i++) {
                            sprintf(tempStr, "%02X", m_device_list[device_index].scan_resp[i]);
                            strcat(adv_str, tempStr);
                        }
                        strcat(adv_str, "\r\n");
                        APPL_LOG("[APPL]: %s\r\n", adv_str);
                        uart_write_str((uint8_t *)adv_str, strlen(adv_str));
                    }
                    } 
                    if (var == 1) {
                    if ((m_device_list[device_index].adv_data_len > 0) && (m_device_list[device_index].scan_resp_len > 0)) {
                        char adv_str[150] = {0};
                        
                        sprintf(adv_str, "+ADV:%02d", device_index);
                        strcat(adv_str, "\r\n");
                        APPL_LOG("[APPL]: %s\r\n", adv_str);
                        uart_write_str((uint8_t *)adv_str, strlen(adv_str));
                    }
                    }
                    if (var == 2) {
                    if ((m_device_list[device_index].adv_data_len > 0) && (m_device_list[device_index].scan_resp_len > 0)) {
                        char adv_str[150] = {0};
                        char tempStr[10] = {0};
                        
                        uint8_t battery_level;
                        
                        get_battery(device_index, &battery_level);
                        uint8_t sensor_data_len = m_device_list[device_index].scan_resp_len - 4;
                        sprintf(adv_str, "+ADV:%02d,%02X,%02X,", device_index, sensor_data_len, battery_level);
                        for (int i = 4; i < m_device_list[device_index].scan_resp_len; i++) {
                            sprintf(tempStr, "%02X", m_device_list[device_index].scan_resp[i]);
                            strcat(adv_str, tempStr);
                        }
                        strcat(adv_str, "\r\n");
                        APPL_LOG("[APPL]: %s\r\n", adv_str);
                        uart_write_str((uint8_t *)adv_str, strlen(adv_str));
                    }
                    }
                }
            }

//            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
//                                        &adv_data,
//                                        &type_data);

//            if (err_code != NRF_SUCCESS)
//            {
//                // Compare short local name in case complete name does not match.
//                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
//                                            &adv_data,
//                                            &type_data);
//            }

            // Verify if short or complete name matches target.
//            if (err_code == NRF_SUCCESS)
//            {
//                uint16_t extracted_uuid;

//                // UUIDs found, look for matching UUID
//                for (uint32_t u_index = 0; u_index < (type_data.data_len/UUID16_SIZE); u_index++)
//                {
//                    UUID16_EXTRACT(&extracted_uuid,&type_data.p_data[u_index * UUID16_SIZE]);

//                    APPL_LOG_DEBUG("\t[APPL]: %x\r\n",extracted_uuid);

////                    if(extracted_uuid == TARGET_UUID)
////                    {
////                        // Stop scanning.
////                        err_code = sd_ble_gap_scan_stop();

////                        if (err_code != NRF_SUCCESS)
////                        {
////                            APPL_LOG_DEBUG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
////                        }
//////                        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//////                        APP_ERROR_CHECK(err_code);

////                        m_scan_param.selective = 0; 
////                        m_scan_param.p_whitelist = NULL;

////                        // Initiate connection.
////                        err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
////                                                      &m_scan_param,
////                                                      &m_connection_param);

////                        m_whitelist_temporarily_disabled = false;

////                        if (err_code != NRF_SUCCESS)
////                        {
////                            APPL_LOG_DEBUG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
////                        }
////                        break;
////                    }
//                }
//            }
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG_DEBUG("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG_DEBUG("[APPL]: Connection Request timed out.\r\n");
            }
            break;
        case BLE_GAP_EVT_CONNECTED:
        {
            err_code = ble_hrs_c_handles_assign(&m_ble_hrs_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = ble_bas_c_handles_assign(&m_ble_bas_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_hrs_c_on_ble_evt(&m_ble_hrs_c, p_ble_evt);
    ble_bas_c_on_ble_evt(&m_ble_bas_c, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
		
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
		
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof (ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    register_param.evt_handler            = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.sec_param.kdist_peer.enc = 1;
    register_param.sec_param.kdist_peer.id  = 1;

    err_code = dm_register(&m_dm_app_id, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
//static void whitelist_disable(void)
//{
//    uint32_t err_code;

//    if ((m_scan_mode == BLE_WHITELIST_SCAN) && !m_whitelist_temporarily_disabled)
//    {
//        m_whitelist_temporarily_disabled = true;

//        err_code = sd_ble_gap_scan_stop();
//        if (err_code == NRF_SUCCESS)
//        {
//            scan_start();
//        }
//        else if (err_code != NRF_ERROR_INVALID_STATE)
//        {
//            APP_ERROR_CHECK(err_code);
//        }
//    }
//    m_whitelist_temporarily_disabled = true;
//}


/**@brief Heart Rate Collector Handler.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    uint32_t err_code;

    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:

            // Heart rate service discovered. Enable notification of Heart Rate Measurement.
            err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
            APP_ERROR_CHECK(err_code);

            APPL_LOG_DEBUG("Heart rate service discovered \r\n");
            break;

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            APPL_LOG_DEBUG("[APPL]: HR Measurement received %d \r\n", p_hrs_c_evt->params.hrm.hr_value);

            APPL_LOG("Heart Rate = %d\r\n", p_hrs_c_evt->params.hrm.hr_value);
            break;
        }

        default:
            break;
    }
}


/**@brief Battery levelCollector Handler.
 */
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    uint32_t err_code;

    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
            // Batttery service discovered. Enable notification of Battery Level.
            APPL_LOG_DEBUG("[APPL]: Battery Service discovered. \r\n");

            APPL_LOG_DEBUG("[APPL]: Reading battery level. \r\n");

            err_code = ble_bas_c_bl_read(p_bas_c);
            APP_ERROR_CHECK(err_code);


            APPL_LOG_DEBUG("[APPL]: Enabling Battery Level Notification. \r\n");
            err_code = ble_bas_c_bl_notif_enable(p_bas_c);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
        {
            APPL_LOG_DEBUG("[APPL]: Battery Level received %d %%\r\n", p_bas_c_evt->params.battery_level);

            APPL_LOG_DEBUG("Battery = %d %%\r\n", p_bas_c_evt->params.battery_level);
            break;
        }

        case BLE_BAS_C_EVT_BATT_READ_RESP:
        {
            APPL_LOG_DEBUG("[APPL]: Battery Level Read as %d %%\r\n", p_bas_c_evt->params.battery_level);

            APPL_LOG_DEBUG("Battery = %d %%\r\n", p_bas_c_evt->params.battery_level);
            break;
        }

        default:
            break;
    }
}


/**
 * @brief Heart rate collector initialization.
 */
static void hrs_c_init(void)
{
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler = hrs_c_evt_handler;

    uint32_t err_code = ble_hrs_c_init(&m_ble_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Battery level collector initialization.
 */
static void bas_c_init(void)
{
    ble_bas_c_init_t bas_c_init_obj;

    bas_c_init_obj.evt_handler = bas_c_evt_handler;

    uint32_t err_code = ble_bas_c_init(&m_ble_bas_c, &bas_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting default scan parameters.
 */
static void scan_set_default_params(void)
{
    m_scan_param.active       = 0;                      // Active scanning set.
    m_scan_param.selective    = 0;                      // Selective scanning not set.
    m_scan_param.interval     = DEFAULT_SCAN_INTERVAL;  // Scan interval.
    m_scan_param.window       = DEFAULT_SCAN_WINDOW;    // Scan window.
    m_scan_param.p_whitelist  = NULL;                   // No whitelist provided.
    m_scan_param.timeout      = 0x0000;                 // No timeout.
}


/**@brief Function for setting scan parameters.
 */
static void scan_set_params(uint8_t active, uint8_t selective, uint16_t interval, uint16_t window, uint16_t timeout)
{
    if ((active == 0) || (active == 1)) {
        m_scan_param.active = active;
    }
    
    if ((selective == 0) || (selective == 1)) {
        m_scan_param.selective = selective;
    }
    
    if ((interval >= 0x0004) && (interval <= 0x4000)) {
        m_scan_param.interval = interval;
    }
    
    if ((window >= 0x0004) && (window <= 0x4000)) {
        m_scan_param.window = window;
    }
    
    m_scan_param.timeout = timeout;
    
    APPL_LOG("[APPL]: Active scan: %d\r\n", m_scan_param.active);
    APPL_LOG("[APPL]: Selective scan: %d\r\n", m_scan_param.selective);
    APPL_LOG("[APPL]: Scan interval: %d\r\n", m_scan_param.interval);
    APPL_LOG("[APPL]: Scan window: %d\r\n", m_scan_param.window);
    APPL_LOG("[APPL]: Scan timeout: %d\r\n", m_scan_param.timeout);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
//    ble_gap_whitelist_t   whitelist;
//    ble_gap_addr_t      * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
//    ble_gap_irk_t       * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
    uint32_t              err_code;
    uint32_t              count;

    // Verify if there is any flash access pending, if yes delay starting scanning until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // Initialize whitelist parameters.
//    whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
//    whitelist.irk_count  = 0;
//    whitelist.pp_addrs   = p_whitelist_addr;
//    whitelist.pp_irks    = p_whitelist_irk;

//    // Request creating of whitelist.
//    err_code = dm_whitelist_create(&m_dm_app_id,&whitelist);
//    APP_ERROR_CHECK(err_code);

//    if (((whitelist.addr_count == 0) && (whitelist.irk_count == 0)) ||
//        (m_scan_mode != BLE_WHITELIST_SCAN)                        ||
//        (m_whitelist_temporarily_disabled))
//    {
//        // No devices in whitelist, hence non selective performed.
//        m_scan_param.active       = 0;            // Active scanning set.
//        m_scan_param.selective    = 0;            // Selective scanning not set.
//        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
//        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
//        m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
//        m_scan_param.timeout      = 0x0000;       // No timeout.
//    }
//    else
//    {
//        // Selective scanning based on whitelist first.
//        m_scan_param.active       = 0;            // Active scanning set.
//        m_scan_param.selective    = 1;            // Selective scanning not set.
//        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
//        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
//        m_scan_param.p_whitelist  = &whitelist;   // Provide whitelist.
//        m_scan_param.timeout      = 0x001E;       // 30 seconds timeout.
//    }

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to stop scanning.
 */
static void scan_stop(void)
{
//    uint32_t err_code;
//    err_code = sd_ble_gap_scan_stop();
//    APP_ERROR_CHECK(err_code);
    sd_ble_gap_scan_stop();
}


/**@brief Function for GPIO initialization.
 *
 * @details Initializes the GPIOs used by the application.
 */
static void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_1);
}


/**@brief Function for initializing the nrf log module.
 */
static void nrf_log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for writing a string to the UART.
 */
static uint32_t uart_write_str(uint8_t *str, uint8_t len)
{
    uint32_t err_code;
    volatile uint8_t hej;
    
    for (int i=0; i < len; i++) {
        err_code = app_uart_put(str[i]);
        if (err_code != NRF_SUCCESS) {
            break;;
        }
    }
    
    return err_code;
}


/**@brief Function for adding a new device to the device list.
 */
static uint32_t add_device_to_list(ble_gap_addr_t addr)
{
    int8_t index_of_found_device = -1;
    
    // Find first free position
    for (int i=0; i < MAX_NBR_DEVICES; i++) {
        if (m_device_list[i].peer_addr.addr[5] == 0) {
            index_of_found_device = i;
            break;
        }
    }
    
    // Add device if free position found
    if (index_of_found_device == -1) {
        return 1;
    } else {
        for (int i = BLE_GAP_ADDR_LEN - 1; i >= 0; i--) {
            m_device_list[index_of_found_device].peer_addr.addr[i] = addr.addr[i];
        }
        m_device_list[index_of_found_device].peer_addr.addr_type = addr.addr_type;
    }

    // Debug print of device list
    for (int i=0; i < MAX_NBR_DEVICES; i++) {
        APPL_LOG("[APPL]: *** Device #%02d ***\r\n", i);
        APPL_LOG("[APPL]: Address: %02X:", m_device_list[i].peer_addr.addr[5]);
        APPL_LOG("%02X:", m_device_list[i].peer_addr.addr[4]);
        APPL_LOG("%02X:", m_device_list[i].peer_addr.addr[3]);
        APPL_LOG("%02X:", m_device_list[i].peer_addr.addr[2]);
        APPL_LOG("%02X:", m_device_list[i].peer_addr.addr[1]);
        APPL_LOG("%02X\r\n", m_device_list[i].peer_addr.addr[0]);
        APPL_LOG("[APPL]: Address type: %d\r\n", m_device_list[i].peer_addr.addr_type);
    }
    
    return 0;
}


/**@brief Function for writing the list of all devices to the UART.
 */
static uint32_t list_device(uint8_t index)
{
    char str[50] = {0};
    
    if (index < MAX_NBR_DEVICES) {
        sprintf(str, "*** Device #%02d ***\r\n", index);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "Address: %02X:", m_device_list[index].peer_addr.addr[5]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "%02X:", m_device_list[index].peer_addr.addr[4]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "%02X:", m_device_list[index].peer_addr.addr[3]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "%02X:", m_device_list[index].peer_addr.addr[2]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "%02X:", m_device_list[index].peer_addr.addr[1]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "%02X\r\n", m_device_list[index].peer_addr.addr[0]);
        uart_write_str((uint8_t *)str, strlen(str));
        sprintf(str, "Address type: %d\r\n", m_device_list[index].peer_addr.addr_type);
        uart_write_str((uint8_t *)str, strlen(str));
        return 0;
    } else {
        return 1;
    }
}


static uint32_t send_adv_data(uint8_t index)
{
    char adv_str[150] = {0};
    char tempStr[10] = {0};
                        
    sprintf(adv_str, "+ADV:%02d,%02X,", index, (m_device_list[index].adv_data_len+m_device_list[index].scan_resp_len));
    for (int i = 0; i < m_device_list[index].adv_data_len; i++) {
        sprintf(tempStr, "%02X", m_device_list[index].adv_data[i]);
        strcat(adv_str, tempStr);
    }
    for (int i = 0; i < m_device_list[index].scan_resp_len; i++) {
        sprintf(tempStr, "%02X", m_device_list[index].scan_resp[i]);
        strcat(adv_str, tempStr);
    }
    strcat(adv_str, "\r\n");
    APPL_LOG("[APPL]: %s\r\n", adv_str);
    uart_write_str((uint8_t *)adv_str, strlen(adv_str));
                        
    return 0;
}

static void uart_rx_command_handler(uint8_t *rx, uint8_t rx_len)
{
    uint32_t err_code;
    
    int i = 0;
    while (rx[i]) {
        rx[i] = toupper(rx[i]);
        i++;
        if (i == rx_len) {
            break;
        }
    }
    
    APPL_LOG("[APPL]: Command received: %s\r\n", rx);
    
    if (strcmp((char *)rx, "AT") == 0) {
        uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        APPL_LOG("[APPL]: Response sent\r\n");
        
	} else if (strcmp((char *)rx, "ATZ") == 0) {
        // TODO: Implement command
				// Stop scanning
				scan_stop();
				// TODO: Set default scan parameters
				// Erase device list
				memset(m_device_list, 0, sizeof(m_device_list));
        if (true) {
            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strcmp((char *)rx, "AT+INFO?") == 0) {
        // TODO: Implement command
		uart_write_str((uint8_t *)"FW:\r\n", strlen("OK\r\n"));	
        if (true) {
            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strcmp((char *)rx, "AT+SCANSTART") == 0) {
        scan_start();
        // TODO: Check if scan started OK?
        if (true) {
            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strcmp((char *)rx, "AT+SCANSTOP") == 0) {
        scan_stop();
        // TODO: Check if scan stopped OK?
        if (true) {
            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strncmp((char *)rx, "AT+SCANSETP", strlen("AT+SCANSETP")) == 0) {
        uint8_t command[12];
        uint8_t active, selective;
        uint16_t interval, window, timeout;
        uint8_t nbr_of_variables = sscanf((char *)rx, "%11c=%hhd,%hhd,%hd,%hd,%hd", command, &active, &selective, &interval, &window, &timeout);
        if (nbr_of_variables == 6) {
            scan_set_params(active, selective, interval, window, timeout);
            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strncmp((char *)rx, "AT+DEVADD", strlen("AT+DEVADD")) == 0) {
        uint8_t command[10];
        ble_gap_addr_t a;
        uint8_t nbr_of_variables = sscanf((char *)rx, "%9c=%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", command, &a.addr[5], &a.addr[4], &a.addr[3], &a.addr[2], &a.addr[1], &a.addr[0]);
        if (nbr_of_variables == 7) {
            err_code = add_device_to_list(a);
            if (err_code == 0) {
                uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
            } else {
                uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
            }
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strncmp((char *)rx, "AT+DEVLIST", strlen("AT+DEVLIST")) == 0) {
        uint8_t command[11];
        uint8_t nbr;
        uint8_t nbr_of_variables = sscanf((char *)rx, "%10c=%hhd", command, &nbr);
        if (nbr_of_variables == 2) {
            err_code = list_device(nbr);
            if (err_code == 0) {
                uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
            } else {
                uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
            }
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    } else if (strncmp((char *)rx, "AT+ADV", strlen("AT+ADV")) == 0) {
        uint8_t command[7];
        uint8_t nbr;
        uint8_t nbr_of_variables = sscanf((char *)rx, "%6c=%hhd", command, &nbr);
        if (nbr_of_variables == 2) {
            err_code = send_adv_data(nbr);
            if (err_code == 0) {
                uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
            } else {
                uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
            }
        } else {
            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
        }
        APPL_LOG("[APPL]: Response sent\r\n");
        
    }

}


static void uart_event_handler(app_uart_evt_t *p_event)
{
    uint8_t c;
//    uint32_t err_code;
    static uint8_t rx_data[MAX_CHAR_RX];
    static uint8_t index;
    
    switch (p_event->evt_type) {
        case APP_UART_DATA_READY:
            /**< An event indicating that UART data has been received. 
            The data is available in the FIFO and can be fetched using @ref app_uart_get. */
            app_uart_get(&c);
//            err_code = app_uart_get(&c);
//            APP_ERROR_CHECK(err_code);
            if (c == '\n') {
                // Full command received
                rx_data[index - 1] = 0;
                uart_rx_command_handler(rx_data, index);
                // Reset command buffer
                index = 0;
                memset(rx_data, 0, sizeof(rx_data));
//            } else if (c == '\n') {
                // Do nothing, ignore all newlines
            } else {
                // Add to command buffer
                if (index < MAX_CHAR_RX) {
                    rx_data[index] = c;
                    index++;
                } else {
                    // Pass error message, string too long
                }
            }
            break;
        
        case APP_UART_FIFO_ERROR:
            /**< An error in the FIFO module used by the app_uart module has occured. 
            The FIFO error code is stored in app_uart_evt_t.data.error_code field. */
            APPL_LOG("[APPL]: APP_UART_FIFO_ERROR\r\n");
            index = 0;
            break;
        
        case APP_UART_COMMUNICATION_ERROR:
            /**< An communication error has occured during reception. 
            The error is stored in app_uart_evt_t.data.error_communication field. */
            APPL_LOG("[APPL]: APP_UART_COMMUNICATION_ERROR: %d\r\n", p_event->data.error_communication);
            NRF_UART0->ERRORSRC = 0x0F;
            index = 0;
            app_uart_flush();
            break;
        
        case APP_UART_TX_EMPTY:
            /**< An event indicating that UART has completed transmission of all available data in the TX FIFO. */
            //APPL_LOG("[APPL]: APP_UART_TX_EMPTY\r\n");
            break;
        
        case APP_UART_DATA:
            /**< An event indicating that UART data has been received, and data is present in data field. 
            This event is only used when no FIFO is configured. */
            // Should not happen as the FIFO is used
            APPL_LOG("[APPL]: APP_UART_DATA\r\n");
            break;
    }
}


/**@brief Function for initializing the uart module.
 */
static void uart_init(void)
{
    ret_code_t err_code;
    
    const app_uart_comm_params_t comm_params =
    {
        UART_RX_PIN_NUMBER,
        UART_TX_PIN_NUMBER,
        UART_RTS_PIN_NUMBER,
        UART_CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud460800
//        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}


//static void spi_command_handler(uint8_t *rx, uint8_t rx_len, uint8_t *tx, uint8_t *tx_len)
//{
//    uint32_t err_code;
//    
//    int i = 0;
//    while (rx[i]) {
//        rx[i] = toupper(rx[i]);
//        i++;
//        if (i == rx_len) {
//            break;
//        }
//    }
//    
//    APPL_LOG("[APPL]: Command received: %s\r\n", rx);
//    
//    if (strcmp((char *)rx, "ATZ") == 0) {
//        // TODO: Implement command
//        strcpy((char *)tx, "ATZ: COMMAND NOT IMPLEMENTED YET\r\n");
//        *tx_len = strlen((char *)tx);
////        uart_write_str((uint8_t *)"ATZ: COMMAND NOT IMPLEMENTED YET\r\n", strlen("ATZ: COMMAND NOT IMPLEMENTED YET\r\n"));
//        APPL_LOG("[APPL]: %s\r\n", "ATZ: COMMAND NOT IMPLEMENTED YET");
//        
//    } else if (strcmp((char *)rx, "AT+SCANSTART") == 0) {
//        scan_start();
//        // TODO: Check if scan started OK?
//        if (true) {
//            strcpy((char *)tx, "OK\r\n");
//            *tx_len = strlen((char *)tx);
////            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
//        } else {
////            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//        }
//        
//    } else if (strcmp((char *)rx, "AT+SCANSTOP") == 0) {
//        scan_stop();
//        // TODO: Check if scan stopped OK?
//        if (true) {
////            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
//        } else {
////            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//        }
//        
//    } else if (strncmp((char *)rx, "AT+SCANSETP", strlen("AT+SCANSETP")) == 0) {
//        uint8_t command[12];
//        uint8_t active, selective;
//        uint16_t interval, window, timeout;
//        uint8_t nbr_of_variables = sscanf((char *)rx, "%11c=%hhd,%hhd,%hd,%hd,%hd", command, &active, &selective, &interval, &window, &timeout);
//        if (nbr_of_variables == 6) {
//            scan_set_params(active, selective, interval, window, timeout);
////            uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
//        } else {
////            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//        }
//        
//    } else if (strncmp((char *)rx, "AT+DEVADD", strlen("AT+DEVADD")) == 0) {
//        uint8_t command[10];
//        ble_gap_addr_t a;
//        uint8_t nbr_of_variables = sscanf((char *)rx, "%9c=%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", command, &a.addr[5], &a.addr[4], &a.addr[3], &a.addr[2], &a.addr[1], &a.addr[0]);
//        if (nbr_of_variables == 7) {
//            err_code = add_device_to_list(a);
//            if (err_code == 0) {
////                uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
//            } else {
////                uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//            }
//        } else {
////            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//        }
//        
//    } else if (strncmp((char *)rx, "AT+DEVLIST", strlen("AT+DEVLIST")) == 0) {
//        uint8_t command[11];
//        uint8_t nbr;
//        uint8_t nbr_of_variables = sscanf((char *)rx, "%10c=%hhd", command, &nbr);
//        if (nbr_of_variables == 2) {
//            err_code = list_device(nbr);
//            if (err_code == 0) {
////                uart_write_str((uint8_t *)"OK\r\n", strlen("OK\r\n"));
//            } else {
////                uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//            }
//        } else {
////            uart_write_str((uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"));
//        }
//        
//    }
//    
//}


//static void spi_slave_handler(nrf_drv_spis_event_t event)
//{
//    uint32_t err_code;
//    uint8_t tx_len = 0;
//    
//    APPL_LOG("[APPL]: spi_slave_handler\r\n");
//    
//    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
//    {
//        // Received bytes: event.rx_amount.
//    }
//    
//    switch (event.evt_type) {
//        case NRF_DRV_SPIS_BUFFERS_SET_DONE:
//            /**< Memory buffer set event. Memory buffers have been set successfully 
//            to the SPI slave device, and SPI transactions can be done. */
//            APPL_LOG("[APPL]: spi_slave_handler: NRF_DRV_SPIS_BUFFERS_SET_DONE\r\n");
//            break;

//        case NRF_DRV_SPIS_XFER_DONE:
//            /**< SPI transaction event. SPI transaction has been completed. */  
//            APPL_LOG("[APPL]: spi_slave_handler: NRF_DRV_SPIS_XFER_DONE\r\n");
//            for (int i = 0; i < event.rx_amount; i++) {
//                APPL_LOG("%d ", m_slave_rx_buffer[i]);
//            }
//            APPL_LOG("\r\n");
//            if (m_slave_rx_buffer[0] == 0) {
//                tx_len = 0;
//            } else {
//                spi_command_handler((uint8_t *)m_slave_rx_buffer, sizeof(m_slave_rx_buffer), (uint8_t *)m_slave_tx_buffer, &tx_len);
//            }
//            err_code = nrf_drv_spis_buffers_set(&m_spi_slave_1,
//                                       (uint8_t*) m_slave_tx_buffer, sizeof(m_slave_tx_buffer),
//                                       (uint8_t*) m_slave_rx_buffer, sizeof(m_slave_rx_buffer));
//            if (err_code != NRF_SUCCESS) {
//                // Buffer setup failed. Take recovery action.
//            }
//            break;
//        
//        case NRF_DRV_SPIS_EVT_TYPE_MAX:
//            /**< Enumeration upper bound. */      
//            APPL_LOG("[APPL]: spi_slave_handler: NRF_DRV_SPIS_EVT_TYPE_MAX - Should never happen\r\n");
//            break;
//    }
//}


/**@brief Function for initializing the SPI slave module.
 */
//static void spis_init(void)
//{
//    uint32_t err_code;
//    
//    nrf_drv_spis_config_t config = NRF_DRV_SPIS_DEFAULT_CONFIG(1);
////    config.mode      = NRF_DRV_SPIS_MODE_0;
////    config.bit_order = NRF_DRV_SPIS_BIT_ORDER_MSB_FIRST;
//    config.csn_pin = 9;
//    err_code = nrf_drv_spis_init(&m_spi_slave_1, &config, spi_slave_handler);
//    if (err_code != NRF_SUCCESS)
//    {
//        // Initialization failed. Take recovery action.
//    }
//    
//    err_code = nrf_drv_spis_buffers_set(&m_spi_slave_1,
//                                       (uint8_t*) m_slave_tx_buffer, sizeof(m_slave_tx_buffer),
//                                       (uint8_t*) m_slave_rx_buffer, sizeof(m_slave_rx_buffer));
//    if (err_code != NRF_SUCCESS)
//    {
//        // Buffer setup failed. Take recovery action.
//    }
//}


static void timer_timeout_handler(void * p_context)
{
    // Do nothing
}


static void radio_notification_blink(bool radio_active)
{
    if (radio_active) {
        nrf_gpio_pin_set(LED_1);
    } else {
        nrf_gpio_pin_clear(LED_1);
    }
}


static void radio_notification_init()
{
    uint32_t err_code;
    err_code = ble_radio_notification_init(APP_IRQ_PRIORITY_LOW, NRF_RADIO_NOTIFICATION_DISTANCE_800US, radio_notification_blink);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    uint32_t err_code;
    bool erase_bonds = false;

    // Initialize.
    gpio_init();
    nrf_gpio_pin_set(LED_1);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    err_code = app_timer_create(&m_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
//    buttons_leds_init(&erase_bonds);
    nrf_log_init();
    uart_init();
//    spis_init();
    APPL_LOG("Heart rate collector example\r\n");
    ble_stack_init();
    device_manager_init(erase_bonds);
    db_discovery_init();
    hrs_c_init();
    bas_c_init();
    radio_notification_init();
    nrf_gpio_pin_clear(LED_1);

    // Start scanning for peripherals and initiate connection
    // with devices that advertise Heart Rate UUID.
    scan_set_default_params();
    //scan_start();

    for (;; )
    {
        power_manage();
    }
}


