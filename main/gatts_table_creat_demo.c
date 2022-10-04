/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_system.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp32/rom/uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"



const char *TAG = "ESP32";
bool Flag_Wait_Exit;
bool Flag_Device_Ready;
int8_t Flag_Find_near = 0;
int ledState = 0;
uint32_t Flag_led = 1;
uint8_t adv_config_done = 0;

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define BUTTON				    0
#define LED_BUILD_IN 			2
#define Sim_Tx 17
#define Sim_Rx 16
#define Uart_Port_Num 2
#define Uart_Baud_Rate 115200
#define PowerKey 2 // PWR 7070
#define DAM_BUF_TX 512
#define DAM_BUF_RX 2049
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
uint8_t rxbuf[256];
static uart_isr_handle_t *handle_console;
#define EX_UART_NUM UART_NUM_2
//static QueueHandle_t uart0_queue;
#define PATTERN_CHR_NUM    (3)
#define GPIO_INPUT_PIN_SEL  	(1 << BUTTON)

typedef enum
{
    EVEN_OK = 0,
    EVEN_TIMEOUT,
    EVEN_ERROR,
	EVEN_SMS
} SIMCOM_ResponseEvent_t;
typedef void (*SIMCOM_SendATCallBack_t)(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
SIMCOM_ResponseEvent_t AT_RX_event;

uint16_t Reboot7070_Delay_Counter = 0;
typedef struct{
	char CMD[DAM_BUF_TX];
	uint32_t lenCMD;
	char ExpectResponseFromATC[20];
	uint32_t TimeoutATC;
	uint32_t CurrentTimeoutATC;
	uint8_t RetryCountATC;
	SIMCOM_SendATCallBack_t SendATCallBack;
	char AT_RX_Data[256];
	bool Flag_Device_Ready;
}ATCommand_t;
typedef struct{
	char index[3];
	char content[100];
}SMS_t;


ATCommand_t SIMCOM_ATCommand;
SMS_t SIMCOM_SMS;

nvs_handle_t nvsHandle;

QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t qSMS = NULL;
QueueHandle_t qFind = NULL;
QueueHandle_t qLed = NULL;


#define GATTS_TABLE_TAG "GATTS_DUY"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ESP_DUY"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

//static uint8_t adv_config_done       = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
//#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','U','Y'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};


static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


void TurnOn7070(void);
void SendATCommand(void);
void Init_Simcom(void);
void UART_ISR_ROUTINE(void);
static void IRAM_ATTR uart_intr_handle(void *arg);
void Init_gpio(uint32_t pgio_pin);
void ATResponse_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void ATC_SendATCommand(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction);
void ATCommand_RX_Process();
void Reset_wait();
void SMS_Recei();
void WaitandExitLoop(bool *Flag);
void blink_task(void* arg);
void Init_gpio_input();
void IRAM_ATTR gpio_isr_handler(void* arg);
void main_task(void* arg);
//void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void log_error_if_nonzero(const char *message, int error_code);
void button_status(void *arg);
void button_intr(void *arg);
void time_counter(void *arg);
void Simcom_RetrySendATC();

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    /* Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
   // #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);

    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    //#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;

            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                char string[param->write.len];
                memcpy(string,param->write.value,sizeof(string));
                ESP_LOGI(GATTS_TABLE_TAG, "value:%s", string);
               // ESP_LOGI(GATTS_TABLE_TAG,"%");
                if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            if(strstr(string,"stop_find")){
            	ESP_LOGI("BLE","Stop find\n");
            	esp_restart();
            }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void start_ble(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();

    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    while(1){
    	vTaskDelay(50/portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    nvs_flash_init();

    Init_Simcom();

    qSMS = xQueueCreate(1,sizeof(char *));
    qFind = xQueueCreate(1,sizeof(bool));
    gpio_evt_queue = xQueueCreate(1,sizeof(uint32_t));
    qLed = xQueueCreate(1,sizeof(uint32_t));


    Init_gpio(PowerKey);

    TurnOn7070();
    ESP_LOGI(TAG,"Turned on SIMCOM \n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    // release the pre registered UART handler/subroutine
    ESP_ERROR_CHECK(uart_isr_free(EX_UART_NUM));

    // register new UART subroutine
    ESP_ERROR_CHECK(uart_isr_register(EX_UART_NUM,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

    // enable RX interrupt
    ESP_ERROR_CHECK(uart_enable_rx_intr(EX_UART_NUM));
    // Send AT
    ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
    WaitandExitLoop(&Flag_Wait_Exit);
    ATC_SendATCommand("AT+CNMI=2,1\r\n", "+CNMI=2,1", 1000, 4, ATResponse_Callback);
    WaitandExitLoop(&Flag_Wait_Exit);
    ESP_LOGI(TAG,"1");
    xTaskCreate(SMS_Recei,"recei SMS",4096,NULL,10,NULL);
    ESP_LOGI(TAG,"1");
    xTaskCreate(Reset_wait,"reset wait",4096,NULL,7,NULL);
    ESP_LOGI(TAG,"1");

    if(nvs_open("storage",NVS_READWRITE, &nvsHandle) == ESP_OK){
    		nvs_get_i8(nvsHandle,"Flag_BLE",&Flag_Find_near);
    		if(Flag_Find_near == 1){ //find near
    			//esp_base_mac_addr_set(mac)
    			Init_gpio(2);
    			Init_gpio_input();
    			printf("Find Near is on!\n");
    			Flag_Find_near = 0;
    			nvs_set_i32(nvsHandle,"Flag_BLE",&Flag_Find_near);
    			nvs_commit(nvsHandle);
    			xTaskCreate(blink_task,"blink_task",4096,NULL,5,NULL);
    			xTaskCreate(button_status, "button task", 1024, NULL, 8, NULL );
    			xTaskCreate(start_ble,"start_ble",4096,NULL,9,NULL);
    			xTaskCreate(time_counter,"time counter", 1024,NULL,4,NULL);

		}
    		else if(Flag_Find_near == 0){
    			printf("Normal mode !\n");


    		}
    	}

    	while(1)
    		{
    			vTaskDelay(1000);
    		}
}

void Init_Simcom(void){
	uart_config_t  uart_config= {
			.baud_rate = Uart_Baud_Rate,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	int intr_alloc_flags = 0 ;
	uart_driver_install(Uart_Port_Num, DAM_BUF_RX, 0, 0, NULL, intr_alloc_flags);
	uart_param_config(Uart_Port_Num,&uart_config);

	uart_set_pin(Uart_Port_Num, Sim_Tx, Sim_Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//xTaskCreate(UART_ISR_ROUTINE, "Uart_echo_task1", 4096, NULL, 10, NULL);
}

void Init_gpio(uint32_t gpio_pin){
	gpio_config_t gpio_cfg ={
			.pin_bit_mask = 1ULL << gpio_pin,
			.mode = GPIO_MODE_OUTPUT,
			.pull_down_en = 0,
			.pull_up_en = 0,
			.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&gpio_cfg);
}

void TurnOn7070(void){
	gpio_set_level(PowerKey, 1);
	while(1){
		if(Reboot7070_Delay_Counter  < 40) { Reboot7070_Delay_Counter++;}
		else { Reboot7070_Delay_Counter = 0; break;}
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
	gpio_set_level(PowerKey,0);
}

 void IRAM_ATTR uart_intr_handle(void * arg){
	uint16_t rx_fifo_len,status;
	uint16_t i = 0;

	status = UART2.int_st.val;
	rx_fifo_len = UART2.status.rxfifo_cnt;

	while(rx_fifo_len){
		SIMCOM_ATCommand.AT_RX_Data[i++] = UART2.fifo.rw_byte;
		rx_fifo_len --;
	}
	uart_clear_intr_status(UART_NUM_2, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

	//memcpy(chars,rxbuf,256);
	ESP_EARLY_LOGI(TAG,"Receive Data: %s",SIMCOM_ATCommand.AT_RX_Data);
	//ATCommand_RX_Process();
	xTaskCreate(ATCommand_RX_Process, "ATCommand_RX_Process", 4096, NULL, 7, NULL );
}
 void SendATCommand(void){
	 ESP_LOGI(TAG,"Send: %s",SIMCOM_ATCommand.CMD);
	 uart_write_bytes(UART_NUM_2, (const char *)SIMCOM_ATCommand.CMD, strlen(SIMCOM_ATCommand.CMD));


 }
 void ATC_SendATCommand(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction)
 {
 	strcpy(SIMCOM_ATCommand.CMD, Command);
 	SIMCOM_ATCommand.lenCMD = strlen(SIMCOM_ATCommand.CMD);
 	strcpy(SIMCOM_ATCommand.ExpectResponseFromATC, ExpectResponse);
 	SIMCOM_ATCommand.RetryCountATC = RetryCount;
 	SIMCOM_ATCommand.SendATCallBack = CallBackFunction;
 	SIMCOM_ATCommand.TimeoutATC = Timeout;
 	SIMCOM_ATCommand.CurrentTimeoutATC = 0;

 	SendATCommand();
 }

 void ATResponse_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
 {
 	AT_RX_event = event;
 	if(event == EVEN_OK)
 	{
 		ESP_LOGW(TAG, "Device is ready to use\r\n");
 		Flag_Wait_Exit = true;
 		Flag_Device_Ready = true;
 	}
 	else if(event == EVEN_TIMEOUT)
 	{
 		ESP_LOGE(TAG, "Device is not ready \r\n");
 		Flag_Wait_Exit = true;
 	}
 	else if(event == EVEN_ERROR)
 	{
 		ESP_LOGE(TAG, "AT Check Error \r\n");
 		Flag_Wait_Exit = true;

 	}
 	else if(event == EVEN_SMS){
 		ESP_LOGE(TAG, "SMS \r\n");
		Flag_Wait_Exit = true;
 	}
 }
void ATCommand_RX_Process(void * arg){
	while(1){
		if(strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,"\"SM\"")){
			strcat(SIMCOM_ATCommand.ExpectResponseFromATC,"SM");
		}
		if(strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,"+CMGR:")){
			memset(SIMCOM_ATCommand.ExpectResponseFromATC, 0, sizeof(SIMCOM_ATCommand.ExpectResponseFromATC));
			strcat(SIMCOM_ATCommand.ExpectResponseFromATC,"+CMGR:");

		}
		if((SIMCOM_ATCommand.ExpectResponseFromATC[0]!= 0 && strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,SIMCOM_ATCommand.ExpectResponseFromATC))){
				SIMCOM_ATCommand.ExpectResponseFromATC[0] = 0;
				if(SIMCOM_ATCommand.SendATCallBack  != NULL){
					SIMCOM_ATCommand.TimeoutATC = 0;
					SIMCOM_ATCommand.SendATCallBack(EVEN_OK,SIMCOM_ATCommand.AT_RX_Data);
				}

				// check error
				if(strstr(SIMCOM_ATCommand.AT_RX_Data,"ERROR")){
					if(SIMCOM_ATCommand.SendATCallBack != NULL){
						SIMCOM_ATCommand.SendATCallBack(EVEN_ERROR,SIMCOM_ATCommand.AT_RX_Data);
					}
				}
				// check mess
				if(strstr(SIMCOM_ATCommand.AT_RX_Data,"+CMTI")){
					ESP_LOGI(TAG,"2");
					char * buff = strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,"\"SM\"")?strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,"\"SM\""):strstr((const char*)SIMCOM_ATCommand.AT_RX_Data,"\"ME\"");
					memset(SIMCOM_SMS.index, 0, sizeof(SIMCOM_SMS.index));
					strcpy(SIMCOM_SMS.index,buff+5);
					xQueueSend(qSMS,&SIMCOM_SMS.index,portMAX_DELAY);

				}
				// check SMS Content
				if(strstr(SIMCOM_ATCommand.AT_RX_Data,"CMGR:")){
					ESP_LOGI(TAG,"29");
					SIMCOM_ATCommand.SendATCallBack(EVEN_SMS,SIMCOM_ATCommand.AT_RX_Data);
					ESP_LOGI(TAG,"data1:%s\n",SIMCOM_ATCommand.AT_RX_Data);
					if(strstr((const char *)SIMCOM_ATCommand.AT_RX_Data, "FIND_NEAR")){
						ESP_LOGI(TAG,"1");
						Flag_Find_near = 1;
						char AT_Buf[20];
						strcat((char *)AT_Buf,"AT+CMGD=");
						strcat((char *)AT_Buf,(char *)SIMCOM_SMS.index);
						ATC_SendATCommand(AT_Buf, "+CMGD=", 1000, 4, ATResponse_Callback);
						xQueueSend(qFind,&Flag_Find_near,portMAX_DELAY);
					}
					if(strstr((const char *)SIMCOM_ATCommand.AT_RX_Data, "STOP_FIND")){
						ESP_LOGI(TAG,"3");
						Flag_Find_near = 1;
						char AT_Buf[20];
						strcat((char *)AT_Buf,"AT+CMGD=");
						strcat((char *)AT_Buf,(char *)SIMCOM_SMS.index);
						ATC_SendATCommand(AT_Buf, "+CMGD=", 1000, 4, ATResponse_Callback);
						esp_restart();
					}
				}
			}
		SIMCOM_ATCommand.AT_RX_Data[0] = 0;
		SIMCOM_ATCommand.ExpectResponseFromATC[0]=0;
		vTaskDelay(50 /portTICK_PERIOD_MS);
	}

}

void SMS_Recei(void * arg){
	while(1){
		if(xQueueReceive(qSMS,&SIMCOM_SMS.index, portMAX_DELAY)){
			ESP_LOGI(TAG,"1");
			if(SIMCOM_SMS.index != NULL){
				memset(SIMCOM_ATCommand.CMD, 0, sizeof(SIMCOM_ATCommand.CMD));

				ATC_SendATCommand("AT+CNMI=2,1\r\n", "+CNMI=2,1", 1000, 4, ATResponse_Callback);
				// wait code bellow this line
				WaitandExitLoop(&Flag_Wait_Exit);
				ATC_SendATCommand("AT+CMGF=1\r\n", "+CMGF=1", 1000, 4, ATResponse_Callback);
				// wait code bellow this line
				//WaitandExitLoop(&Flag_Wait_Exit);
				ESP_LOGI(TAG,"3");
				char AT_Buf[20];
				strcat((char *)AT_Buf,"AT+CMGR=");
				strcat((char *)AT_Buf,(char *)SIMCOM_SMS.index);
				strcat((char *)AT_Buf,"\r\n");
				ATC_SendATCommand(AT_Buf, "+CMGR=", 1000, 4, ATResponse_Callback);
				WaitandExitLoop(&Flag_Wait_Exit);
				AT_Buf[0] = 0;


			}
		}
		vTaskDelay(50 /portTICK_PERIOD_MS);
	}
}
void Reset_wait(void * arg){
	while(1){
		if(xQueueReceive(qFind,&Flag_Find_near,portMAX_DELAY)&&Flag_Find_near == 1){
			printf("Restart esp!");
			nvs_set_i8(nvsHandle, "Flag_BLE", (int8_t)Flag_Find_near);
			nvs_commit(nvsHandle);
			vTaskDelay(100 /portTICK_PERIOD_MS);

			esp_restart();
		}
		vTaskDelay(50 /portTICK_PERIOD_MS);
	}
}

void WaitandExitLoop(bool *Flag)
{
	int32_t count;
	count = 0;
    while(1)
    {
        if(*Flag == true)
        {

            *Flag = false;
            break;

        }

        if(count == 50 && SIMCOM_ATCommand.RetryCountATC > 0 ){
        	Simcom_RetrySendATC();
        	SIMCOM_ATCommand.RetryCountATC--;
          }
        vTaskDelay(50 /portTICK_PERIOD_MS);
        count++;
    }
}
void Simcom_RetrySendATC()
{
	SendATCommand();
}
void button_intr(void* arg){
    while (1){
    	if(xQueueReceive(qLed,&Flag_led,portMAX_DELAY)){
    		gpio_set_level(LED_BUILD_IN,0);
    		ESP_LOGI("FOUN","FOUND DONE!");
    		Flag_Find_near = 0;
    		nvs_set_i32(nvsHandle,"Flag_BLE",Flag_Find_near);
    		nvs_commit(nvsHandle);
    		esp_restart();
    	}

    }
}
void blink_task(void*arg){
	while(1){
		gpio_set_level(LED_BUILD_IN, 1);
			vTaskDelay(500 / portTICK_PERIOD_MS);

			gpio_set_level(LED_BUILD_IN, 0);
			vTaskDelay(500 / portTICK_PERIOD_MS);
	}


}
void Init_gpio_input()
{
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, gpio_isr_handler, (void*) BUTTON);
}
void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);//Copy item gpio_num vao Queue handler gpio_evt_queue
}



void button_status(void *arg){
	uint32_t gpio_num;
	TickType_t l_time = 0;
	uint32_t cnt  = 0;
	while(1){
		if(xQueueReceive(gpio_evt_queue,&gpio_num,50) ){
			if(gpio_num == 0 && gpio_get_level(gpio_num) == 0){
				ESP_LOGI("button","1");

				//vTaskDelay(200/portTICK_PERIOD_MS);
				xQueueSend(qLed,&Flag_led,portMAX_DELAY);

			}
		}
	}
}


void log_error_if_nonzero(const char *message, int error_code)
 {
     if (error_code != 0) {
         ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
     }
 }


void time_counter(void *arg){
	TickType_t startTicks = xTaskGetTickCount();
	while(xTaskGetTickCount() - startTicks < (300* 1000/portTICK_RATE_MS)){
		vTaskDelay(50/portTICK_PERIOD_MS);
	}
	ESP_LOGI("BLE","Over Time!");
	esp_restart();
}
