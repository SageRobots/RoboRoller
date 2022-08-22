#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "stepper.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
#include <hx711.h>
#include "stepper.h"
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to s
#define TIMER_INTERVAL1_S 0.00005

#define GATTS_TAG "GATTS_DEMO"

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_CHAR_UUID_TEST_A2      0xFF02
#define GATTS_NUM_HANDLE_TEST_A     7

#define TEST_DEVICE_NAME            "Muscle Roller"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};
static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {};

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

    uint16_t char_handle2;
    esp_bt_uuid_t char_uuid2;
    uint16_t descr_handle2;
    esp_bt_uuid_t descr_uuid2;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile = {};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

static esp_adc_cal_characteristics_t *adcCharsCharge, *adcCharsEStop;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_11db;

const gpio_num_t pinDirZ = (gpio_num_t)33;
const gpio_num_t pinStepZ = (gpio_num_t)25;
const gpio_num_t pinMSZ = (gpio_num_t)26;
const gpio_num_t pinEn = (gpio_num_t)32;
const gpio_num_t pinDirX = (gpio_num_t)23;
const gpio_num_t pinStepX = (gpio_num_t)22;
const gpio_num_t pinMSX = (gpio_num_t)15;
const gpio_num_t pinLCClk = (gpio_num_t)27;
const gpio_num_t pinLCDat = (gpio_num_t)14;
const gpio_num_t pinLCClk2 = (gpio_num_t)12;
const gpio_num_t pinLCDat2 = (gpio_num_t)13;
const gpio_num_t pinHomZ = (gpio_num_t)19;
const gpio_num_t pinHomX = (gpio_num_t)21;
const gpio_num_t pinClk = (gpio_num_t)4;
const gpio_num_t pinMOSI = (gpio_num_t)5;
const gpio_num_t pinMISO = (gpio_num_t)16;
const gpio_num_t pinCS0 = (gpio_num_t)17;
const gpio_num_t pinCS1 = (gpio_num_t)18;

const adc1_channel_t pinCharge = ADC1_CHANNEL_0;
const adc2_channel_t pinEStop = ADC2_CHANNEL_2;

//init motors
Stepper motorX(pinStepX, pinDirX, pinHomX, pinCS0, pinMSX, true);
Stepper motorZ(pinStepZ, pinDirZ, pinHomZ, pinCS1, pinMSZ, true);

float battery;

struct cycle {
    bool cycling;
    int step;
    float force;
    float startX;
    float endX;
    float startZ;
    float travelSpeed;
    float speed;
};

struct cycle cycle;

int64_t timeNow;
int64_t timePrint = 0;

const float LC1Zero = 187200; //near side
const float LC125lb = -392000;
const float LC2Zero = -92093; //far side
const float LC225lb = 479727;
float forceL, forceR;

template <typename Out>
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    }
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        if(param->read.handle == gl_profile.char_handle2) {
            std::string str = "";
            //motor x homed, home sensor, position, target, speed
            if(motorX.homed) str += "1,";
            else str += "0,";

            if(gpio_get_level(pinHomX)) str += "0,";
            else str += "1,";

            str += std::to_string(motorX.position).substr(0,3);
            str += ",";
            str += std::to_string(motorX.target).substr(0,3);
            str += ",";
            str += std::to_string(motorX.speed).substr(0,3);
            str += ",";

            //motor z homed, home sensor, position, target, speed, force, targetForce
            if(motorZ.homed) str += "1,";
            else str += "0,";

            if(gpio_get_level(pinHomZ)) str += "0,";
            else str += "1,";

            str += std::to_string(motorZ.position).substr(0,3);
            str += ",";
            str += std::to_string(motorZ.target).substr(0,3);
            str += ",";
            str += std::to_string(motorZ.speed).substr(0,3);
            str += ",";
            str += std::to_string(motorZ.force).substr(0,3);
            str += ",";
            str += std::to_string(motorZ.targetForce).substr(0,3);

            rsp.attr_value.len = str.length();
            for(int i = 0; i < str.length(); i++) {
                rsp.attr_value.value[i] = str[i];
            }

        } else {
            rsp.attr_value.len = 3;
            rsp.attr_value.value[0] = 0x00;
            rsp.attr_value.value[1] = 0x11;
            rsp.attr_value.value[2] = 0x22;
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            std::string str((char *)param->write.value);
            std::string cmd = str.substr(0,4);
            std::string value = str.substr(5);
            std::cout << cmd << ", " << value << "\n"; 
            if(cmd == "xhom") {
                motorX.home();
            } else if(cmd == "xpos") {
                motorX.target = std::stof(value);
            } else if(cmd == "xspd") {
                motorX.speed = std::stof(value);
            } else if(cmd == "zhom") {
                motorZ.home();
            } else if(cmd == "zpos") {
                motorZ.target = std::stof(value);
            } else if(cmd == "zspd") {
                motorZ.speed = std::stof(value);
            } else if(cmd == "zfrc") {
                motorZ.targetForce = std::stof(value);
            } else if(cmd == "stop") {
                motorX.target = motorX.position;
                motorZ.target = motorZ.position;
                motorZ.targetForce = 0;
                cycle.cycling = false;
            } else if(cmd == "cycl") {
                // get each comma separated value
                std::vector<std::string> elems = split(str, ',');
                cycle.startX = std::stof(elems[1]);
                cycle.endX = std::stof(elems[2]);
                cycle.startZ = std::stof(elems[3]);
                cycle.force = std::stof(elems[4]);
                cycle.travelSpeed = std::stof(elems[5]);
                cycle.speed = std::stof(elems[6]);
                cycle.cycling = true;
                std::cout << "elems " << elems[0] << ", " << elems[1] 
                    << ", " << elems[2] << ", " << elems[3] << "\n";
            }

        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;
        gl_profile.char_uuid.len = ESP_UUID_LEN_16;
        gl_profile.char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;
        gl_profile.char_uuid2.len = ESP_UUID_LEN_16;
        gl_profile.char_uuid2.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A2;

        esp_ble_gatts_start_service(gl_profile.service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL); //ESP_GATT_AUTO_RSP
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }

        add_char_ret = esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.char_uuid2,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL); //ESP_GATT_AUTO_RSP
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_INCL_SRVC_EVT: {
        break;
    }
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        if(param->add_char.char_uuid.uuid.uuid16 == gl_profile.char_uuid.uuid.uuid16) {
            gl_profile.char_handle = param->add_char.attr_handle;
            gl_profile.descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        } else if(param->add_char.char_uuid.uuid.uuid16 == gl_profile.char_uuid2.uuid.uuid16) {
            gl_profile.char_handle2 = param->add_char.attr_handle;
            gl_profile.descr_uuid2.len = ESP_UUID_LEN_16;
            gl_profile.descr_uuid2.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        }

        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }

        esp_err_t add_descr_ret = ESP_OK;
        if(param->add_char.char_uuid.uuid.uuid16 == gl_profile.char_uuid.uuid.uuid16) {
            add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile.service_handle, &gl_profile.descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        } else if(param->add_char.char_uuid.uuid.uuid16 == gl_profile.char_uuid2.uuid.uuid16) {
            add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile.service_handle, &gl_profile.descr_uuid2,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        }

        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if(param->add_char_descr.descr_uuid.uuid.uuid16 == gl_profile.descr_uuid.uuid.uuid16) {
            gl_profile.descr_handle = param->add_char_descr.attr_handle;
        } else if(param->add_char_descr.descr_uuid.uuid.uuid16 == gl_profile.descr_uuid2.uuid.uuid16) {
            gl_profile.descr_handle2 = param->add_char_descr.attr_handle;
        }
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile.conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile.gatts_if) {
            if (gl_profile.gatts_cb) {
                gl_profile.gatts_cb(event, gatts_if, param);
            }
        }
    } while (0);
}


static bool IRAM_ATTR timer_0_isr(void *para) {
    bool stepX = false;
    bool stepZ = false;
    motorX.intrCount++;
    motorZ.intrCount++;

    //motorX
    if(motorX.intrCount >= motorX.intrInterval) {
        motorX.intrCount = 0;

        if(motorX.bPosError) {
            stepX = true;
        }
    }

    //motorZ
    if(motorZ.intrCount >= motorZ.intrInterval) {
        motorZ.intrCount = 0;

        if(motorZ.bPosMode) { //position mode
            if(motorZ.bPosError) {
                stepZ = true;
            }
        } else { //force mode
            if(motorZ.bForceError) {
                stepZ = true;
            }
        }
    }

    if(stepX) gpio_set_level(motorX.pinStep, 1);
    if(stepZ) gpio_set_level(motorZ.pinStep, 1);
    if(stepX | stepZ) {
        //set timer 1 to reset step pins
        uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_1);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL1_S * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, timer_counter_value);
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
    }

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

    return pdFALSE;
}

static bool IRAM_ATTR timer_1_isr(void *para) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    gpio_set_level(motorX.pinStep, 0);
    gpio_set_level(motorZ.pinStep, 0);
    return pdFALSE;
}

static void tg0_timer_init(timer_idx_t timer_idx,
                                   timer_autoreload_t auto_reload, double timer_interval_ms) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = auto_reload,
        .divider = TIMER_DIVIDER,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_ms * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
}

static void enc_task(void *arg) {
    //set CS pins high to start
    gpio_set_level(pinCS0, 1);
    gpio_set_level(pinCS1, 1);

    //create SPI
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={};
    buscfg.mosi_io_num=pinMOSI;
    // buscfg.data0_io_num=-1;
    buscfg.miso_io_num=pinMISO;
    // buscfg.data1_io_num=-1;
    buscfg.sclk_io_num=pinClk;
    buscfg.quadwp_io_num=-1;
    // buscfg.data2_io_num=-1;
    buscfg.quadhd_io_num=-1;
    // buscfg.data3_io_num=-1;
    // buscfg.data4_io_num=-1;
    // buscfg.data5_io_num=-1;
    // buscfg.data6_io_num=-1;
    // buscfg.data7_io_num=-1;
    // buscfg.max_transfer_sz=0;
    // buscfg.flags=0;
    // buscfg.intr_flags=0;

    spi_device_interface_config_t devcfg={};
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    // devcfg.dummy_bits = 0;
    devcfg.mode=1;
    // devcfg.duty_cycle_pos = 128;
    // devcfg.cs_ena_pretrans = 0;
    // devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz=5000000;
    // devcfg.input_delay_ns = 0;
    devcfg.spics_io_num=-1;   //CS pins manually implemented
    // devcfg.flags = 0;
    devcfg.queue_size = 7;

    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while(1) {
        motorX.update(spi);
        motorZ.update(spi);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void loadCell(void *pvParameters) {
    hx711_t dev = {
        .dout = (gpio_num_t)pinLCDat,
        .pd_sck = (gpio_num_t)pinLCClk,
        .gain = HX711_GAIN_A_64
    };

    // initialize device
    while (1)
    {
        esp_err_t r = hx711_init(&dev);
        if (r == ESP_OK)
            break;
        printf("Could not initialize HX711: %d (%s)\n", r, esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            printf("Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        int32_t data;
        r = hx711_read_data(&dev, &data);
        if (r != ESP_OK)
        {
            printf("Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        forceL = 25*(data-LC1Zero)/LC125lb;
        motorZ.force = forceL+forceR;
        // printf("Raw: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loadCell2(void *pvParameters) {
    hx711_t dev = {
        .dout = (gpio_num_t)pinLCDat2,
        .pd_sck = (gpio_num_t)pinLCClk2,
        .gain = HX711_GAIN_A_64
    };

    // initialize device
    while (1)
    {
        esp_err_t r = hx711_init(&dev);
        if (r == ESP_OK)
            break;
        printf("Could not initialize HX711: %d (%s)\n", r, esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            printf("Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        int32_t data;
        r = hx711_read_data(&dev, &data);
        if (r != ESP_OK)
        {
            printf("Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        forceR =  25*(data-LC2Zero)/LC225lb;
        motorZ.force = forceL+forceR;
        // printf("Raw: %d\n", data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void runCycle() {
    switch(cycle.step) {
        case 0: //move to start x and start z
        motorX.target = cycle.startX;
        motorX.complete = false;
        motorX.speed = cycle.travelSpeed;

        motorZ.targetForce = 0;
        motorZ.target = cycle.startZ;
        motorZ.complete = false;
        motorZ.speed = cycle.travelSpeed;
        cycle.step = 10;
        break;

        case 10: //wait for move complete
        if(motorX.complete && motorZ.complete) cycle.step = 20;
        break;

        case 20: //apply force
        motorZ.targetForce = cycle.force;
        motorZ.bForceError = true;
        cycle.step = 25;
        break;

        case 25:
        if(!motorZ.bForceError) cycle.step = 30;
        break;

        case 30: // move to end x
        motorX.speed = cycle.speed;
        motorX.target = cycle.endX;
        motorX.complete = false;
        cycle.step = 40;
        break;

        case 40: //wait for move complete
        if(motorX.complete) cycle.step = 50;
        break;

        case 50: // set force to 0 and move to start z
        motorZ.speed = cycle.travelSpeed;
        motorZ.targetForce = 0;
        motorZ.target = cycle.startZ;
        motorZ.complete = false;
        cycle.step = 60;
        break;

        case 60: //wait for move complete
        if(motorZ.complete) cycle.step = 0;
        break;
    }
}

void zSpeed() {
    float error = fabs(motorZ.targetForce - motorZ.force);
    float kp = 1.5;
    float speed = kp*error;
    if(speed > 20) speed = 20;
    if(speed < 0.5) speed = 0.5;
    motorZ.speed = speed;
}

extern "C" void app_main(void) {
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    gl_profile.gatts_cb = gatts_profile_a_event_handler;
    gl_profile.gatts_if = ESP_GATT_IF_NONE;
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(0);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    //configure outputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<<pinDirX|1ULL<<pinStepX|1ULL<<pinMSX|1ULL<<pinEn;
    io_conf.pin_bit_mask |= 1ULL<<pinDirZ|1ULL<<pinStepZ|1ULL<<pinMSZ;
    io_conf.pin_bit_mask |= 1ULL<<pinCS0|1ULL<<pinCS1|1ULL<<pinLCClk|1ULL<<pinLCClk2;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

    //configure inputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL<<pinHomX|1ULL<<pinHomZ;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);

    //configure ADCs
    adc1_config_channel_atten(pinCharge, atten);
    adcCharsCharge = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, adcCharsCharge);
    adc2_config_channel_atten(pinEStop, atten);
    adcCharsEStop = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, atten, width, DEFAULT_VREF, adcCharsEStop);

    tg0_timer_init(TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL0_S);
    int *arg = 0;
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_0_isr, arg, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    tg0_timer_init(TIMER_1, TIMER_AUTORELOAD_DIS, TIMER_INTERVAL1_S);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, timer_1_isr, arg, 0);
    timer_start(TIMER_GROUP_0, TIMER_1);

    xTaskCreate(loadCell, "loadCell", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(loadCell2, "loadCell2", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(enc_task, "enc_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    while(1) {
        vTaskDelay(10/portTICK_RATE_MS);
        timeNow = esp_timer_get_time();

        if(motorZ.targetForce != 0) zSpeed();
        if(cycle.cycling) runCycle();
        if(timeNow - timePrint > 1000000) {
            int raw = adc1_get_raw(pinCharge);
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adcCharsCharge);
            battery = (float)voltage*57.0/10000.0;
            //if voltage is too low, deep sleep indefinitely
            if(battery < 9.6) {
                printf("Low battery: %.2f V\n", battery);
                esp_deep_sleep_start();
            }
            adc2_get_raw(pinEStop, width, &raw);
            // raw = adc1_get_raw(pinEStop);
            //Convert adc_reading to voltage in mV
            voltage = esp_adc_cal_raw_to_voltage(raw, adcCharsEStop);
            float eStop = (float)voltage*57.0/10000.0;
            if(eStop < 5) printf("estopped\n");
            // printf("pinHomX: %i\n", gpio_get_level(motorX.pinHome));
            printf("X %.1f\t Z %.1f\t step %i\n", motorX.position, motorZ.position, cycle.step);
            printf("force %.1f\t FL %.1f\t FR %.1f\n", motorZ.force, forceL, forceR);
            timePrint = timeNow;
        }
    }
}