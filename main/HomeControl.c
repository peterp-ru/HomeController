#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_debug_helpers.h"
#include "esp_cpu_utils.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_http_server.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_check.h"
#include "onewire_bus.h"
#include "ds18x20.h"
#include "sys/socket.h"
#include "HomeControlTypes.h"
#include "RingBufferHC.h"
#include "HTTPServerHC.h"

static const char *TAG = "HC_dev";

#define BLINK_GPIO GPIO_NUM_2
#define hc_sayStatusPeriod 2000
#define HC_ReadTemperaturePeriod 7000
#define HC_MaxWIFIReconnectAttempts 10
#define HC_STATUS_BIT_WIFI BIT0
#define HC_STATUS_BIT_HTTP BIT1
#define HC_STATUS_BIT_1820 BIT2
#define HC_STATUS_BIT_OTA  BIT6
#define HC_STATUS_BIT_INIT BIT7
// Define inverted control level
#define HC_CONTROLLEVEL_LO 1
#define HC_CONTROLLEVEL_HI 0
#define GPIO_ONE_WIRE GPIO_NUM_33
#define LOG_BUFFER_SIZE 1024 * 64 - 1
#define CONFIG_RESTART_DEBUG_STACK_DEPTH 28
#define CONFIG_RESTART_DEBUG_LOG_SIZE 4096 - sizeof(time_t) - CONFIG_RESTART_DEBUG_STACK_DEPTH * sizeof(uint32_t)
//#define HC_SHOWLOG true

// Do not exceed 4Kb. 
typedef struct {
    time_t BackTraceTime;
    uint32_t BackTrace[CONFIG_RESTART_DEBUG_STACK_DEPTH];
    char LogTail[CONFIG_RESTART_DEBUG_LOG_SIZE];
  } re_restart_debug_t;

const gpio_num_t OutputGPIO[MAX_DEVICES]={GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_21,GPIO_NUM_22,GPIO_NUM_23,GPIO_NUM_25,GPIO_NUM_26,GPIO_NUM_27};

esp_err_t hc_CurrentError = ESP_OK;
uint8_t hc_Status = 0xC3;
uint8_t NotFoundSensorCount = 0;
uint8_t NumberAttempt = 0;

esp_netif_t *NetworkInterface;
TaskHandle_t TaskSayStatusHandle = NULL;
TaskHandle_t TemperatureReaderHandle = NULL;
TimerHandle_t WifiReconnectTimer = NULL;
onewire_bus_handle_t OWBus;
onewire_bus_config_t OWBus_config = {
    .bus_gpio_num = GPIO_ONE_WIRE,
};
onewire_bus_rmt_config_t OWRmt_config = {
    .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
};

char AccessPoints[4][2][16];
vprintf_like_t LogFuncOriginal = NULL;
ServerDataT HTTPServerData;
ContextDataT HTTPServerContext = {
    .BasePath = "/data",
    .Controls.Count = 0,
    .DSes.Count = 0,
    .NVSConfiguration.ConfigFileName = NULL,
    .NVSConfiguration.KeyConfigFile = "ConfigFileName",
    .NVSConfiguration.NVS_Namespace = "HC_NVSNAME"
};
__NOINIT_ATTR static re_restart_debug_t DebugInfo;

void ClearConfiguredDevices(DSesT *DSDev){
    for (int i = 0; i < DSDev->Count; i++){
        if (DSDev->DSDevices[i]->dsName) free(DSDev->DSDevices[i]->dsName);
        ds18x20_del_device(DSDev->DSDevices[i]->DSDevice);
        free(DSDev->DSDevices[i]);
    } 
    DSDev->Count = 0;

}

esp_err_t SearchOWDevices(onewire_bus_handle_t bus, ContextDataT *Cntx){
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;
    DSesT *stDevices = &Cntx->DSes;
    uint8_t DeviceCount;
    xSemaphoreTake(Cntx->ControlDSData_mux, portMAX_DELAY);
    ClearConfiguredDevices(stDevices);
    DeviceCount =  stDevices->Count;
    ds18x20_device_handle_t DS18x20device;
    // Stable readings from devices DS1820 require a brief period before communication
    // vTaskDelay(hc_sayStatusPeriod * 2 / portTICK_PERIOD_MS);
    // create 1-wire device iterator, which is used for device search
    if (onewire_new_device_iter(bus, &iter) != ESP_OK) {
        ESP_LOGE(TAG, "Fatal error. Cannot create 1-wire device iterator");
        xSemaphoreGive(Cntx->ControlDSData_mux);
        return ESP_ERR_NO_MEM;
    }
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    uint8_t FindAttemptCount = 0;
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18x20_config_t ds_cfg = {};
            if (ds18x20_new_device(&next_onewire_device, &ds_cfg, &DS18x20device) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18x20[%d], address: %016llX", DeviceCount, next_onewire_device.address);
                stDevices->DSDevices[DeviceCount] = calloc(1, sizeof(DSDataT));
                if (!stDevices->DSDevices[DeviceCount]) {
                    ds18x20_del_device(DS18x20device);
                    ESP_LOGE(TAG, "Fatal error. No mem for DS18X20 data. DS18x20[%d], address: %016llX", 
                        DeviceCount, next_onewire_device.address);
                    xSemaphoreGive(Cntx->ControlDSData_mux);
                    return ESP_ERR_NO_MEM;
                }
                stDevices->DSDevices[DeviceCount]->DSDevice = DS18x20device;
                stDevices->DSDevices[DeviceCount]->DSAddress = next_onewire_device.address;
                DeviceCount++;
                if (DeviceCount >= MAX_DEVICES) {
                    ESP_LOGW(TAG, "Max DS18x20 number reached, stop searching...");
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
        FindAttemptCount++;
    } while (search_result != ESP_ERR_NOT_FOUND || FindAttemptCount <= MAX_DEVICES + 1);
    stDevices->Count = DeviceCount;
    if (onewire_del_device_iter(iter) != ESP_OK) ESP_LOGW(TAG, "Error while onewire_del_device_iter(iter) call");
    ESP_LOGI(TAG, "Searching done, %d DS18x20 device(s) found", DeviceCount);
    // Set resolution for all DS1820s
    for (int i = 0; i < DeviceCount; i++) {
        if(ds18x20_set_resolution(stDevices->DSDevices[i]->DSDevice, DS18B20_RESOLUTION_9B) != ESP_OK) {
            ESP_LOGW(TAG, "Set resolution error. DS18x20[%d], address: %016llX", i, stDevices->DSDevices[i]->DSAddress);
        }
    }
    xSemaphoreGive(Cntx->ControlDSData_mux);
    return ESP_OK;

}

gpio_num_t FindPin(int PinNumber){
    for (int i=0; i<MAX_DEVICES; i++) {
        if (PinNumber == (int)OutputGPIO[i]) return OutputGPIO[i];
    }
    return GPIO_NUM_NC;

}

DSDataHandleT FindThermometr( DSesT *DSDev, unsigned long long DSAddress, char *coName){
    for (int i=0; i<DSDev->Count; i++) {
        if (DSAddress == DSDev->DSDevices[i]->DSAddress) {
            if (DSDev->DSDevices[i]->dsName == NULL) {
                DSDev->DSDevices[i]->dsName = strdup(coName);
            }
            return DSDev->DSDevices[i];
        }
    }
    return NULL;

}

void ClearConfiguredControls(ControlsT *ConfCntrl){
    for (int i = 0; i < ConfCntrl->Count; i++){
        gpio_set_level(ConfCntrl->ControlOutputData[i]->gpioPin, HC_CONTROLLEVEL_LO);
        if (ConfCntrl->ControlOutputData[i]->coName) free(ConfCntrl->ControlOutputData[i]->coName);
        free(ConfCntrl->ControlOutputData[i]);
    } 
    ConfCntrl->Count = 0;
    NotFoundSensorCount = 0;

}

esp_err_t LoadControlConfig(char *FileName, ContextDataT *DataCntx){
    const int MaxStringLength = 180;
    const char Delimiters[3]=" \t";
    const int MaxColumn = 9;
    char BufferStr[MaxStringLength];
    char *SingleWord;
    int i;
    int LineCount = 0;
    int mHour;
    int mMinute;
    ControlOutputDataT LineData;
    if (DataCntx->DSes.Count > 0)
        ESP_LOGI(TAG, "Clearing & new search for sensors. Current count of sensors: %d", DataCntx->DSes.Count);
    if(SearchOWDevices(OWBus, DataCntx) != ESP_OK){
        ESP_LOGW(TAG, "Error searching sensors.");
    }
    ESP_LOGI(TAG, "Loading configuration from file %s. Current controls: %d", FileName, DataCntx->Controls.Count);
    xSemaphoreTake(DataCntx->ControlDSData_mux, portMAX_DELAY);
    ClearConfiguredControls(&DataCntx->Controls);
    FILE *ConfigFile = fopen(FileName, "r");
    if (ConfigFile == NULL) {
        ESP_LOGI(TAG, "Cannot open file %s", FileName);
        xSemaphoreGive(DataCntx->ControlDSData_mux);
        return ESP_ERR_NOT_FOUND;
    }
    while (fgets(BufferStr, MaxStringLength, ConfigFile)){
        LineCount++;
        if (strlen(BufferStr) >= MaxStringLength - 1) {
            ESP_LOGW(TAG, "Line %d too long. Ignored.", LineCount);
            continue;
        }
        if (LineCount > 20){
            ESP_LOGW(TAG, "Line limit (20) reached. Load aborted.");
            break;
        }
        SingleWord = strchr(BufferStr, '#');
        if(SingleWord != NULL) SingleWord[0] = '\0';
        SingleWord = strtok(BufferStr, Delimiters);
        for (i=0; i < MaxColumn; i++) {
            if (SingleWord == NULL) break;
            switch (i) {
                case 0:
                    LineData.coName = strdup(SingleWord);
                    break;
                case 1:
                    LineData.gpioPin = FindPin(atoi(SingleWord)); 
                    break;
                case 2:
                    LineData.Thermometr = FindThermometr(&DataCntx->DSes, strtoull(SingleWord, NULL, 16), LineData.coName);
                    break;
                case 3:
                    sscanf(SingleWord, "%2d:%2d", &mHour, &mMinute);
                    LineData.DayStartTimeInSec = mHour * 3600 + mMinute * 60;
                    break;
                case 4:
                    sscanf(SingleWord, "%2d:%2d", &mHour, &mMinute);
                    LineData.DayStopTimeInSec = mHour * 3600 + mMinute * 60;
                    break;
                case 5:
                    LineData.DayMinTemp = strtof(SingleWord, NULL);
                    break;
                case 6:
                    LineData.DayMaxTemp = strtof(SingleWord, NULL);
                    break;
                case 7:
                    LineData.NigthMinTemp = strtof(SingleWord, NULL);
                    break;
                case 8:
                    LineData.NigthMaxTemp = strtof(SingleWord, NULL);
                    break;
            }
            SingleWord = strtok(NULL, Delimiters);
        }
        // skip empty lines
        if (i == 0) continue;
        // Check for error 
        if (i != MaxColumn
            || LineData.gpioPin == GPIO_NUM_NC
            || LineData.gpioPin == GPIO_NUM_0
            || LineData.DayStartTimeInSec >= LineData.DayStopTimeInSec
            || LineData.DayMinTemp == LineData.DayMaxTemp
            || LineData.NigthMinTemp == LineData.NigthMaxTemp){
            ESP_LOGI(TAG, "Data error in line %d column %d. Ignored.", LineCount, i);
        } else if (LineData.Thermometr == NULL) {
            NotFoundSensorCount++;
            ESP_LOGI(TAG, "Sensor not found in line %d. Ignored.", LineCount);
        } else {
            if (DataCntx->Controls.Count < MAX_DEVICES) {
                LineData.CurrentCondition = HC_CONTROLLEVEL_LO; // on startup always LO
                DataCntx->Controls.ControlOutputData[DataCntx->Controls.Count] = malloc(sizeof(ControlOutputDataT));
                if (DataCntx->Controls.ControlOutputData[DataCntx->Controls.Count]) {
                    memcpy(DataCntx->Controls.ControlOutputData[DataCntx->Controls.Count], &LineData, sizeof(ControlOutputDataT));
                    DataCntx->Controls.Count++;
                    ESP_LOGI(TAG, "Added control %d in condition LO", LineData.gpioPin);
                } else {
                    fclose(ConfigFile);
                    xSemaphoreGive(DataCntx->ControlDSData_mux);
                    ESP_RETURN_ON_FALSE(false, ESP_ERR_NO_MEM, TAG, "no mem for ControlOutputData data");
                }
            } else {
                fclose(ConfigFile);
                xSemaphoreGive(DataCntx->ControlDSData_mux);
                ESP_RETURN_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, TAG, "Maximum number of controls reached. Abort reading.");
            }
        }
    }
    fclose(ConfigFile);
    xSemaphoreGive(DataCntx->ControlDSData_mux);
    ESP_LOGI(TAG, "Configured %d controls. Not found sensor count: %d.", DataCntx->Controls.Count, NotFoundSensorCount);
    return ESP_OK;

}

bool SwitchControlOutput(ControlOutputDataHandleT ControlData, time_t CurrenTimeInSec, float CurrentTemp, bool TempError){
    int8_t Action = -1;
    // Switching
    if (CurrenTimeInSec >= ControlData->DayStartTimeInSec && CurrenTimeInSec < ControlData->DayStopTimeInSec){ 
        if (!TempError && CurrentTemp < ControlData->DayMinTemp && ControlData->CurrentCondition == HC_CONTROLLEVEL_LO) 
            Action = HC_CONTROLLEVEL_HI;
        else if (!TempError && CurrentTemp >= ControlData->DayMaxTemp && ControlData->CurrentCondition == HC_CONTROLLEVEL_HI)
            Action = HC_CONTROLLEVEL_LO;
    } else {
        if (!TempError && CurrentTemp < ControlData->NigthMinTemp && ControlData->CurrentCondition == HC_CONTROLLEVEL_LO)
            Action = HC_CONTROLLEVEL_HI;
        else if (!TempError && CurrentTemp >= ControlData->NigthMaxTemp && ControlData->CurrentCondition == HC_CONTROLLEVEL_HI)
            Action = HC_CONTROLLEVEL_LO;
    }
    if (Action != -1) {
        gpio_set_level(ControlData->gpioPin, Action);
        ESP_LOGI(TAG, "Control %d switched from %s to %s", ControlData->gpioPin, 
            ControlData->CurrentCondition == HC_CONTROLLEVEL_HI ? "HI" : "LO", 
            Action == HC_CONTROLLEVEL_HI ? "HI" : "LO");
        ControlData->CurrentCondition = Action;
        return true;
    } else return false;

}

void SwitchAllControl(ControlsT *ConfCntrl){
    time_t now;
    struct tm* tmCurrentTime;
    float CurrentTemp = 0;
    bool TempInError;
    time(&now);
    tmCurrentTime = localtime(&now);
    // Convert to time only
    time_t CurrenTimeInSec = tmCurrentTime->tm_hour * 3600 + tmCurrentTime->tm_min * 60 + tmCurrentTime->tm_sec;
    for (uint8_t i=0; i < ConfCntrl->Count; i++){
        // take current temp
        if (ConfCntrl->ControlOutputData[i]->Thermometr != NULL) {
            CurrentTemp = ConfCntrl->ControlOutputData[i]->Thermometr->LastValue;
            TempInError = ConfCntrl->ControlOutputData[i]->Thermometr->ReadError;
        } else TempInError = true;
        if (SwitchControlOutput(ConfCntrl->ControlOutputData[i], CurrenTimeInSec, CurrentTemp, TempInError)) break;
    }

}

void ReadTemperatures(void *Param) {
    bool GotError;
    float temperature;
    bool NeedReload;
    time_t now;
    ContextDataT *mContext = (ContextDataT*)Param;
    DSDataHandleT *aDS = mContext->DSes.DSDevices;
    char *ConfigFileName = mContext->NVSConfiguration.ConfigFileName;
    while (true) {
        if (xSemaphoreTake(mContext->ControlDSData_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS)) {
            NeedReload = false;
            for (int i = 0; i < mContext->DSes.Count; i ++) {
                GotError = true;
                NeedReload = NeedReload || aDS[i]->NumberOfErrors == 255;
                if (ds18x20_trigger_temperature_conversion(aDS[i]->DSDevice) == ESP_OK){
                    if (ds18x20_get_temperature(aDS[i]->DSDevice, &temperature) == ESP_OK){
                        time(&now);
                        aDS[i]->LastValue = temperature;
                        aDS[i]->LastRead = now;
                        aDS[i]->ReadError = false;
                        GotError = false;
                        ESP_LOGD(TAG, "read from DSDevices[%d]: %.2fC", i, temperature);
                    } else {
                        ESP_LOGW(TAG, "ds18x20_get_temperature error on DSDevices[%d]", i);
                    }
                } else {
                    ESP_LOGW(TAG, "ds18x20_trigger_temperature_conversion error on DSDevices[%d]", i);
                }
                if (GotError) {
                    if (aDS[i]->NumberOfErrors < 255) aDS[i]->NumberOfErrors++;
                    aDS[i]->ReadError = true;
                }
            }
            // start switching controls
            SwitchAllControl(&mContext->Controls);
            xSemaphoreGive(mContext->ControlDSData_mux);
            if (NeedReload && ConfigFileName != NULL) LoadControlConfig(ConfigFileName, mContext);
        } else {
            ESP_LOGW(TAG, "Temperature not read! Semaphore timed out.");
        }
        vTaskDelay(HC_ReadTemperaturePeriod / portTICK_PERIOD_MS);
    }

}

void time_sync_notification_cb(struct timeval *tv){
    char strftime_buf[21];
    struct tm timeinfo;
    if (HTTPServerContext.SystemInfo.StartupTime == 0){
        time(&HTTPServerContext.SystemInfo.StartupTime);
        HTTPServerContext.SystemInfo.StartupTime = HTTPServerContext.SystemInfo.StartupTime - (int)(esp_log_timestamp()/1000);
    }
    localtime_r(&tv->tv_sec, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%d.%m.%Y %T ", &timeinfo);
    ESP_LOGI(TAG, "Time synchronization notification: %s", strftime_buf);

}

void initialize_sntp(void){
    setenv("TZ","MSK-3",1);
    tzset();
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

bool SelectWiFiAP(wifi_config_t *WiFiConfig){
    wifi_ap_record_t *ArrayAPInfo;
    uint16_t ap_count = 0;
    int8_t SelectedAP = -1;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    if (ap_count > 0) {
        ArrayAPInfo = malloc(ap_count * sizeof(wifi_ap_record_t));
        ESP_RETURN_ON_FALSE(ArrayAPInfo != NULL, false, TAG, "no mem for ArrayAPInfo data");                    
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ArrayAPInfo));
        for (int i = 0; i < ap_count; i++) {
            for (int y=0; (y < 3) && (SelectedAP == -1); y++){
                if (strcmp((char*)ArrayAPInfo[i].ssid, AccessPoints[y][0]) == 0) {
                    SelectedAP = y;
                }
            }
            ESP_LOGD(TAG, "SSID \t\t%s", ArrayAPInfo[i].ssid);
            ESP_LOGD(TAG, "RSSI \t\t%d", ArrayAPInfo[i].rssi);
        }
        free(ArrayAPInfo);
    }
    if (SelectedAP == -1){
        ESP_LOGW(TAG, "Cannot find famous access points. Total APs scanned: %u", ap_count);
        return false;
    } else {
        ESP_LOGI(TAG, "Found AP SSID %s", AccessPoints[SelectedAP][0]);
        bzero(WiFiConfig, sizeof(wifi_config_t));
        memcpy(WiFiConfig->sta.ssid, AccessPoints[SelectedAP][0], strlen(AccessPoints[SelectedAP][0]));
        memcpy(WiFiConfig->sta.password, AccessPoints[SelectedAP][1], strlen(AccessPoints[SelectedAP][1]));
        WiFiConfig->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    return true;
    }

}

static void vTimerCallback(TimerHandle_t tmr){
    if (NumberAttempt == HC_MaxWIFIReconnectAttempts) {
        esp_wifi_disconnect();
        wifi_config_t WiFiStaCfg;
        bzero(&WiFiStaCfg, sizeof(wifi_config_t));
        WiFiStaCfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        WiFiStaCfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        esp_wifi_set_config(WIFI_IF_STA, &WiFiStaCfg);
        esp_wifi_scan_start(NULL, false);
    } else 
        esp_wifi_connect();

}

static void ConfirmOtaUpdate(void){
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY || ota_state == ESP_OTA_IMG_VALID
            || ota_state == ESP_OTA_IMG_UNDEFINED) {
            hc_Status &= ~HC_STATUS_BIT_OTA;
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY){
                ESP_LOGI(TAG, "Confirm OTA update! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
        }
    }

}

void ReconnectingProcess(void){
    hc_Status |= HC_STATUS_BIT_WIFI;
    // reconnecting process
    unsigned int TimerPeriod = 1 << NumberAttempt;
    if (NumberAttempt < HC_MaxWIFIReconnectAttempts) 
        NumberAttempt ++; 
    if (!(hc_Status & HC_STATUS_BIT_HTTP) && NumberAttempt == HC_MaxWIFIReconnectAttempts - 1) {
        // must stop web server and other socket applications. If they are running
        ESP_ERROR_CHECK(Stop_HTTP_Server(&HTTPServerData));
        hc_Status |= HC_STATUS_BIT_HTTP;
        esp_sntp_stop();
    }
    if(WifiReconnectTimer == NULL){
        WifiReconnectTimer = xTimerCreate("wifi_tmr", TimerPeriod * 1000 / portTICK_PERIOD_MS, pdFALSE, NULL, vTimerCallback);
        if(WifiReconnectTimer != NULL) xTimerStart(WifiReconnectTimer, 0);
    }else{
        if(xTimerIsTimerActive(WifiReconnectTimer) == pdFALSE){
            xTimerChangePeriod(WifiReconnectTimer, TimerPeriod * 1000 / portTICK_PERIOD_MS, 0); 
        }
    }
    ESP_LOGI(TAG, "Scheduled reconnect at %d second", TimerPeriod);

}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    wifi_config_t WiFiStaConfig;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &WiFiStaConfig));
        if (WiFiStaConfig.sta.ssid[0] == 0) {
            ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, false));
        } else {
            ESP_LOGI(TAG, "Stored SSID: %s, connecting...", WiFiStaConfig.sta.ssid);
            ESP_ERROR_CHECK(esp_wifi_connect());
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        if (SelectWiFiAP(&WiFiStaConfig)) {
            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &WiFiStaConfig));
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else {
            NumberAttempt = HC_MaxWIFIReconnectAttempts;
            ReconnectingProcess();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ReconnectingProcess();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        hc_Status &= ~HC_STATUS_BIT_WIFI;
        NumberAttempt = 0;
        // check if address changed
        if (event->ip_changed && !(hc_Status & HC_STATUS_BIT_HTTP)) {
            ESP_ERROR_CHECK(Stop_HTTP_Server(&HTTPServerData));
            hc_Status |= HC_STATUS_BIT_HTTP;
            esp_sntp_stop();
        }
        // start web server and other socket applications 
        if (hc_Status & HC_STATUS_BIT_HTTP) {
            ESP_ERROR_CHECK(Start_HTTP_Server(&HTTPServerData));
            hc_Status &= ~HC_STATUS_BIT_HTTP;
            initialize_sntp();
        }
        // check ota update
        if (hc_Status & HC_STATUS_BIT_OTA) ConfirmOtaUpdate();
    } else if (event_base == ESP_HTTP_SERVER_EVENT && event_id == HTTP_SERVER_EVENT_ON_CONNECTED ){
        int *SocketDescriptor = (int*) event_data;
        //char IpStr6[INET6_ADDRSTRLEN];
        char IpStr4[INET_ADDRSTRLEN];
        struct sockaddr_in6 Address6;
        socklen_t SizeAddress6 = sizeof(Address6);
        if (getpeername(*SocketDescriptor, (struct sockaddr *)&Address6, &SizeAddress6) < 0){
            ESP_LOGE(TAG, "Error getting client IP in new session (HTTP_SERVER_EVENT_ON_CONNECTED)");
        } else {
            //inet_ntop(AF_INET6, &Address6.sin6_addr, IpStr6, sizeof(IpStr6));
            inet_ntop(AF_INET, &Address6.sin6_addr.un.u32_addr[3], IpStr4, sizeof(IpStr4));
            //ESP_LOGI(TAG, "Got connect from host: %s (%s)", IpStr6, IpStr4);
            ESP_LOGI(TAG, "New http session from: %s", IpStr4);
        }
    }
}

esp_err_t mount_storage(const char* base_path){
    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = NULL,
        .max_files = 3,   // This sets the maximum number of files that can be open at the same time
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Partition '%s' registered, Size: total: %d, used: %d", base_path, total, used);
    return ESP_OK;

}

void sayStatus(void *VoidParam){   
    uint8_t BlinkCounter;
    int16_t BlinkPeriod;
    while (true){
        if(hc_Status & HC_STATUS_BIT_INIT) {
            BlinkCounter=10;
            BlinkPeriod=100;
        }else{
            BlinkPeriod=200;
            if(hc_Status & HC_STATUS_BIT_HTTP) BlinkCounter=2;
            else if(hc_Status & HC_STATUS_BIT_WIFI) BlinkCounter=1;
            else{
                BlinkCounter=1;
                BlinkPeriod=2000;
            }
        }
        for (uint8_t i = 0; i < BlinkCounter; i++){
            for (uint8_t u=0; u<2; u++){
                gpio_set_level(BLINK_GPIO, !u);
                vTaskDelay(BlinkPeriod / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(hc_sayStatusPeriod / portTICK_PERIOD_MS);
    }
}

int LogToRingBuffer(const char *fmt, va_list args){
    const size_t MaxMsgLen = 254;
    static char BufferString[254];
    static char MessageString[254];
    static _lock_t bufferLock = 0;
    char MonthAndDay[21];
    char *OpenBracket;
    int LenMessage;
    struct timeval tv;
    struct tm timeinfo;
    _lock_acquire(&bufferLock);
    LenMessage = vsnprintf(BufferString, MaxMsgLen - 1, fmt, args);
    if (LenMessage>=0) {
        OpenBracket = strchr(BufferString, '(');
        if (OpenBracket) {
            strncpy(MessageString, BufferString, OpenBracket - BufferString + 1);
            MessageString[OpenBracket - BufferString + 1] = '\0';
            gettimeofday(&tv, NULL);
            localtime_r(&tv.tv_sec, &timeinfo);
            strftime(MonthAndDay, sizeof(MonthAndDay), "%F %T ", &timeinfo);
            strcat(MessageString, MonthAndDay);
            strcat(MessageString + strlen(MonthAndDay), OpenBracket + 1);
            RingBufferPut(HTTPServerContext.LogBuffer, MessageString, strlen(MessageString));
        } else {
            RingBufferPut(HTTPServerContext.LogBuffer, BufferString, LenMessage);
        }
    }
#ifdef HC_SHOWLOG
    printf("%s", BufferString);
#endif
    _lock_release(&bufferLock);
    return LenMessage;
    
}

esp_err_t SaveLogToFile(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    char FilePath[25];
    struct stat FileStat;
    FILE *FileDescriptor = NULL;
    //Check free space
    // FilePath = "/data/202404211229.log"
    time(&now);
    localtime_r(&now, &timeinfo);
    strcpy(FilePath, HTTPServerContext.BasePath);
    int BasePathLen = strlen(HTTPServerContext.BasePath);
    strftime(FilePath + BasePathLen, 25 - BasePathLen, "/%Y%m%d%H%M.log", &timeinfo);
    if (stat(FilePath, &FileStat) == 0) {
        ESP_LOGE(TAG, "File %s already exists", FilePath);
        return ESP_FAIL;
    }
    FileDescriptor = fopen(FilePath, "w");
    if (!FileDescriptor) {
        ESP_LOGE(TAG, "Failed to create file %s", FilePath);
        return ESP_FAIL;
    }
    int remaining = HTTPServerContext.LogBuffer->mSize - HTTPServerContext.LogBuffer->mFree;
    size_t ScratchBufferSize = (remaining < (int)(esp_get_free_heap_size() / 2) ? remaining : (int)(esp_get_free_heap_size() / 2));
    char *Buf = malloc(ScratchBufferSize);
    if (!Buf) {
        // No mem for receive file
        ESP_LOGE(TAG, "No mem for receive file");
        fclose(FileDescriptor);
        unlink(FilePath);
        return ESP_FAIL;
    }
    int Getted = RingBufferRead(HTTPServerContext.LogBuffer, Buf, ScratchBufferSize, true);
    while (Getted >0) {
        if (Getted != fwrite(Buf, 1, Getted, FileDescriptor)) {
            ESP_LOGE(TAG, "Couldn't write everything to file! Storage may be full?");
            free(Buf);
            fclose(FileDescriptor);
            unlink(FilePath);
            return ESP_FAIL;
        }
        Getted = RingBufferRead(HTTPServerContext.LogBuffer, Buf, ScratchBufferSize, false);
    }
    if (Getted < 0) {
        ESP_LOGE(TAG, "Semaphore timed out while RingBufferRead");
        free(Buf);
        fclose(FileDescriptor);
        unlink(FilePath);
        return ESP_FAIL;
    }
    free(Buf);
    fclose(FileDescriptor);
    return ESP_OK;

}

void IRAM_ATTR DebugBacktraceUpdate(){
    esp_backtrace_frame_t stk_frame;
    esp_backtrace_get_start(&(stk_frame.pc), &(stk_frame.sp), &(stk_frame.next_pc)); 
    DebugInfo.BackTrace[0] = esp_cpu_process_stack_pc(stk_frame.pc);
    bool corrupted = (esp_stack_ptr_is_sane(stk_frame.sp) &&
                    esp_ptr_executable((void*)esp_cpu_process_stack_pc(stk_frame.pc))) ? false : true; 
    uint8_t i = CONFIG_RESTART_DEBUG_STACK_DEPTH;
    while (i-- > 0 && stk_frame.next_pc != 0 && !corrupted) {
        if (!esp_backtrace_get_next_frame(&stk_frame)) {
            corrupted = true;
        }
        DebugInfo.BackTrace[CONFIG_RESTART_DEBUG_STACK_DEPTH - i] = esp_cpu_process_stack_pc(stk_frame.pc);
    }
    time(&DebugInfo.BackTraceTime);
    // need fill DebugInfo.LogTail with size CONFIG_RESTART_DEBUG_LOG_SIZE
    size_t ReadPoint;
    size_t Tail = HTTPServerContext.LogBuffer->mTail;
    size_t ToRead = HTTPServerContext.LogBuffer->mSize - HTTPServerContext.LogBuffer->mFree;
    if (ToRead > CONFIG_RESTART_DEBUG_LOG_SIZE) ToRead = CONFIG_RESTART_DEBUG_LOG_SIZE;
    if (ToRead == 0) return;
    if (Tail > ToRead) {
        ReadPoint = Tail - ToRead;
        memcpy(DebugInfo.LogTail, HTTPServerContext.LogBuffer->mBuffer + ReadPoint, ToRead);
    } else {
        size_t PartOneSize = ToRead - Tail;
        ReadPoint = HTTPServerContext.LogBuffer->mSize - PartOneSize;
        memcpy(DebugInfo.LogTail, HTTPServerContext.LogBuffer->mBuffer + ReadPoint, PartOneSize);
        memcpy(DebugInfo.LogTail + PartOneSize, HTTPServerContext.LogBuffer->mBuffer, Tail);
    }

}

/**
 * Declare the symbol pointing to the former implementation of esp_panic_handler function
 */
extern void __real_esp_panic_handler(void *info);

/**
 * Redefine esp_panic_handler function to save a log buffer before actually restarting
 */
void __wrap_esp_panic_handler(void *info){
    DebugBacktraceUpdate();
    __real_esp_panic_handler(info);

}

static void uninitializeCtrl(void){
    if (WifiReconnectTimer != NULL) xTimerDelete(WifiReconnectTimer, 0);
    if (TemperatureReaderHandle != NULL) vTaskDelete(TemperatureReaderHandle);
    hc_CurrentError = Stop_HTTP_Server(&HTTPServerData);
    hc_Status |= HC_STATUS_BIT_HTTP;
    esp_sntp_stop();
    ClearConfiguredControls(&HTTPServerContext.Controls);
    ClearConfiguredDevices(&HTTPServerContext.DSes);
    if (HTTPServerContext.NVSConfiguration.ConfigFileName != NULL) {
        free(HTTPServerContext.NVSConfiguration.ConfigFileName);
        HTTPServerContext.NVSConfiguration.ConfigFileName = NULL;
    }
    esp_netif_destroy_default_wifi(NetworkInterface);
    hc_Status |= HC_STATUS_BIT_WIFI;
    onewire_bus_del(OWBus);
    OWBus = NULL;
    esp_vfs_spiffs_unregister(NULL);
    esp_event_loop_delete_default();
    nvs_flash_deinit();
    if (TaskSayStatusHandle != NULL) vTaskDelete(TaskSayStatusHandle);
    TaskSayStatusHandle = NULL;
    DebugBacktraceUpdate();
    RingBufferDestroy(HTTPServerContext.LogBuffer);
    free(HTTPServerContext.LogBuffer);
    HTTPServerContext.LogBuffer = NULL;
}

void app_main(void){
    int i;
#ifdef HC_SHOWLOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    // esp_log_level_set(TAG, ESP_LOG_INFO);
#else
    esp_log_level_set(TAG, ESP_LOG_INFO);
#endif
    // esp_log_level_set("wifi_init", ESP_LOG_INFO);
    // esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("HC_HTTPServerHC", ESP_LOG_INFO);
    HTTPServerContext.LogBuffer = malloc(sizeof(HCRingbufferT));
    RingBufferCreate(HTTPServerContext.LogBuffer, LOG_BUFFER_SIZE); // initial size 64K
    // copying crash dump info into log buffer
    esp_reset_reason_t LastReset = esp_reset_reason();
    if (LastReset != ESP_RST_POWERON){
        char tmpBuf[140];
        struct tm timeinfo = { 0 };
        localtime_r(&DebugInfo.BackTraceTime, &timeinfo);
        strftime(tmpBuf, 140, "Start Debug info from %d.%m.%Y %X \nBacktrace:\n", &timeinfo);
        RingBufferPut(HTTPServerContext.LogBuffer, tmpBuf, strlen(tmpBuf));
        i = 0;
        char *BufferPoint;
        while (i < CONFIG_RESTART_DEBUG_STACK_DEPTH) {
            BufferPoint = tmpBuf;
            for (int j = 0; j < 7 && i < CONFIG_RESTART_DEBUG_STACK_DEPTH; j++) {
                BufferPoint += sprintf(BufferPoint, "0x%08lX ", DebugInfo.BackTrace[i]);
                i++;
            }
            *BufferPoint++ = 13;
            *BufferPoint++ = 0;
            RingBufferPut(HTTPServerContext.LogBuffer, tmpBuf, strlen(tmpBuf));
        }
        sprintf(tmpBuf, "\nTail of the log:\n");
        RingBufferPut(HTTPServerContext.LogBuffer, tmpBuf, strlen(tmpBuf));
        RingBufferPut(HTTPServerContext.LogBuffer, DebugInfo.LogTail, 
            strlen(DebugInfo.LogTail) < CONFIG_RESTART_DEBUG_LOG_SIZE ? strlen(DebugInfo.LogTail) : CONFIG_RESTART_DEBUG_LOG_SIZE);
        sprintf(tmpBuf, "\nEnd of Debug info\n");
        RingBufferPut(HTTPServerContext.LogBuffer, tmpBuf, strlen(tmpBuf));
    } else {
        memset(&DebugInfo, 0, sizeof(re_restart_debug_t));
    }
    // Init secret data
    extern const unsigned char Secret_bin_start[] asm("_binary_Secret_bin_start");
    extern const unsigned char Secret_bin_end[]   asm("_binary_Secret_bin_end");
    const size_t Secret_bin_size = (Secret_bin_end - Secret_bin_start);
    memcpy(AccessPoints, Secret_bin_start, Secret_bin_size);
    // Initialize main structure
    esp_chip_info_t ChipInfo;
    // HTTPServerContext.BasePath alreay initialized
    // HTTPServerContext.Controls.Count alreay initialized
    HTTPServerContext.ControlDSData_mux = xSemaphoreCreateMutex();
    // HTTPServerContext.DSes.Count alreay initialized
    // HTTPServerContext.LogBuffer alreay initialized
    // HTTPServerContext.NVSConfiguration alreay initialized
    esp_chip_info(&ChipInfo);
    const char * tmpStr;
    if (ChipInfo.model == CHIP_ESP32) tmpStr="ESP32";
    else if (ChipInfo.model == CHIP_ESP32S2) tmpStr="ESP32-S2";
    else if (ChipInfo.model == CHIP_ESP32S3) tmpStr="ESP32-S3";
    else if (ChipInfo.model == CHIP_ESP32C3) tmpStr="ESP32-C3";
    else if (ChipInfo.model == CHIP_ESP32C2) tmpStr="ESP32-C2";
    else if (ChipInfo.model == CHIP_ESP32C6) tmpStr="ESP32-C6";
    else if (ChipInfo.model == CHIP_ESP32H2) tmpStr="ESP32-H2";
    else tmpStr="POSIX/Linux simulator";
    HTTPServerContext.SystemInfo.ChipCores = ChipInfo.cores;
    memcpy(HTTPServerContext.SystemInfo.ChipModel, tmpStr, strlen(tmpStr)+1);
    HTTPServerContext.SystemInfo.ChipRevision = ChipInfo.revision;
    // HTTPServerContext.SystemInfo.HostName see later
    const esp_app_desc_t *ApplicationDescr = esp_app_get_description();
    memcpy(HTTPServerContext.SystemInfo.IdfVersion, ApplicationDescr->idf_ver, sizeof(HTTPServerContext.SystemInfo.IdfVersion));
    // HTTPServerContext.SystemInfo.InvalidVersion see later
    // HTTPServerContext.SystemInfo.MacAddress see later
    // HTTPServerContext.SystemInfo.PartConfAddr see later
    // HTTPServerContext.SystemInfo.PartRunnAddr see later
    // HTTPServerContext.SystemInfo.ProjectName see later
    if (LastReset == ESP_RST_SDIO) tmpStr="Reset over SDIO";
    else if (LastReset == ESP_RST_BROWNOUT) tmpStr="Brownout reset";
    else if (LastReset == ESP_RST_DEEPSLEEP) tmpStr="Exiting deep sleep mode";
    else if (LastReset == ESP_RST_WDT) tmpStr="Other watchdogs";
    else if (LastReset == ESP_RST_TASK_WDT) tmpStr="Task watchdog";
    else if (LastReset == ESP_RST_INT_WDT) tmpStr="Interrupt watchdog";
    else if (LastReset == ESP_RST_PANIC) tmpStr="Exception/panic";
    else if (LastReset == ESP_RST_SW) tmpStr="Software restart";
    else if (LastReset == ESP_RST_EXT) tmpStr="External pin";
    else if (LastReset == ESP_RST_POWERON) tmpStr="Power-on";
    else tmpStr="Unknown";
    memcpy(HTTPServerContext.SystemInfo.ResetReason, tmpStr, strlen(tmpStr)+1);
    // HTTPServerContext.SystemInfo.RunningVersion see later
    HTTPServerContext.SystemInfo.StartupTime = 0;
    // finished filling SystemInfo & HTTPServerContext
    HTTPServerData.ServerHandle = NULL;
    HTTPServerData.ContextServerData = &HTTPServerContext;
    LogFuncOriginal = esp_log_set_vprintf(LogToRingBuffer);
    // Configure pin led
    gpio_reset_pin(BLINK_GPIO);
    hc_CurrentError=gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    // Start blinker
    xTaskCreate(sayStatus, "sayStatus", 1024, NULL, tskIDLE_PRIORITY, &TaskSayStatusHandle);
    // Configure control load pins and set to Off
    for (i = 0; i < MAX_DEVICES; i++) {
        gpio_reset_pin(OutputGPIO[i]);
        gpio_set_direction(OutputGPIO[i], GPIO_MODE_OUTPUT);
        gpio_set_level(OutputGPIO[i], HC_CONTROLLEVEL_LO);
    } 
    // Initialize default NVS psrtition
    hc_CurrentError = nvs_flash_init();
    if (hc_CurrentError == ESP_ERR_NVS_NO_FREE_PAGES || hc_CurrentError == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        hc_CurrentError = nvs_flash_init();
    }
    ESP_ERROR_CHECK(hc_CurrentError);
    // Initialize esp event subsystem
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Initialize file storage 
    ESP_ERROR_CHECK(mount_storage(HTTPServerContext.BasePath));
    //Initialize tcp/ip stack
    ESP_ERROR_CHECK(esp_netif_init());
    // Read partition information
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
    esp_app_desc_t running_app_info;
    esp_app_desc_t invalid_app_info;
    HTTPServerContext.SystemInfo.PartRunnAddr = running->address;
    HTTPServerContext.SystemInfo.PartConfAddr = configured->address;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        memcpy(HTTPServerContext.SystemInfo.RunningVersion, running_app_info.version, sizeof(running_app_info.version));
        memcpy(HTTPServerContext.SystemInfo.ProjectName, running_app_info.project_name, sizeof(running_app_info.project_name));
    }
    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
        memcpy(HTTPServerContext.SystemInfo.InvalidVersion, invalid_app_info.version, sizeof(invalid_app_info.version));
    }
    // Initialize one wire bus
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&OWBus_config, &OWRmt_config, &OWBus));
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", GPIO_ONE_WIRE);
    //Initialize WiFi station
    NetworkInterface = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ESP_HTTP_SERVER_EVENT, HTTP_SERVER_EVENT_ON_CONNECTED, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Read MAC address from wifi interface
    esp_wifi_get_mac(WIFI_IF_STA, HTTPServerContext.SystemInfo.MacAddress);
    HTTPServerContext.SystemInfo.HostName=CONFIG_LWIP_LOCAL_HOSTNAME;
    // Get config file name and load it
    nvs_handle_t HCNVSHandle;
    size_t StringSize = 0;
    hc_CurrentError = nvs_open(HTTPServerContext.NVSConfiguration.NVS_Namespace, NVS_READONLY, &HCNVSHandle);
    if (hc_CurrentError == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGW(TAG, "Namespace %s not found in NVS", HTTPServerContext.NVSConfiguration.NVS_Namespace);
    } else if (hc_CurrentError == ESP_OK) {
        hc_CurrentError = nvs_get_str(HCNVSHandle, HTTPServerContext.NVSConfiguration.KeyConfigFile, NULL, &StringSize);
        if (hc_CurrentError != ESP_OK && hc_CurrentError != ESP_ERR_NVS_NOT_FOUND) ESP_ERROR_CHECK(hc_CurrentError);
        if (StringSize == 0) {
            ESP_LOGW(TAG, "Config file name not stored in NVS");
        } else {
            HTTPServerContext.NVSConfiguration.ConfigFileName = malloc(StringSize);
            hc_CurrentError = nvs_get_str(HCNVSHandle, HTTPServerContext.NVSConfiguration.KeyConfigFile, HTTPServerContext.NVSConfiguration.ConfigFileName, &StringSize);
            if (hc_CurrentError != ESP_OK) {
                free(HTTPServerContext.NVSConfiguration.ConfigFileName);
                HTTPServerContext.NVSConfiguration.ConfigFileName = NULL;
                nvs_close(HCNVSHandle);
                ESP_ERROR_CHECK(hc_CurrentError);
            }
            nvs_close(HCNVSHandle);
            //Read config file
            hc_CurrentError = LoadControlConfig(HTTPServerContext.NVSConfiguration.ConfigFileName, &HTTPServerContext);
        }
    } else {
        ESP_ERROR_CHECK(hc_CurrentError);
    }
    // Search one wire devices, in not found in LoadControlConfig function
    if (HTTPServerContext.NVSConfiguration.ConfigFileName == NULL && HTTPServerContext.DSes.Count == 0) {
        ESP_ERROR_CHECK(SearchOWDevices(OWBus, &HTTPServerContext));
    }
    // Start the file server called from wifi_event_handler
    ESP_ERROR_CHECK(esp_wifi_start());
    // Start task for read temperature
    if (xTaskCreate(ReadTemperatures, "ReadTemp", 1024*3, &HTTPServerContext, tskIDLE_PRIORITY, &TemperatureReaderHandle) != pdPASS)
       ESP_LOGE(TAG, "Error create task ReadTemperatures");
    hc_Status &= ~HC_STATUS_BIT_INIT;
    ESP_LOGI(TAG, "Initialize finished");
    // Check OTA updates after pause 10 sec for got ip address and start HTTP server
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            hc_Status &= ~HC_STATUS_BIT_OTA;
            ESP_LOGE(TAG, "OTA check timed out! Start rollback to the previous version ...");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }
    // Check and correct not found sensors
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (HTTPServerContext.NVSConfiguration.ConfigFileName != NULL && NotFoundSensorCount > 0) {
            i++;
            vTaskDelay(5000 * i / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Next attempt (%d) to load config file.", i);
            hc_CurrentError = LoadControlConfig(HTTPServerContext.NVSConfiguration.ConfigFileName, &HTTPServerContext);
        } else break;
    }
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&uninitializeCtrl));
    if (LastReset != ESP_RST_POWERON) SaveLogToFile();
    ESP_LOGI(TAG, "Startup complete");

}
