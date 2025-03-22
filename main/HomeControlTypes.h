#pragma once

#include "driver/gpio.h"
#include "onewire_bus.h"
#include "ds18x20.h"
#include "RingBufferHC.h"

#define MAC_ADDR_SIZE 6
#define MAX_DEVICES 8

typedef struct SystemInfoDataT {
    time_t StartupTime;
    uint8_t MacAddress[MAC_ADDR_SIZE];
    const char *HostName;
    uint32_t PartRunnAddr;
    uint32_t PartConfAddr;
    char RunningVersion[32];
    char InvalidVersion[32];
    char ProjectName[32];
    char IdfVersion[32];
    char ChipModel[25];
    uint16_t ChipRevision;
    uint8_t ChipCores;
    char ResetReason[25];
} SystemInfoDataT;

typedef struct DSDataT {
    char *dsName;
    ds18x20_device_handle_t DSDevice;
    onewire_device_address_t DSAddress;
    float LastValue;
    time_t LastRead;
    uint8_t NumberOfErrors;
    bool ReadError;
} DSDataT;

typedef struct DSDataT * DSDataHandleT;

typedef struct ControlOutputDataT {
    char *coName;
    gpio_num_t gpioPin;
    time_t DayStartTimeInSec;
    time_t DayStopTimeInSec;
    DSDataHandleT Thermometr;
    float DayMinTemp;
    float DayMaxTemp;
    float NigthMinTemp;
    float NigthMaxTemp;
    int CurrentCondition;
} ControlOutputDataT;

typedef struct ControlOutputDataT * ControlOutputDataHandleT;

typedef struct ControlsT {
    uint8_t Count;
    ControlOutputDataHandleT ControlOutputData[MAX_DEVICES];
} ControlsT;

typedef struct DSesT {
    uint8_t Count;
    DSDataHandleT DSDevices[MAX_DEVICES];
} DSesT;

typedef struct NVSConfigT{
    const char *NVS_Namespace;
    const char *KeyConfigFile;
    char *ConfigFileName;
        
} NVSConfigT;

typedef struct ContextDataT {
    char *BasePath;
    HCRingbufferT *LogBuffer;
    ControlsT Controls;
    DSesT DSes;
    SystemInfoDataT SystemInfo;
    NVSConfigT NVSConfiguration;
    SemaphoreHandle_t ControlDSData_mux;
} ContextDataT;

typedef struct ServerDataT {
    httpd_handle_t ServerHandle;
    ContextDataT *ContextServerData;
} ServerDataT;

esp_err_t LoadControlConfig(char *FileName, ContextDataT *DataCntx);
