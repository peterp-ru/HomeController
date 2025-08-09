#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_vfs.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "nvs_flash.h"
#include "RingBufferHC.h"
#include "HTTPServerHC.h"

/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define MAX_FILE_SIZE   (200*1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"
#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)
// #define HTTPD_CONN_CLOSE_HEADER true

const char *TAG = "HC_HTTPServerHC";
const char *FileManagerPrefix = "/FileManager";
const char *HTMLPageStart = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\"/>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";

void vRestartCallback(TimerHandle_t tmr){
    esp_restart();
    
}
    
void FinalizeHTTPPage(httpd_req_t *req){
    #ifdef HTTPD_CONN_CLOSE_HEADER
        httpd_resp_set_hdr(req, "Connection", "close");
    #endif
        httpd_resp_send_chunk(req, NULL, 0);
    
}
    
void SendStartOfHTTPPage(httpd_req_t *req, const char *StartStr){
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_sendstr_chunk(req, StartStr);

}

esp_err_t GetLogPageHandler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/plain");   
    size_t tmpBufferSize = 0x3FFF; //16K
    int Getted;
    size_t ScratchBufferSize = MIN(tmpBufferSize, (int)(esp_get_free_heap_size() / 4));
    char *tmpBuffer = malloc(ScratchBufferSize); 
    if (tmpBuffer == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send log file, no memory");
        return ESP_ERR_NO_MEM;
    }
    Getted = RingBufferRead(((struct ContextDataT *)req->user_ctx)->LogBuffer, tmpBuffer, tmpBufferSize, true);
    while (Getted >0) {
        if (httpd_resp_send_chunk(req, tmpBuffer, Getted) != ESP_OK) {
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send log file");
            free(tmpBuffer);
            return ESP_FAIL;
        }
        Getted = RingBufferRead(((struct ContextDataT *)req->user_ctx)->LogBuffer, tmpBuffer, tmpBufferSize, false);
    }
    if (Getted < 0) {
        httpd_resp_sendstr_chunk(req, NULL);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Semaphore timed out");
        free(tmpBuffer);
        return ESP_FAIL;
    }
    FinalizeHTTPPage(req);
    free(tmpBuffer);
    return ESP_OK;

}
    
static esp_err_t ConfigurationPageHandler(httpd_req_t *req){
    const int HourInSec = 3600;
    char *tmpStr;
    char tmpBuf[140];
    uint16_t mHour;
    uint16_t mMinute;
    uint64_t mAddress;
    ContextDataT *DataCntx = (struct ContextDataT *)req->user_ctx;
    ControlsT mControls = DataCntx->Controls;
    NVSConfigT mNVSConfig = DataCntx->NVSConfiguration;
    SemaphoreHandle_t SemaphoreMux = DataCntx->ControlDSData_mux;
    SendStartOfHTTPPage(req, HTMLPageStart);
    httpd_resp_sendstr_chunk(req, "<title>ESP-32 device</title></head><body><h2>Configuration</h2><br>");
    if (mControls.Count > 0){
        httpd_resp_sendstr_chunk(req, "<h3>Controls informarion</h3>");
        if (xSemaphoreTake(SemaphoreMux, HC_SemMaxWaitTime / portTICK_PERIOD_MS)) {
            httpd_resp_sendstr_chunk(req, "<table border=\"1\"><tr><th>Name</th><th>Load</th><th>DSNumber</th>"
                "<th>DayStartTime</th><th>DayStopTime</th><th>DayMinTemp</th><th>DayMaxTemp</th>"
                "<th>NigthMinTemp</th><th>NigthMaxTemp</th></tr>");
            for (int i = 0; i < mControls.Count; i++){
                if (mControls.ControlOutputData[i]->coName == NULL) tmpStr = "Unnamed"; else tmpStr = mControls.ControlOutputData[i]->coName;
                if (mControls.ControlOutputData[i]->Thermometr != NULL)  mAddress = mControls.ControlOutputData[i]->Thermometr->DSAddress;
                else mAddress = 0;
                mHour = mControls.ControlOutputData[i]->DayStartTimeInSec / HourInSec;
                mMinute = (mControls.ControlOutputData[i]->DayStartTimeInSec - mHour * HourInSec) / 60;
                sprintf(tmpBuf, "<tr align = \"center\"><td>%s</td><td>%d</td><td>%016llX</td><td>%02d:%02d</td>", 
                    tmpStr, mControls.ControlOutputData[i]->gpioPin, mAddress, mHour, mMinute);
                httpd_resp_sendstr_chunk(req, tmpBuf);
                mHour = mControls.ControlOutputData[i]->DayStopTimeInSec / HourInSec;
                mMinute = (mControls.ControlOutputData[i]->DayStopTimeInSec - mHour * HourInSec) / 60;
                sprintf(tmpBuf, "<td>%02d:%02d</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>", 
                    mHour, mMinute, mControls.ControlOutputData[i]->DayMinTemp, mControls.ControlOutputData[i]->DayMaxTemp,
                    mControls.ControlOutputData[i]->NigthMinTemp, mControls.ControlOutputData[i]->NigthMaxTemp);
                httpd_resp_sendstr_chunk(req, tmpBuf);
            }
            httpd_resp_sendstr_chunk(req, "</table>");
            xSemaphoreGive(SemaphoreMux);
        } else httpd_resp_sendstr_chunk(req, "<br>Read data failed! Semaphore timed out.");
    }
    httpd_resp_sendstr_chunk(req, "<br><table border=\"0\"><col width=\"700px\"/><tr><td><table border=\"0\"><tr>"
        "<td>Current config file:</td><td>");
    if(mNVSConfig.ConfigFileName) httpd_resp_sendstr_chunk(req, mNVSConfig.ConfigFileName);
    httpd_resp_sendstr_chunk(req, "</td></tr><tr><td><label for=\"filename\">New config file:</label></td>"
        "<td><input id=\"filename\" type=\"text\"></td></tr><tr><td><button id=\"upload\" type=\"button\" onclick"
        "=\"upload()\">Apply new Configuration</button></td></tr></table></td></tr></table>");
    /* Get handle to embedded firmware update script */
    extern const unsigned char Config_script_start[] asm("_binary_ConfigurationScript_html_start");
    extern const unsigned char Config_script_end[]   asm("_binary_ConfigurationScript_html_end");
    const size_t Config_script_size = (Config_script_end - Config_script_start);
    httpd_resp_send_chunk(req, (const char *)Config_script_start, Config_script_size);
    FinalizeHTTPPage(req);
    return ESP_OK;

}
    
    static esp_err_t HomePageHandler(httpd_req_t *req){
        char tmpBuf[140];
        const char *tmpStr;
        time_t now = 0;
        struct tm timeinfo = { 0 };
        SystemInfoDataT SystemInfoData = ((struct ContextDataT *)req->user_ctx)->SystemInfo;
        SendStartOfHTTPPage(req, HTMLPageStart);
        httpd_resp_sendstr_chunk(req, "<title>ESP-32 device</title>"
            "</head><body><h1>ESP-32 device</h1><h2>Home controller</h2>"
            "<h3>System information</h3><table><col width=\"150px\"/><tr><td>Current time</td><td>");
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(tmpBuf, 140, "%d.%m.%Y %X", &timeinfo);
        httpd_resp_sendstr_chunk(req, tmpBuf);
        httpd_resp_sendstr_chunk(req, "</td></tr>");
    
        time_t UpTime = now - SystemInfoData.StartupTime;
        int Days = (int)(UpTime / 86400);
        int Hours = (int)((UpTime - Days * 86400) / 3600);
        sprintf(tmpBuf, "<tr><td>Uptime</td><td>%d %02d:", Days, Hours);
        httpd_resp_sendstr_chunk(req, tmpBuf);
        localtime_r(&UpTime, &timeinfo);
        strftime(tmpBuf, 140, "%M:%S", &timeinfo);
        httpd_resp_sendstr_chunk(req, tmpBuf);
        httpd_resp_sendstr_chunk(req, "</td></tr>");
    
        sprintf(tmpBuf, "<tr><td>Heap memory</td><td>%ld</td></tr>", esp_get_free_heap_size());
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>Minimum memory</td><td>%ld</td></tr>", esp_get_minimum_free_heap_size());
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        httpd_resp_sendstr_chunk(req, "<tr><td>Reset reason</td><td>");
        httpd_resp_sendstr_chunk(req, SystemInfoData.ResetReason);
        httpd_resp_sendstr_chunk(req, "</td></tr>");
    
        sprintf(tmpBuf, "<tr><td>Firmware name</td><td>%s</td></tr>", SystemInfoData.ProjectName);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>Firmware version</td><td>%s</td></tr>", SystemInfoData.RunningVersion);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>SDK version</td><td>%s</td></tr>", SystemInfoData.IdfVersion);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>Host name</td><td>%s</td></tr>", SystemInfoData.HostName);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>MAC address</td><td>%02x:%02x:%02x:%02x:%02x:%02x</td></tr>", 
            SystemInfoData.MacAddress[0], SystemInfoData.MacAddress[1], SystemInfoData.MacAddress[2],
            SystemInfoData.MacAddress[3], SystemInfoData.MacAddress[4], SystemInfoData.MacAddress[5]);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        httpd_resp_sendstr_chunk(req, "<tr><td>Chip model</td><td>");
        httpd_resp_sendstr_chunk(req, SystemInfoData.ChipModel);
    
        httpd_resp_sendstr_chunk(req, "</td></tr>");
        sprintf(tmpBuf, "<tr><td>Chip revision number</td><td>%d.%02d</td></tr>", 
            (int)(SystemInfoData.ChipRevision / 100), SystemInfoData.ChipRevision - 100);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>CPU cores</td><td>%d</td></tr>", SystemInfoData.ChipCores);
        httpd_resp_sendstr_chunk(req, tmpBuf);
        httpd_resp_sendstr_chunk(req, "</table>");
        DSesT mDSes = ((struct ContextDataT *)req->user_ctx)->DSes;
        SemaphoreHandle_t SemaphoreMux = ((struct ContextDataT *)req->user_ctx)->ControlDSData_mux;
        if (mDSes.Count > 0){
            httpd_resp_sendstr_chunk(req, "<h3>Temperature sensor information</h3>");
            if (xSemaphoreTake(SemaphoreMux, HC_SemMaxWaitTime / portTICK_PERIOD_MS)) {
                httpd_resp_sendstr_chunk(req, "<table border=\"1\"><thead><tr><th>Name</th><th>Address</th><th>Last read</th>"
                    "<th>Errors</th><th>Value</th></tr></thead><tbody>");
                for (int i = 0; i < mDSes.Count; i++){
                    if (mDSes.DSDevices[i]->dsName == NULL) tmpStr = "Unnamed"; else tmpStr = mDSes.DSDevices[i]->dsName;
                    localtime_r(&mDSes.DSDevices[i]->LastRead, &timeinfo);
                    sprintf(tmpBuf, "<tr><td>%s</td><td>%016llX</td><td>", tmpStr, mDSes.DSDevices[i]->DSAddress);
                    httpd_resp_sendstr_chunk(req, tmpBuf);
                    strftime(tmpBuf, 140, "%d.%m.%Y %X", &timeinfo);
                    httpd_resp_sendstr_chunk(req, tmpBuf);
                    sprintf(tmpBuf, "</td><td>%d</td><td>%.2f</td></tr>", mDSes.DSDevices[i]->NumberOfErrors, mDSes.DSDevices[i]->LastValue);
                    httpd_resp_sendstr_chunk(req, tmpBuf);
                }
                httpd_resp_sendstr_chunk(req, "</tbody></table>");
                xSemaphoreGive(SemaphoreMux);
            } else httpd_resp_sendstr_chunk(req, "<br>Read data failed! Semaphore timed out.");
        }
        httpd_resp_sendstr_chunk(req, "<br><a href=\"/FileManager\">File manager</a>"
            "<br><a href=\"/FirmwareUpdate\">Firmware update</a>"
            "<br><a href=\"/Configuration\">Device configuration</a>"
            "<br><a href=\"/GetLog\">Log file</a></body></html>");
        FinalizeHTTPPage(req);
        return ESP_OK;
    
    }
    
    static esp_err_t FirmwareUpdateHandler(httpd_req_t *req){
        char tmpBuf[140];
        SystemInfoDataT SystemInfoData = ((struct ContextDataT *)req->user_ctx)->SystemInfo;
        SendStartOfHTTPPage(req, HTMLPageStart);
        httpd_resp_sendstr_chunk(req, "<title>ESP-32 device</title></head><body><h2>ESP-32 Firmware update</h2>"
            "<table border=\"1\"><col width=\"200px\"/><col width=\"100px\"/>");
    
        sprintf(tmpBuf, "<tr><td>Running partition:</td><td align=\"right\">%ld</td></tr>", SystemInfoData.PartRunnAddr);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>Boot partition:</td><td align=\"right\">%ld</td></tr>", SystemInfoData.PartConfAddr);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        httpd_resp_sendstr_chunk(req, "<tr><td colspan =\"2\">Not equal value can happen if either "
            "the OTA boot data or preferred boot image become corrupted somehow.</td></tr>");
    
        sprintf(tmpBuf, "<tr><td>Running firmware version:</td><td align=\"right\">%s</td></tr>", SystemInfoData.RunningVersion);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        sprintf(tmpBuf, "<tr><td>Invalid firmware version:</td><td align=\"right\">%s</td></tr>", SystemInfoData.InvalidVersion);
        httpd_resp_sendstr_chunk(req, tmpBuf);
    
        httpd_resp_sendstr_chunk(req, "</table>");
    
        /* Get handle to embedded firmware update script */
        extern const unsigned char firmware_script_start[] asm("_binary_FirmwareScript_html_start");
        extern const unsigned char firmware_script_end[]   asm("_binary_FirmwareScript_html_end");
        const size_t firmware_script_size = (firmware_script_end - firmware_script_start);
        httpd_resp_send_chunk(req, (const char *)firmware_script_start, firmware_script_size);
        FinalizeHTTPPage(req);
        return ESP_OK;
    
    }
    
/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize){
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;

}

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
 static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath){
    char entrypath[FILE_PATH_MAX];
    char entrysize[16];
    const char *entrytype;

    struct dirent *entry;
    struct stat entry_stat;

    DIR *dir = opendir(dirpath);
    const size_t dirpath_len = strlen(dirpath);

    /* Retrieve the base path of file storage to construct the full path */
    strlcpy(entrypath, dirpath, sizeof(entrypath));

    if (!dir) {
        ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
        return ESP_FAIL;
    }

    /* Send HTML file header */
    SendStartOfHTTPPage(req, HTMLPageStart);
    httpd_resp_sendstr_chunk(req, "<title>ESP-32 device</title></head><body><h2>ESP-32 File Manager</h2>");

    /* Get handle to embedded file upload script */
    extern const unsigned char upload_script_start[] asm("_binary_upload_script_html_start");
    extern const unsigned char upload_script_end[]   asm("_binary_upload_script_html_end");
    const size_t upload_script_size = (upload_script_end - upload_script_start);

    /* Add file upload form and script which on execution sends a POST request to /upload */
    httpd_resp_send_chunk(req, (const char *)upload_script_start, upload_script_size);

    /* Send file-list table definition and column labels */
    httpd_resp_sendstr_chunk(req,
        "<table border=\"1\">"
        "<col width=\"300px\" /><col width=\"50px\" /><col width=\"90px\" /><col width=\"60px\" />"
        "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th><th>Delete</th></tr></thead>"
        "<tbody>");

    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL) {
        entrytype = (entry->d_type == DT_DIR ? "folder" : "file");

        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1) {
            ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%ld", entry_stat.st_size);

        /* Send chunk of HTML file containing table entries with file name and size */
        httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        if (entry->d_type == DT_DIR) {
            httpd_resp_sendstr_chunk(req, "/");
        }
        httpd_resp_sendstr_chunk(req, "\">");
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "</a></td><td>");
        httpd_resp_sendstr_chunk(req, entrytype);
        httpd_resp_sendstr_chunk(req, "</td><td align=\"right\">");
        httpd_resp_sendstr_chunk(req, entrysize);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/delete");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
        httpd_resp_sendstr_chunk(req, "</td></tr>\n");
    }
    closedir(dir);

    /* Finish the file list table */
    httpd_resp_sendstr_chunk(req, "</tbody></table>");
    /* Used and Total bytes */
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        char UsedBytes[250];
        sprintf(UsedBytes, "<br><table><tr><td>Total bytes:</td><td align=\"right\">%d</td></tr>"
            "<tr><td>Used bytes:</td><td align=\"right\">%d</td></tr><tr>"
            "<td>Free bytes:</td><td align=\"right\">%d</td></tr></table>", total, used, total - used);
        httpd_resp_sendstr_chunk(req, UsedBytes);
     }
    /* Send remaining chunk of HTML file to complete it */
    httpd_resp_sendstr_chunk(req, "<a href=\"/\">Home page</a></body></html>");
    FinalizeHTTPPage(req);
    return ESP_OK;

}

/* Handler to redirect incoming GET request for /index.html to /
 * This can be overridden by uploading file with same name */
static esp_err_t index_html_get_handler(httpd_req_t *req){
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;

}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
static esp_err_t favicon_get_handler(httpd_req_t *req){
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=31536000, immutable");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    FinalizeHTTPPage(req);
    return ESP_OK;

}

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename){
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");

}

/* Handler to upload a file onto the server */
static esp_err_t upload_post_handler(httpd_req_t *req){
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    /* Skip leading "/upload" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct ContextDataT *)req->user_ctx)->BasePath,
                                             req->uri + sizeof("/upload") - 1, sizeof(filepath));
    if (!filename) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == 0) {
        ESP_LOGE(TAG, "File already exists : %s", filepath);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");
        return ESP_FAIL;
    }

    /* File cannot be larger than a limit */
    if (req->content_len > MAX_FILE_SIZE) {
        ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "File size must be less than "
                            MAX_FILE_SIZE_STR "!");
        /* Return failure to close underlying connection else the
         * incoming file content will keep the socket busy */
        return ESP_FAIL;
    }

    fd = fopen(filepath, "w");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to create file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Receiving file : %s...", filename);

    /* Content length of the request gives
     * the size of the file being uploaded */
    int remaining = req->content_len;
    /* Retrieve the pointer to scratch buffer for temporary storage */
    size_t ScratchBufferSize = MIN(remaining, (int)(esp_get_free_heap_size() / 4));
    char *Buf = malloc(ScratchBufferSize);
    if (!Buf) {
        fclose(fd);
        unlink(filepath);
        ESP_LOGE(TAG, "No mem for receive file. Requested size %d", ScratchBufferSize);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
        return ESP_FAIL;
    }
    int received;
    while (remaining > 0) {
        /* Receive the file part by part into a buffer */
        if ((received = httpd_req_recv(req, Buf, MIN(remaining, ScratchBufferSize))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry if timeout occurred */
                continue;
            }
            /* In case of unrecoverable error,
             * close and delete the unfinished file*/
            free(Buf);
            fclose(fd);
            unlink(filepath);
            ESP_LOGE(TAG, "File reception failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }
        /* Write buffer content to file on storage */
        if (received && (received != fwrite(Buf, 1, received, fd))) {
            /* Couldn't write everything to file!
             * Storage may be full? */
            free(Buf);
            fclose(fd);
            unlink(filepath);
            ESP_LOGE(TAG, "File write failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");
            return ESP_FAIL;
        }
        /* Keep track of remaining size of
         * the file left to be uploaded */
        remaining -= received;
    }
    free(Buf);
    /* Close file upon upload completion */
    fclose(fd);
    ESP_LOGI(TAG, "File reception complete");
    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", FileManagerPrefix);
    httpd_resp_sendstr(req, "File saved successfully");
    return ESP_OK;

}

/* Handler to delete a file from the server */
static esp_err_t delete_post_handler(httpd_req_t *req){
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    /* Skip leading "/delete" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct ContextDataT *)req->user_ctx)->BasePath,
                                             req->uri  + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "File does not exist : %s", filename);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting file : %s", filename);
    /* Delete file */
    unlink(filepath);

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", FileManagerPrefix);
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;

}

static esp_err_t otaupdate_post_handler(httpd_req_t *req){
    char filepath[FILE_PATH_MAX];
    esp_err_t err;
    SystemInfoDataT SystemInfoData = ((struct ContextDataT *)req->user_ctx)->SystemInfo;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    const char *filename = get_path_from_uri(filepath, ((struct ContextDataT *)req->user_ctx)->BasePath,
                                             req->uri  + sizeof("/otaupdate") - 1, sizeof(filepath));
    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    /* Content length of the request gives the size of the file being uploaded */
    int remaining = req->content_len;
    /* Retrieve the pointer to scratch buffer for temporary storage */
    size_t ScratchBufferSize = MIN(remaining, (int)(esp_get_free_heap_size() / 2));
    char *Buf = malloc(ScratchBufferSize);
    if (!Buf) {
        ESP_LOGE(TAG, "No mem for receive file. Requested size %d", ScratchBufferSize);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
        return ESP_FAIL;
    }
    int received;
    bool image_header_was_checked = false;
    bool FailInWhile = false;
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        free(Buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to esp_ota_get_next_update_partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32,
             update_partition->subtype, update_partition->address);
    while (remaining > 0) {
        received = httpd_req_recv(req, Buf, MIN(remaining, ScratchBufferSize));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry if timeout occurred */
                continue;
            }
            /* In case of unrecoverable error */
            ESP_LOGE(TAG, "File reception failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            FailInWhile = true;
            break;
        }
        if (!image_header_was_checked) {
            esp_app_desc_t new_app_info;
            if (received > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                // check current version with downloading
                memcpy(&new_app_info, &Buf[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                char tmpEentry[32];
                strlcpy(tmpEentry, new_app_info.version, sizeof(tmpEentry));
                ESP_LOGI(TAG, "New firmware version: %s", tmpEentry);
                // check project name
                if (memcmp(new_app_info.project_name, SystemInfoData.ProjectName, sizeof(new_app_info.project_name)) != 0) {
                    ESP_LOGW(TAG, "Wrong firmware name. We will not continue the update.");
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Wrong firmware name. We will not continue the update.");
                    FailInWhile = true;
                    break;
                }
                // check current version with last invalid partition
                if (memcmp(SystemInfoData.InvalidVersion, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                    ESP_LOGW(TAG, "New version is the same as invalid version.");
                    ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", SystemInfoData.InvalidVersion);
                    ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "New version is the same as invalid version.");
                    FailInWhile = true;
                    break;
                }
                if (memcmp(new_app_info.version, SystemInfoData.RunningVersion, sizeof(new_app_info.version)) == 0) {
                    ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Current running version is the same as a new.");
                    FailInWhile = true;
                    break;
                }
                image_header_was_checked = true;
                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
                    esp_ota_abort(update_handle);
                    FailInWhile = true;
                    break;
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");
            } else {
                ESP_LOGE(TAG, "received package is not fit len");
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "received package is not fit len");
                FailInWhile = true;
                break;
            }
        }
        err = esp_ota_write(update_handle, (const void *)Buf, received);
        if (err != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
            esp_ota_abort(update_handle);
            FailInWhile = true;
            break;
        }
        remaining -= received;
    }
    free(Buf);
    if (FailInWhile) {
        return ESP_FAIL;
    } 
    
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Image validation failed, image is corrupted");
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_end failed");
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_set_boot_partition failed");
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    const int RestartDelay = 10;
    TimerHandle_t  vRestartTimer = xTimerCreate("restart_tmr", RestartDelay * 1000 / portTICK_PERIOD_MS, pdFALSE, NULL, vRestartCallback);
    xTimerStart(vRestartTimer, 0);
    ESP_LOGI(TAG, "Scheduled restart at %d second", RestartDelay);
    httpd_resp_sendstr(req, "Flash is complete. Device restarting in 10 sec. Press Refresh buitton on you browser.");
    return ESP_OK;

}

static esp_err_t NewConfigFileName_post_handler(httpd_req_t *req){
    ContextDataT *HTTPServerCntx = (struct ContextDataT *)req->user_ctx;
    //char *mBasePath = ((struct ContextDataT *)req->user_ctx)->BasePath;
    int received;
    char FullFileName[FILE_PATH_MAX];
    struct stat file_stat;
    nvs_handle_t HCNVSHandle;
    //NVSConfigT mNVSConfug = ((struct ContextDataT *)req->user_ctx)->NVSConfiguration;
    //DSesT *mDSes = &((struct ContextDataT *)req->user_ctx)->DSes;
    //ControlsT *mControls = &((struct ContextDataT *)req->user_ctx)->Controls;
    /* Content length of the request gives the size of the file being uploaded */
    size_t base_pathlen = strlen(HTTPServerCntx->BasePath);
    if (req->content_len >= FILE_PATH_MAX - base_pathlen - 1) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    size_t ScratchBufferSize = MIN(req->content_len, (int)(esp_get_free_heap_size() / 4));
    char *Buf = malloc(ScratchBufferSize);
    if (!Buf) {
        ESP_LOGE(TAG, "No mem for receive file name. Requested size %d", ScratchBufferSize);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file name");
        return ESP_FAIL;
    }
    received = httpd_req_recv(req, Buf, req->content_len);
    if (received != req->content_len) {
        free(Buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file name");
        return ESP_FAIL;
    }
    strcpy(FullFileName, HTTPServerCntx->BasePath);
    FullFileName[base_pathlen] = '/';
    strlcpy(FullFileName + base_pathlen + 1, Buf, received + 1);
    free(Buf);
    if (stat(FullFileName, &file_stat) == -1) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File does not exist");
        return ESP_FAIL;
    }
    if (LoadControlConfig(FullFileName, HTTPServerCntx) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Errors in LoadControlConfig");
        return ESP_FAIL;
    }
    // store FileName in NVS
    if (nvs_open(HTTPServerCntx->NVSConfiguration.NVS_Namespace, NVS_READWRITE, &HCNVSHandle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot open NVS for save data");
        return ESP_FAIL;
    }
    if (nvs_set_str(HCNVSHandle, HTTPServerCntx->NVSConfiguration.KeyConfigFile, FullFileName) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot save data in NVS");
        return ESP_FAIL;
    };
    nvs_commit(HCNVSHandle);
    nvs_close(HCNVSHandle);
    if (HTTPServerCntx->NVSConfiguration.ConfigFileName != NULL) free(HTTPServerCntx->NVSConfiguration.ConfigFileName);
    HTTPServerCntx->NVSConfiguration.ConfigFileName = strdup(FullFileName);
    httpd_resp_sendstr(req, "File saved, configuration updated. Press Refresh buitton on you browser.");
    return ESP_OK;

}

static esp_err_t download_get_handler(httpd_req_t *req){
    const char *FirmwareUpdatePrefix = "/FirmwareUpdate";
    const char *ConfigurationPrefix = "/Configuration";
    const char *GetLogPrefix = "/GetLog";
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    if (req->uri[strlen(req->uri) - 1] == '/') {
        return HomePageHandler(req);
    }
    char *PosFM = strstr(req->uri, FirmwareUpdatePrefix);
    if (PosFM != NULL) {
        return FirmwareUpdateHandler(req);
    }
    PosFM = strstr(req->uri, ConfigurationPrefix);
    if (PosFM != NULL) 
        return ConfigurationPageHandler(req);
    PosFM = strstr(req->uri, GetLogPrefix);
    if (PosFM != NULL) 
        return GetLogPageHandler(req);

    PosFM = strstr(req->uri, FileManagerPrefix);
    if (PosFM != NULL) {
        strlcpy(PosFM + 1, PosFM + strlen(FileManagerPrefix), strlen(req->uri));
    }

    const char *filename = get_path_from_uri(filepath, ((struct ContextDataT *)req->user_ctx)->BasePath,
                                             req->uri, sizeof(filepath));
    if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
        return http_resp_dir_html(req, filepath);
    }

    /* Check if URI correspond to one of the hardcoded paths */
    if (strcmp(filename, "/index.html") == 0) {
        return index_html_get_handler(req);
    } else if (strcmp(filename, "/favicon.ico") == 0) {
        return favicon_get_handler(req);
    }

    if (stat(filepath, &file_stat) == -1) {
        /* If file not present  */
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Error 404 - Page not found");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);
    /* Retrieve the pointer to scratch buffer for temporary storage */
    size_t ScratchBufferSize = MIN(file_stat.st_size, (int)(esp_get_free_heap_size() / 4));
    char *Buf = malloc(ScratchBufferSize);
    if (!Buf) {
        ESP_LOGE(TAG, "No mem for receive file name. Requested size %d", ScratchBufferSize);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file name");
        return ESP_FAIL;
    }
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(Buf, 1, ScratchBufferSize, fd);
        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, Buf, chunksize) != ESP_OK) {
                fclose(fd);
                free(Buf);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
               return ESP_FAIL;
           }
        }
        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);
    /* Close file after sending complete */
    fclose(fd);
    free(Buf);
    ESP_LOGI(TAG, "File sending complete");
    FinalizeHTTPPage(req);
    return ESP_OK;

}

esp_err_t Start_HTTP_Server(ServerDataT *HTTPDataHndl){
    if (HTTPDataHndl->ServerHandle != NULL) {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&HTTPDataHndl->ServerHandle, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "HTTP Server started. Registering handlers...");
    /* URI handler for getting uploaded files */
    httpd_uri_t file_download = {
        .uri       = "/*",  // Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = download_get_handler,
        .user_ctx  = HTTPDataHndl->ContextServerData    // Pass server data as context
    };
    httpd_register_uri_handler(HTTPDataHndl->ServerHandle, &file_download);
    /* URI handler for uploading files to server */
    httpd_uri_t file_upload = {
        .uri       = "/upload/*",   // Match all URIs of type /upload/path/to/file
        .method    = HTTP_POST,
        .handler   = upload_post_handler,
        .user_ctx  = HTTPDataHndl->ContextServerData    // Pass server data as context
    };
    httpd_register_uri_handler(HTTPDataHndl->ServerHandle, &file_upload);
    /* URI handler for deleting files from server */
    httpd_uri_t file_delete = {
        .uri       = "/delete/*",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_POST,
        .handler   = delete_post_handler,
        .user_ctx  = HTTPDataHndl->ContextServerData    // Pass server data as context
    };
    httpd_register_uri_handler(HTTPDataHndl->ServerHandle, &file_delete);
    /* URI handler for upload for ota update */
    httpd_uri_t file_otaupdate = {
        .uri       = "/otaupdate/*",   // Match all URIs of type /otaupdate/path/to/file
        .method    = HTTP_POST,
        .handler   = otaupdate_post_handler,
        .user_ctx  = HTTPDataHndl->ContextServerData    // Pass server data as context
    };
    httpd_register_uri_handler(HTTPDataHndl->ServerHandle, &file_otaupdate);
    /* URI handler for upload for config file name update */
    httpd_uri_t NewConfigFileName = {
        .uri       = "/NewConfigFileName",   // Match URIs of type /NewConfigFileName
        .method    = HTTP_POST,
        .handler   = NewConfigFileName_post_handler,
        .user_ctx  = HTTPDataHndl->ContextServerData    // Pass server data as context
    };
    httpd_register_uri_handler(HTTPDataHndl->ServerHandle, &NewConfigFileName);
    ESP_LOGI(TAG, "HTTP handlers registered");
    return ESP_OK;

}

esp_err_t Stop_HTTP_Server(ServerDataT *HTTPDataHndl){
    esp_err_t ret=ESP_OK;
    ESP_LOGD(TAG, "TTPDataHndl->ServerHandle: %p", HTTPDataHndl->ServerHandle);
    if (HTTPDataHndl->ServerHandle){
        httpd_unregister_uri_handler(HTTPDataHndl->ServerHandle, "/NewConfigFileName", HTTP_POST);    // NewConfigFileName   
        httpd_unregister_uri_handler(HTTPDataHndl->ServerHandle, "/otaupdate/*", HTTP_POST);          // file_otaupdate   
        httpd_unregister_uri_handler(HTTPDataHndl->ServerHandle, "/delete/*", HTTP_POST);             // file_delete 
        httpd_unregister_uri_handler(HTTPDataHndl->ServerHandle, "/upload/*", HTTP_POST);             // file_upload 
        httpd_unregister_uri_handler(HTTPDataHndl->ServerHandle, "/*", HTTP_GET);                     // file_download
        ESP_LOGD(TAG, "HTTP handlers unregistered. Stopping server...");
        ret = httpd_stop(HTTPDataHndl->ServerHandle);
        HTTPDataHndl->ServerHandle = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
    return ret;

}
