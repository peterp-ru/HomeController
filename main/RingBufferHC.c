#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "RingBufferHC.h"

esp_err_t RingBufferCreate(HCRingbufferT *Buffer, size_t Size){
    Buffer->RingBuffer_mux = xSemaphoreCreateMutex();
    if (xSemaphoreTake(Buffer->RingBuffer_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS) == pdFALSE) return ESP_ERR_TIMEOUT;
    Buffer->mBuffer = malloc(Size);
    if (Buffer->mBuffer == NULL) return ESP_ERR_NO_MEM;
    Buffer->mSize = Size;
    Buffer->mHead = 0;
    Buffer->mTail = 0;
    Buffer->mFree = Size;
    xSemaphoreGive(Buffer->RingBuffer_mux);
    return ESP_OK;

}

esp_err_t RingBufferDestroy(HCRingbufferT *Buffer){
    if (xSemaphoreTake(Buffer->RingBuffer_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS) == pdFALSE) return ESP_ERR_TIMEOUT;
    if (Buffer->mBuffer) {
        free(Buffer->mBuffer);
        Buffer->mBuffer = NULL;
        Buffer->mSize = 0;
        Buffer->mFree = 0;
    }
    xSemaphoreGive(Buffer->RingBuffer_mux);
    return ESP_OK;  

}

int RingBufferPut(HCRingbufferT *Buffer, const char *pvItem, size_t mItemSize){
    if (xSemaphoreTake(Buffer->RingBuffer_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS) == pdFALSE) return -1;
    if (mItemSize > Buffer->mSize) {
        xSemaphoreGive(Buffer->RingBuffer_mux);
        return 0;
    }
    if (Buffer->mSize - Buffer->mTail >= mItemSize){ 
        memcpy(Buffer->mTail + Buffer->mBuffer, pvItem, mItemSize);
        Buffer->mTail += mItemSize;
    } else {
        size_t FirstPart = Buffer->mSize - Buffer->mTail;
        size_t SecondPart = mItemSize - FirstPart;
        if (FirstPart > 0) memcpy(Buffer->mTail + Buffer->mBuffer, pvItem, FirstPart);
        memcpy(Buffer->mBuffer, pvItem + FirstPart, SecondPart);
        Buffer->mTail = SecondPart;
    }
    if (Buffer->mFree > mItemSize){
        Buffer->mFree -= mItemSize;
    } else {
        Buffer->mFree = 0;
        Buffer->mHead = Buffer->mTail;
    }
    xSemaphoreGive(Buffer->RingBuffer_mux);
    return mItemSize;

}

int RingBufferGet(HCRingbufferT *Buffer, char *pvItem, size_t mItemSize){
    if (xSemaphoreTake(Buffer->RingBuffer_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS) == pdFALSE) return -1;
    if (mItemSize > Buffer->mSize - Buffer->mFree) mItemSize = Buffer->mSize - Buffer->mFree;
    if (mItemSize == 0) {
        xSemaphoreGive(Buffer->RingBuffer_mux);
        return 0; 
    }
    if (Buffer->mSize - Buffer->mHead >= mItemSize){ 
        memcpy(pvItem, Buffer->mBuffer + Buffer->mHead, mItemSize);
        Buffer->mHead += mItemSize;
    } else {
        size_t FirstPart = Buffer->mSize - Buffer->mHead;
        size_t SecondPart = mItemSize - FirstPart;
        if (FirstPart > 0) memcpy(pvItem, Buffer->mBuffer + Buffer->mHead, FirstPart);
        memcpy(pvItem + FirstPart, Buffer->mBuffer, SecondPart);
        Buffer->mHead = SecondPart;
    }
    Buffer->mFree += mItemSize;
    xSemaphoreGive(Buffer->RingBuffer_mux);
    return mItemSize;

}

int RingBufferRead(HCRingbufferT *Buffer, char *pvItem, size_t mItemSize, bool StartRead){
    size_t Remains;
    if (xSemaphoreTake(Buffer->RingBuffer_mux, HC_SemMaxWaitTime / portTICK_PERIOD_MS) == pdFALSE) return -1;
    if (StartRead) Buffer->mReadPoint = Buffer->mHead;
    if (Buffer->mTail > Buffer->mReadPoint)
        Remains = Buffer->mTail - Buffer->mReadPoint;
    else if (Buffer->mTail == Buffer->mReadPoint)
        if (Buffer->mFree == 0 && StartRead) 
            Remains = Buffer->mSize;
        else
            Remains = 0;
    else
        Remains = Buffer->mSize - Buffer->mReadPoint + Buffer->mTail;
    if (mItemSize > Remains) mItemSize = Remains;
    if (mItemSize == 0) {
        xSemaphoreGive(Buffer->RingBuffer_mux);
        return 0; 
    }
    if (Buffer->mSize - Buffer->mReadPoint >= mItemSize){ 
        memcpy(pvItem, Buffer->mBuffer + Buffer->mReadPoint, mItemSize);
        Buffer->mReadPoint += mItemSize;
    } else {
        size_t FirstPart = Buffer->mSize - Buffer->mReadPoint;
        size_t SecondPart = mItemSize - FirstPart;
        if (FirstPart > 0) memcpy(pvItem, Buffer->mBuffer + Buffer->mReadPoint, FirstPart);
        memcpy(pvItem + FirstPart, Buffer->mBuffer, SecondPart);
        Buffer->mReadPoint = SecondPart;
    }
    xSemaphoreGive(Buffer->RingBuffer_mux);
    return mItemSize;

}
