#pragma once

#define HC_SemMaxWaitTime 3000 

typedef struct HCRingbufferDefinition {
    uint8_t *mBuffer;
    size_t mSize;
    size_t mHead;
    size_t mTail;
    size_t mFree;
    size_t mReadPoint;
    SemaphoreHandle_t RingBuffer_mux;
} HCRingbufferT;

esp_err_t RingBufferCreate(HCRingbufferT *Buffer, size_t Size);
int RingBufferGet(HCRingbufferT *Buffer, char *pvItem, size_t mItemSize);
int RingBufferPut(HCRingbufferT *Buffer, const char *pvItem, size_t mItemSize);
int RingBufferRead(HCRingbufferT *Buffer, char *pvItem, size_t mItemSize, bool StartRead);
esp_err_t RingBufferDestroy(HCRingbufferT *Buffer);
