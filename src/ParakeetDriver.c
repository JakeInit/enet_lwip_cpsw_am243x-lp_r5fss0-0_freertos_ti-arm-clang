/*
 * parakeetDriver.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#include <string.h>
#include "ParakeetDriver.h"
#include "Parser.h"

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

const int ETHERNET_MESSAGE_DATA_BUFFER_SIZE = 8192;

unsigned char ethernetPortDataBuffer[ETHERNET_MESSAGE_DATA_BUFFER_SIZE];

uint64_t timeOfFirstPoint;
uint64_t updateThreadStartTime_us;
int16_t updateThreadFrameCount = 0;
bool runUpdateThread = false;

uint32_t pointHoldingListSize;
struct PointPolar* pointHoldingList;

struct CmdHeader
{
    unsigned short sign;
    unsigned short cmd;
    unsigned short sn;
    unsigned short len;
};

// std::thread updateThread;
void (*updateThreadCallbackFunction)();
void (*scanCallbackFunction)(struct ScanDataPolar*);

void open();
void ethernetUpdateThreadFunction();
bool isConnected();

void onCompleteLidarMessage(struct CompleteLidarMessage* lidarMessage);
uint32_t calculateEndOfMessageCRC(uint32_t* ptr, uint32_t len);

bool sendMessageWaitForResponseOrTimeout(char* message, uint32_t microsecondsTilTimeout);
bool sendMessageWaitForResponseOrTimeout(char* message, uint32_t microsecondsTilTimeout, uint16_t cmd);
bool sendUdpMessageWaitForResponseOrTimeout(char* message, char* response, uint32_t timeout_us, uint16_t cmd);

struct BufferData bufferData;
struct SensorConfiguration sensorConfiguration;

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
void setTime_Now(struct ScanData* scanData)
{
    scanData->timestamp_us = ClockP_getTimeUsec();
}

void setTime(struct ScanData* scanData, uint64_t time_us)
{
    scanData->timestamp_us = time_us;
}

struct ScanData* ScanData_New()
{
    struct ScanData* scanData = malloc(sizeof(*scanData));
    scanData->setTime_Now = setTime_Now;
    scanData->setTime = setTime;
    scanData->timestamp_us = 0;
    scanData->startAngle_deg = 0;
    scanData->endAngle_deg = 0;
    scanData->count = 0;
    scanData->reserved = 0;

    return scanData;
}

struct SensorConfiguration* SensorConfiguration_new(const char* ipAddress, uint16_t dstPort,
                                                    uint16_t srcPort, bool intensity,
                                                    enum ScanningFrequency scanningFrequency_Hz,
                                                    bool dataSmoothing, bool dragPointRemoval,
                                                    bool resampleFilter)
{
    struct SensorConfiguration* config = malloc(sizeof(*config));
    strncpy(config->ipAddress, ipAddress, INET_ADDRSTRLEN);
    config->dstPort = dstPort;
    config->srcPort = srcPort;
    config->intensity = intensity;
    config->scanningFrequency_Hz = scanningFrequency_Hz;
    config->dataSmoothing = dataSmoothing;
    config->dragPointRemoval = dragPointRemoval;
    config->resampleFilter = resampleFilter;

    return config;
}

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
char* SW_SET_SPEED(uint16_t speed)
{
    char* prefix = SW_SET_SPEED_PREFIX; // 6 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[3]; // 2 bytes to hold int and 1 for null termination
    sprintf(numberChar, "%d", speed);

    static char returnChar[10];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_BIAS(uint16_t bias)
{
    char* prefix = SW_SET_BIAS_PREFIX; // 6 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[3]; // 2 bytes to hold int and 1 for null termination
    sprintf(numberChar, "%d", bias);

    static char returnChar[10];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_DATA_SMOOTHING(bool enable)
{
    char* prefix = SW_SET_DATA_SMOOTHING_PREFIX; // 4 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[2]; // 1 bytes to hold uint8_t (bool) and 1 for null termination
    sprintf(numberChar, "%d", enable);

    static char returnChar[7];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_DRAG_POINT_REMOVAL(bool enable)
{
    char* prefix = SW_SET_DRAG_POINT_REMOVAL_PREFIX; // 4 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[2]; // 1 bytes to hold uint8_t (bool) and 1 for null termination
    sprintf(numberChar, "%d", enable);

    static char returnChar[7];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_OUTPUT_UNIT_OF_MEASURE(bool mm)
{
    char* prefix = SW_SET_OUTPUT_UNIT_OF_MEASURE_PREFIX; // 6 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[2]; // 1 bytes to hold uint8_t (bool) and 1 for null termination
    sprintf(numberChar, "%d", mm);

    static char returnChar[9];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_SMOOTH(bool enable)
{
    char* prefix = SW_SET_SMOOTH_PREFIX; // 4 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[2]; // 1 bytes to hold uint8_t (bool) and 1 for null termination
    sprintf(numberChar, "%d", enable);

    static char returnChar[6];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* SW_SET_RESAMPLE_FILTER(bool enable)
{
    char* prefix = SW_SET_RESAMPLE_FILTER_PREFIX; // 8 Bytes
    char* suffix = SW_POSTFIX; // 1 Byte

    char numberChar[2]; // 1 bytes to hold uint8_t (bool) and 1 for null termination
    sprintf(numberChar, "%d", enable);

    static char returnChar[11];
    memset(returnChar, 0, sizeof(returnChar));

    strcpy(returnChar, prefix);
    strcat(returnChar, numberChar);
    strcat(returnChar, suffix);
    return returnChar;
}

char* numberToFixedSizeString(uint16_t value, uint32_t size)
{
    uint16_t uvalue = value;
    static char* result;
    static bool charMemSet = false;

    if(charMemSet)
    {
        result = realloc(result, size * sizeof(char));
        memset(result, 0, size * (sizeof(char)));
    }
    else
    {
        result = malloc(size * sizeof(*result));
        charMemSet = true;
    }

    for(int32_t i = size; i >= 0; i--)
    {
        result[i - 1] = '0' + (uvalue % 10);
        uvalue /= 10;
    }

    return result;
}

char* SW_SET_SRC_IPV4_PROPERTIES(const unsigned char* ipAddress, const unsigned char* subnetMask,
                                 const unsigned char* gateway, const uint16_t port)
{
    static char* result;
    const uint16_t size = 25; // Add sizes from below + 1 for null termination
    static bool charMemSet = false;
    if(charMemSet)
    {
        result = realloc(result, size * sizeof(char));
        memset(result, 0, size * (sizeof(char)));
    }
    else
    {
        result = malloc(size * sizeof(*result));
        charMemSet = true;
    }

    char delimiter = SW_SET_LIDAR_PROPERTIES_DELIMITER;
    char* prefix = SW_SET_SRC_IPV4_PROPERTIES_PREFIX; // 6 Bytes
    strcpy(result, prefix);

    strcat(result, (char*) ipAddress); // IP_ADDRESS_ARRAY_SIZE = 4 Bytes + 1 for delimiter
    strcat(result, &delimiter);

    strcat(result, (char*) subnetMask); // SUBNET_MASK_ARRAY_SIZE = 4 Bytes + 1 for delimiter
    strcat(result, &delimiter);

    strcat(result, (char*) gateway); // GATEWAY_ARRAY_SIZE = 4 Bytes + 1 for delimiter
    strcat(result, &delimiter);

    char numberChar[3]; // 2 bytes to hold int and 1 for null termination
    sprintf(numberChar, "%d", port);
    strcat(result, numberChar); // PORT_STRING_LENGTH = 5 Bytes, but will use size of uint16_t = 2 Bytes

    char* suffix = SW_POSTFIX; // 1 Byte
    strcat(result, suffix);

    return result;
}

char* SW_SET_DST_IPV4_PROPERTIES(const unsigned char* ipAddress, const unsigned short port)
{
    static char* result;
    const uint16_t size = 15; // Add sizes from below + 1 for null termination
    static bool charMemSet = false;
    if(charMemSet)
    {
        result = realloc(result, size * sizeof(char));
        memset(result, 0, size * (sizeof(char)));
    }
    else
    {
        result = malloc(size * sizeof(*result));
        charMemSet = true;
    }

    char delimiter = SW_SET_LIDAR_PROPERTIES_DELIMITER;
    char* prefix = SW_SET_DST_IPV4_PROPERTIES_PREFIX; // 6 Bytes

    strcpy(result, prefix);

    strcat(result, (char*) ipAddress); // IP_ADDRESS_ARRAY_SIZE = 4 Bytes + 1 for delimiter
    strcat(result, &delimiter);

    char numberChar[3]; // 2 bytes to hold int and 1 for null termination
    sprintf(numberChar, "%d", port);
    strcat(result, numberChar); // PORT_STRING_LENGTH = 5 Bytes, but will use size of uint16_t = 2 Bytes

    char* suffix = SW_POSTFIX; // 1 Byte
    strcat(result, suffix);

    return result;
}

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
void initParakeetDriver()
{
    initParser(&onCompleteLidarMessage);
    updateThreadCallbackFunction = ethernetUpdateThreadFunction;
    bufferData.buffer = ethernetPortDataBuffer;
}
