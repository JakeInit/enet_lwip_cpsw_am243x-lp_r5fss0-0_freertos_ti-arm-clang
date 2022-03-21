/*
 * parakeetDriver.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#include <string.h>
#include "ParakeetDriver.h"
#include "Parser.h"
#include "UdpSocket.h"
#include "Shared.h"
#include "Vector.h"

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

#define ETHERNET_MESSAGE_DATA_BUFFER_SIZE 8192

void (*scanCallbackFunction)(struct ScanDataPolar*);

void openParakeetSocket();
void ethernetUpdateThreadFunction();
bool isSensorConnected();

void onCompleteLidarMessage(struct CompleteLidarMessage* lidarMessage);
uint32_t calculateEndOfMessageCRC(uint32_t* ptr, uint32_t len);

bool sendMessageWaitForResponseOrTimeout(char* message, uint32_t microsecondsTilTimeout);
bool sendMessageWaitForResponseOrTimeoutCmd(char* message, uint32_t microsecondsTilTimeout, uint16_t cmd);
bool sendUdpMessageWaitForResponseOrTimeout(char* message, char* response, uint32_t timeout_us, uint16_t cmd);

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
struct CmdHeader
{
    unsigned short sign;
    unsigned short cmd;
    unsigned short sn;
    unsigned short len;
};

unsigned char ethernetPortDataBuffer[ETHERNET_MESSAGE_DATA_BUFFER_SIZE];

uint64_t timeOfFirstPoint;
uint64_t updateThreadStartTime_us;
uint16_t updateThreadFrameCount = 0;
bool runUpdateThread = false;

bool memoryAllocatedForPointHoldingList = false;
uint32_t pointHoldingListSize;
Vector(struct PointPolar) pointHoldingList;

struct BufferData bufferData;
struct SensorConfiguration sensorConfiguration;
sys_mutex_t readWrite_mutex;

//---------------------------------------------------------------------------------------------
//--------------------Memory Management--------------------------------------------------------
//---------------------------------------------------------------------------------------------
void clearPointHoldingList()
{
    if(!vectorEmpty(pointHoldingList))
    {
        free(pointHoldingList); // Free all memory in list
    }
}

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
    strncpy(config->ipAddress, ipAddress, (sizeof(ipAddress) / sizeof(char)));
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
    memcpy(bufferData.buffer, ethernetPortDataBuffer, sizeof(ethernetPortDataBuffer));

    err_t err = sys_mutex_new(&readWrite_mutex);
    LWIP_ASSERT("failed to create readWrite_mutex", err == ERR_OK);
    LWIP_UNUSED_ARG(err);
}

void shutdownParakeetDriver()
{
    closeSensorConnection();
}

bool isSensorConnected()
{
    return socketIsOpen();
}

void assertIsConnected()
{
    if(!isSensorConnected())
    {
        DebugP_log("Not connected to sensor.");
        exit(0);
    }
}

void connectSensor(struct SensorConfiguration* newSensorConfiguration)
{
    sensorConfiguration = *newSensorConfiguration;
    openParakeetSocket();
}

void openParakeetSocket()
{
    if(udpSocketOpen(sensorConfiguration.srcPort))
    {
        sendMessageWaitForResponseOrTimeout(CW_STOP_ROTATING, STOP_TIMEOUT_MS);
    }
    else
    {
        DebugP_log("Unable to connect to sensor.");
        exit(0);
    }
}

void closeSensorConnection()
{
    if(isSensorConnected() || runUpdateThread)
    {
        stopSensor();
    }

    udpSocketClose();
}

void startSensor()
{
    bufferData.length = 0;
    resetParser();

    enableIntensityData(sensorConfiguration.intensity);
    enableDataSmoothing(sensorConfiguration.dataSmoothing);
    enableRemoveDragPoint(sensorConfiguration.dragPointRemoval);
    setScanningFrequency_Hz(sensorConfiguration.scanningFrequency_Hz);
    enableResampleFilter(sensorConfiguration.resampleFilter);

    if (!sendMessageWaitForResponseOrTimeout(CW_START_NORMALLY, START_TIMEOUT_MS))
    {
        DebugP_log("No response from sensor. Make sure the sensor is on and communication "
                "is over the correct port.");
        exit(0);
    }

    assertIsConnected();
    runUpdateThread = true;
    updateThreadStartTime_us = ClockP_getTimeUsec();
    updateThreadFrameCount = 0;
    // TODO: start update thread here
}

void stopSensor()
{
    sendMessageWaitForResponseOrTimeout(CW_STOP_ROTATING, STOP_TIMEOUT_MS);
    runUpdateThread = false;
    // End updateThread
}

void enableDataSmoothing(bool enable)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeout(SW_SET_DATA_SMOOTHING(enable), MESSAGE_TIMEOUT_MS);
    sensorConfiguration.dataSmoothing = enable;
}

void enableRemoveDragPoint(bool enable)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeout(SW_SET_DRAG_POINT_REMOVAL(enable), MESSAGE_TIMEOUT_MS);
    sensorConfiguration.dragPointRemoval = enable;
}

void enableIntensityData(bool enable)
{
    sensorConfiguration.intensity = enable;
}

void enableResampleFilter(bool enable)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeout(SW_SET_RESAMPLE_FILTER(enable), MESSAGE_TIMEOUT_MS);
    sensorConfiguration.resampleFilter = enable;
}

void setScanningFrequency_Hz(enum ScanningFrequency Hz)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeout(SW_SET_SPEED(Hz * 60), MESSAGE_TIMEOUT_MS);
    sensorConfiguration.scanningFrequency_Hz = Hz;
}

void setSensorIPv4Settings(uint8_t* ipAddress, uint8_t* subnetMask, uint8_t* gateway, uint16_t port)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeoutCmd(SW_SET_SRC_IPV4_PROPERTIES(ipAddress, subnetMask, gateway, port),
                                           MESSAGE_TIMEOUT_MS, UDP_MESSAGE_SET_PROPERTIES_CMD);
    sensorConfiguration.dstPort = port;
    memcpy(sensorConfiguration.ipAddress, ipAddress, 4 * sizeof(uint8_t));
}

void setSensorDestinationIPv4Settings(uint8_t* ipAddress, uint16_t port)
{
    assertIsConnected();
    sendMessageWaitForResponseOrTimeoutCmd(SW_SET_DST_IPV4_PROPERTIES(ipAddress, port),
                                           MESSAGE_TIMEOUT_MS, UDP_MESSAGE_SET_PROPERTIES_CMD);
    sensorConfiguration.srcPort = port;
}

bool isDataSmoothingEnabled()
{
    assertIsConnected();
    return sensorConfiguration.dataSmoothing;
}

bool isDragPointRemovalEnabled()
{
    assertIsConnected();
    return sensorConfiguration.dragPointRemoval;
}

bool isIntensityDataEnabled()
{
    return sensorConfiguration.intensity;
}

enum ScanningFrequency getScanningFrequency_Hz()
{
    assertIsConnected();
    return sensorConfiguration.scanningFrequency_Hz;
}

bool isResampleFilterEnabled()
{
    assertIsConnected();
    return sensorConfiguration.resampleFilter;
}

void ethernetUpdateThreadFunction()
{
    if(!socketIsOpen())
    {
        return;
    }

    sys_mutex_lock(&readWrite_mutex);
    uint16_t charsRead = udpSocketRead(&bufferData, ETHERNET_MESSAGE_DATA_BUFFER_SIZE);
    sys_mutex_unlock(&readWrite_mutex);

    if(charsRead == 0)
    {
        return;
    }

    bufferData.length += charsRead;

    uint16_t bytesParsed = parse(&bufferData);

    for (uint32_t i = bytesParsed; i < bufferData.length; i++)
    {
        ethernetPortDataBuffer[i - bytesParsed] = ethernetPortDataBuffer[i];
    }
    bufferData.length -= bytesParsed;
}

void onScanDataReceived(struct ScanData* scanData)
{
    // in degrees times 100
    uint32_t anglePerPoint_deg = 100 * (scanData->endAngle_deg - scanData->startAngle_deg) / scanData->count;

    // 1 degree * 100
    uint32_t deviationFrom360_deg = 100;

    // Create PointPolar for each data point
    for(uint32_t i = 0; i < scanData->count; i++)
    {
        struct PointPolar pointPolar;
        pointPolar.range_um = scanData->dist_um[i];
        pointPolar.angle_deg = 100 * scanData->startAngle_deg + (anglePerPoint_deg * i);
        pointPolar.intensity = scanData->intensity[i];

        vectorPushBack(pointHoldingList, pointPolar);
    }

    if(scanData->endAngle_deg + deviationFrom360_deg >= 36000) // 360 * 100
    {
        updateThreadFrameCount++;
        struct ScanDataPolar scanDataPolar;
        vectorCopy(pointHoldingList, scanDataPolar.polarPointsList); // From, To
        scanDataPolar.listSize = scanData->count;
        scanDataPolar.timestamp_us = scanData->timestamp_us;

        scanCallbackFunction(&scanDataPolar);
        clearPointHoldingList();
    }
}

void onCompleteLidarMessage(struct CompleteLidarMessage* lidarMessage)
{
    struct ScanData* scanData = ScanData_New();
    scanData->setTime(scanData, lidarMessage->timestamp_us);
    scanData->count = lidarMessage->totalPoints;
    scanData->startAngle_deg = lidarMessage->startAngle;
    scanData->endAngle_deg = lidarMessage->endAngle;

    for (uint32_t i = 0; i < lidarMessage->totalPoints; i++)
    {
        scanData->dist_um[i] = lidarMessage->lidarPoints[i].distance_um;
        scanData->intensity[i] = lidarMessage->lidarPoints[i].intensity;
    }

    onScanDataReceived(scanData);
}

void registerScanCallback(void (*callback)(struct ScanDataPolar*))
{
    scanCallbackFunction = callback;
}

bool sendMessageWaitForResponseOrTimeoutCmd(char* message, uint32_t microsecondsTilTimeout, uint16_t cmd)
{
    sys_mutex_lock(&readWrite_mutex);
    bool state = sendUdpMessageWaitForResponseOrTimeout(message, "OK", microsecondsTilTimeout, cmd);
    sys_mutex_unlock(&readWrite_mutex);

    return state;
}

bool sendMessageWaitForResponseOrTimeout(char* message, uint32_t microsecondsTilTimeout)
{
    return sendMessageWaitForResponseOrTimeoutCmd(message, microsecondsTilTimeout, UDP_MESSAGE_CMD);
}

bool sendUdpMessageWaitForResponseOrTimeout(char* message, char* response, uint32_t timeout_us, uint16_t cmd)
{
    unsigned char buffer[2048] = {0};
    struct CmdHeader* hdr = (struct CmdHeader*)buffer;
    hdr->sign = UDP_MESSAGE_SIGN;
    hdr->cmd = cmd;
    hdr->sn = rand();

    hdr->len = ((uint16_t) ((sizeof(message) / sizeof(char)) + 3) >> 2) * 4;

    memcpy(buffer + sizeof(struct CmdHeader), message, (sizeof(message) / sizeof(char)));

    uint32_t* pcrc = (uint32_t*)(buffer + sizeof(struct CmdHeader) + hdr->len);
    pcrc[0] = calculateEndOfMessageCRC((uint32_t*)(buffer), hdr->len / 4 + 2);

    struct InetAddress inetAddress;
    memcpy(inetAddress.ipAddress, sensorConfiguration.ipAddress, INET_ADDRSTRLEN);
    inetAddress.port = sensorConfiguration.dstPort;

    struct BufferData newBufferData;
    memcpy(newBufferData.buffer, buffer, sizeof(buffer) / sizeof(unsigned char));
    newBufferData.length = sizeof(struct CmdHeader) + sizeof(pcrc[0]) + hdr->len;

    return udpSocketSendMessage(&inetAddress, &newBufferData, response, timeout_us);
}

uint32_t calculateEndOfMessageCRC(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit, data;
    uint32_t crc32 = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (crc32 & 0x80000000)
            {
                crc32 <<= 1;
                crc32 ^= polynomial;
            }
            else
            {
                crc32 <<= 1;
            }

            if (data & xbit)
            {
                crc32 ^= polynomial;
            }

            xbit >>= 1;
        }
    }
    return crc32;
}
