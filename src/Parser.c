/*
 * parser.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#include "Parser.h"
#include "Bool.h"

#include "stdio.h"
#include "string.h"

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

void generateTimestamp();
void parseHeader();
void parseNumPointsInThisPartialSector();
void parseNumPointsInSector();
void parseSectorDataOffset();
void parseStartAngle();
void parseEndAngle();
void parseSensorProperties();
void parseTimestamp();
void parseDeviceNumber();
void parsePoints();
void parseChecksum();
void parsePartialScan();

bool doesChecksumMatch();
uint32_t parseLidarDataFromBuffer();

bool isLidarMessage();
bool isLidarResponse();
bool isAlarmMessage();

uint32_t getTotalPointsOfAllPartialScans();

bool isScanCorrupt();
bool isScanComplete();
void createAndPublishCompleteScan();

// Points to function passed in that has a reference to a complete lidar message as input
// Returns nothing
void (*onCompleteLidarMessageCallback)(struct CompleteLidarMessage* message);

typedef struct PartialLidarMessage Sector;

//----------------------------------------------------------------------------------------------
//----------------Global Variables for Parser---------------------------------------------------
//----------------------------------------------------------------------------------------------
uint16_t header;

struct PartialLidarMessage* currentLidarMessage;

Vector(struct PartialLidarMessage) partialSectorScanDataList;

struct BufferData bufferData;
struct LastGeneratedTimestamp lastGeneratedTimestamp;

const uint32_t FIRST_TIMESTAMP_NULL_VALUE = -1;
const uint32_t TIMESTAMP_RESET_VALUE = 25565;

const uint16_t LIDAR_MESSAGE_HEADER = 0xFAC7;
const uint16_t LIDAR_RESPONSE_HEADER = 0x484C;
const uint16_t ALARM_MESSAGE_HEADER = 0xCECE;

const uint8_t BUFFER_POS_HEADER = 0;
const uint8_t BUFFER_POS_TOTAL_POINTS = 2;
const uint8_t BUFFER_POS_NUM_POINTS_IN_SECTOR = 4;
const uint8_t BUFFER_POS_SECTOR_DATA_OFFSET = 6;
const uint8_t BUFFER_POS_START_ANGLE = 8;
const uint8_t BUFFER_POS_END_ANGLE = 12;
const uint8_t BUFFER_POS_PROPERTY_FLAGS = 16;
const uint8_t BUFFER_POS_TIMESTAMP = 20;
const uint8_t BUFFER_POS_DEVICE_NUMBER = 24;
const uint8_t BUFFER_POS_POINT_DATA = 28;

const uint8_t SIZE_OF_DISTANCE = 2;
const uint8_t SIZE_OF_RELATIVE_START_ANGLE = 2;
const uint8_t SIZE_OF_INTENSITY = 1;
const uint8_t SIZE_OF_LIDAR_POINT = SIZE_OF_DISTANCE + SIZE_OF_RELATIVE_START_ANGLE + SIZE_OF_INTENSITY;

bool parserIsInit = false;
bool memoryAllocatedForCurrentMessage = false;

//---------------------------------------------------------------------------------------------
//--------------------Memory Management--------------------------------------------------------
//---------------------------------------------------------------------------------------------
void clearPartialScanList()
{
    if(!vectorEmpty(partialSectorScanDataList))
    {

        for(Sector* it = vectorBegin(partialSectorScanDataList); it != vectorEnd(partialSectorScanDataList); ++it)
        {
            if(!vectorEmpty(it->lidarPoints))
            {
                vectorFree(it->lidarPoints); // Free all lidar points
            }
        }
        vectorFree(partialSectorScanDataList); // Free all memory in list
    }
}

// Current Lidar Message Memory Management
void mallocCurrentLidarMessage()
{
    if(!memoryAllocatedForCurrentMessage)
    {
        currentLidarMessage = malloc(sizeof(*currentLidarMessage));
        memoryAllocatedForCurrentMessage = true;
    }
}

void clearCurrentLidarMessage()
{
    if(memoryAllocatedForCurrentMessage)
    {
        free(currentLidarMessage->lidarPoints);
        free(currentLidarMessage);
        memoryAllocatedForCurrentMessage = false;
    }
}

//---------------------------------------------------------------------------------------------
//------------------------Parser Functions-----------------------------------------------------
//---------------------------------------------------------------------------------------------
void initParser(void (*message_callback)(struct CompleteLidarMessage *))
{
    onCompleteLidarMessageCallback = message_callback;
    resetParser();
    parserIsInit = true;
}

void resetParser()
{
    lastGeneratedTimestamp.validTimestamp = false;
}

void generateTimestamp()
{
    if (lastGeneratedTimestamp.validTimestamp)
    {
        currentLidarMessage->generatedTimestamp = lastGeneratedTimestamp;
    }

    lastGeneratedTimestamp.validTimestamp = true;
    lastGeneratedTimestamp.timestamp_us = ClockP_getTimeUsec();
}

void parseHeader()
{
    memcpy(&header, bufferData.buffer + BUFFER_POS_HEADER, sizeof(header));
}

void parseNumPointsInThisPartialSector()
{
    memcpy(&currentLidarMessage->numPoints, bufferData.buffer + BUFFER_POS_TOTAL_POINTS, sizeof(currentLidarMessage->numPoints));
}

void parseNumPointsInSector()
{
    memcpy(&currentLidarMessage->numPointsInSector, bufferData.buffer + BUFFER_POS_NUM_POINTS_IN_SECTOR, sizeof(currentLidarMessage->numPointsInSector));
}

void parseSectorDataOffset()
{
    memcpy(&currentLidarMessage->sectorDataOffset, bufferData.buffer + BUFFER_POS_SECTOR_DATA_OFFSET, sizeof(currentLidarMessage->sectorDataOffset));
}

void parseStartAngle()
{
    memcpy(&currentLidarMessage->startAngle, bufferData.buffer + BUFFER_POS_START_ANGLE, sizeof(currentLidarMessage->startAngle));
}

void parseEndAngle()
{
    memcpy(&currentLidarMessage->endAngle, bufferData.buffer + BUFFER_POS_END_ANGLE, sizeof(currentLidarMessage->endAngle));
}

void parseSensorProperties()
{
    memcpy(&currentLidarMessage->sensorPropertyFlags.value, bufferData.buffer + BUFFER_POS_PROPERTY_FLAGS, sizeof(currentLidarMessage->sensorPropertyFlags.value));

    currentLidarMessage->sensorPropertyFlags.unitIsInCM = currentLidarMessage->sensorPropertyFlags.value | 0x1;
    currentLidarMessage->sensorPropertyFlags.withIntensity = currentLidarMessage->sensorPropertyFlags.value | 0x2;
    currentLidarMessage->sensorPropertyFlags.doDragPointRemoval = currentLidarMessage->sensorPropertyFlags.value | 0x4;
    currentLidarMessage->sensorPropertyFlags.doDataSmoothing = currentLidarMessage->sensorPropertyFlags.value | 0x8;
}

void parseTimestamp()
{
    memcpy(&currentLidarMessage->timestamp, bufferData.buffer + BUFFER_POS_TIMESTAMP, sizeof(currentLidarMessage->timestamp));
}

void parseDeviceNumber()
{
    memcpy(&currentLidarMessage->deviceNumber, bufferData.buffer + BUFFER_POS_DEVICE_NUMBER, sizeof(currentLidarMessage->deviceNumber));
}

void parsePoints()
{
    uint16_t BUFFER_POS_DISTANCES = BUFFER_POS_POINT_DATA;
    uint16_t BUFFER_POS_RELATIVE_START_ANGLES = BUFFER_POS_DISTANCES + (currentLidarMessage->numPoints * SIZE_OF_DISTANCE);
    uint16_t BUFFER_POS_INTENSITY = BUFFER_POS_RELATIVE_START_ANGLES + (currentLidarMessage->numPoints * SIZE_OF_RELATIVE_START_ANGLE);

    uint16_t totalPoints = currentLidarMessage->numPoints;
    currentLidarMessage->lidarPoints = malloc(totalPoints*(sizeof(*currentLidarMessage->lidarPoints)));
    for (uint8_t i = 0; i < totalPoints; i++)
    {
        struct LidarPoint lidarPoint;
        uint16_t distance;

        memcpy(&distance, bufferData.buffer + BUFFER_POS_DISTANCES + (i * SIZE_OF_DISTANCE), SIZE_OF_DISTANCE);
        lidarPoint.distance_um = currentLidarMessage->sensorPropertyFlags.unitIsInCM ? distance : distance / 10;

        memcpy(&lidarPoint.relativeStartAngle_deg, bufferData.buffer + BUFFER_POS_RELATIVE_START_ANGLES + (i * SIZE_OF_RELATIVE_START_ANGLE), SIZE_OF_RELATIVE_START_ANGLE);

        if (currentLidarMessage->sensorPropertyFlags.withIntensity)
        {
            memcpy(&lidarPoint.intensity, bufferData.buffer + BUFFER_POS_INTENSITY + (i * SIZE_OF_INTENSITY), SIZE_OF_INTENSITY);
        }
        else
        {
            lidarPoint.intensity = 0;
        }

        currentLidarMessage->lidarPoints[i] = lidarPoint;
    }
}

void parseChecksum()
{
    uint16_t BUFFER_POS_CHECKSUM = BUFFER_POS_POINT_DATA + (SIZE_OF_LIDAR_POINT * currentLidarMessage->numPoints);

    memcpy(&currentLidarMessage->checksum, bufferData.buffer + BUFFER_POS_CHECKSUM, sizeof(currentLidarMessage->checksum));
}

void parsePartialScan()
{
    generateTimestamp();
    parseNumPointsInThisPartialSector();
    parseNumPointsInSector();
    parseSectorDataOffset();
    parseStartAngle();
    parseEndAngle();
    parseSensorProperties();
    parseTimestamp();
    parseDeviceNumber();
    parsePoints();
    parseChecksum();
}

bool doesChecksumMatch()
{
    uint16_t newChecksum = 0;

    newChecksum += currentLidarMessage->numPoints;
    newChecksum += currentLidarMessage->numPointsInSector;
    newChecksum += currentLidarMessage->sectorDataOffset;

    newChecksum += currentLidarMessage->startAngle >> 16;
    newChecksum += currentLidarMessage->startAngle & 0xFFFF;

    newChecksum += currentLidarMessage->endAngle >> 16;
    newChecksum += currentLidarMessage->endAngle & 0xFFFF;

    newChecksum += currentLidarMessage->sensorPropertyFlags.value >> 16;
    newChecksum += currentLidarMessage->sensorPropertyFlags.value & 0xFFFF;

    newChecksum += currentLidarMessage->timestamp >> 16;
    newChecksum += currentLidarMessage->timestamp & 0xFFFF;

    newChecksum += currentLidarMessage->deviceNumber >> 16;
    newChecksum += currentLidarMessage->deviceNumber & 0xFFFF;

    for(uint64_t i = 0; i < currentLidarMessage->numPoints; i++)
    {
        struct LidarPoint lidarPoint = currentLidarMessage->lidarPoints[i];
        newChecksum += lidarPoint.distance_um;
        newChecksum += lidarPoint.relativeStartAngle_deg;
        newChecksum += lidarPoint.intensity;
    }

    return newChecksum == currentLidarMessage->checksum;
}

uint32_t getTotalPointsOfAllPartialScans()
{
    uint32_t totalPoints = 0;
    for(Sector* it = vectorBegin(partialSectorScanDataList); it != vectorEnd(partialSectorScanDataList); ++it)
    {
        totalPoints += it->numPoints;
    }
    return totalPoints;
}

bool isScanComplete()
{
    return getTotalPointsOfAllPartialScans() == currentLidarMessage->numPointsInSector;
}

void createAndPublishCompleteScan()
{
    if(vectorEmpty(partialSectorScanDataList))
    {
        return;
    }

    struct PartialLidarMessage firstMessage = partialSectorScanDataList[0];
    // We will not ship the information from the first revolution, as it will not have a valid timestamp.
    if (!firstMessage.generatedTimestamp.validTimestamp)
    {
        clearPartialScanList();
        return;
    }

    struct CompleteLidarMessage lidarMessage;

    lidarMessage.totalPoints = firstMessage.numPointsInSector;
    lidarMessage.endAngle = firstMessage.deviceNumber;
    lidarMessage.startAngle = firstMessage.startAngle / 1000;
    lidarMessage.endAngle = firstMessage.endAngle / 1000;
    lidarMessage.timestamp_us = firstMessage.generatedTimestamp.timestamp_us;
    lidarMessage.sensorPropertyFlags = firstMessage.sensorPropertyFlags;

    // Add all points in sectors to lidarPoints in lidarMessage
    for(Sector* it = vectorBegin(partialSectorScanDataList); it != vectorEnd(partialSectorScanDataList); ++it)
    {
        for(uint64_t j = 0; j < it->numPoints; j++)
        {
            struct LidarPoint lidarPoint = it->lidarPoints[j];
            vectorPushBack(lidarMessage.lidarPoints, lidarPoint);
        }
    }

    onCompleteLidarMessageCallback(&lidarMessage);
    clearPartialScanList();
}

uint32_t parseLidarDataFromBuffer()
{
    clearCurrentLidarMessage();
    mallocCurrentLidarMessage();
    parsePartialScan(); // Fill in currentLidarMessage
    vectorPushBack(partialSectorScanDataList, *currentLidarMessage);

    if(isScanCorrupt())
    {
        clearPartialScanList();
        vectorPushBack(partialSectorScanDataList, *currentLidarMessage);
    }

    if (isScanComplete())
    {
        createAndPublishCompleteScan();
    }

    return bufferData.length;
}

bool isLidarMessage()
{
    return header == LIDAR_MESSAGE_HEADER;
}

bool isLidarResponse()
{
    return header == LIDAR_MESSAGE_HEADER;
}

bool isAlarmMessage()
{
    return header == ALARM_MESSAGE_HEADER;
}

uint32_t parse(const struct BufferData* newBufferData)
{
    if(parserIsInit)
    {
        DebugP_log("\nParser is not yet initialized");
        return 0;
    }

    bufferData = *newBufferData;

    parseHeader();

    if (isLidarMessage())
    {
        return parseLidarDataFromBuffer();
    }
    else if (isLidarResponse())
    {
        return bufferData.length;
    }
    else if (isAlarmMessage())
    {
        DebugP_log("An alarm has been triggered");
        exit(0);
    }
    else
    {
        DebugP_log("Failure to parse data from buffer");
        return bufferData.length;
    }
}
