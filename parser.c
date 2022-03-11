/*
 * parser.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#include "parser.h"

struct LastGeneratedTimestamp
{
    Bool validTimestamp;
    uint64_t timestamp;
};

struct PartialLidarMessage
{
    uint16_t numPoints;
    uint16_t numPointsInSector;
    uint16_t sectorDataOffset;

    uint32_t startAngle;
    uint32_t endAngle;

    struct LidarSensorProperties sensorPropertyFlags;

    struct LastGeneratedTimestamp generatedTimestamp;
    uint32_t timestamp;
    uint32_t deviceNumber;

    uint32_t lidarPointsSize;
    struct LidarPoint* lidarPoints;

    uint16_t checksum;
};

void generateTimestamp();
void parseHeader();
void parsePartialScan();
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

int parseLidarDataFromBuffer();
Bool doesChecksumMatch();

Bool isLidarMessage();
Bool isLidarResponse();
Bool isAlarmMessage();

int getTotalPointsOfAllPartialScans();

Bool isScanCorrupt();
Bool isScanComplete();
void createAndPublishCompleteScan();

uint16_t header;
struct PartialLidarMessage* currentLidarMessage;
struct PartialLidarMessage** partialSectorScanDataList;

struct BufferData bufferData;
struct LastGeneratedTimestamp lastGeneratedTimestamp;
