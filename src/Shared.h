/*
 * Shared.h
 *
 *  Created on: Mar 14, 2022
 *      Author: Jacob Morgan
 */

#ifndef SRC_SHARED_H_
#define SRC_SHARED_H_

#include "lwip/inet.h"

#include "Bool.h"
#include "Vector.h"

#ifndef MAX_BUFFER_LENGTH
#define MAX_BUFFER_LENGTH 8192
#endif

// UDP Structs
struct BufferData
{
    char* buffer;
    unsigned int length;
};

struct InetAddress
{
    char ipAddress[INET_ADDRSTRLEN];
    unsigned short port;
};

// Parser Structs
struct LidarSensorProperties
{
    bool unitIsInCM;
    bool withIntensity;
    bool doDragPointRemoval;
    bool doDataSmoothing;

    uint32_t value;
};

struct LidarPoint
{
    uint16_t distance_um;
    uint16_t relativeStartAngle_deg;
    uint8_t intensity;
};

struct LastGeneratedTimestamp
{
    bool validTimestamp;
    uint64_t timestamp_us;
};

struct CompleteLidarMessage
{
    uint16_t totalPoints;
    uint32_t startAngle;
    uint32_t endAngle;

    struct LidarSensorProperties sensorPropertyFlags;

    uint64_t timestamp_us;
    uint32_t deviceNumber;

    Vector(struct LidarPoint) lidarPoints;
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

    Vector(struct LidarPoint) lidarPoints;

    uint16_t checksum;
};

// Polar Structs
struct PointPolar
{
    uint32_t range_um;
    uint32_t angle_deg; // units in degrees times 100
    uint16_t intensity;
};

struct ScanDataPolar
{
    Vector(struct PointPolar) polarPointsList;
    uint32_t listSize;
    uint64_t timestamp_us;
};

struct DataPoint
{
    uint32_t angle_deg;
    uint32_t distance_um;
    unsigned char confidence;
};

#endif /* SRC_SHARED_H_ */
