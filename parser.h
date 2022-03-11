/*
 * parser.h
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <stdio.h>
#include <stdarg.h>

#include "Bool.h"
#include "UdpSocket.h"

struct LidarSensorProperties
{
    Bool unitIsInCM;
    Bool withIntensity;
    Bool doDragPointRemoval;
    Bool doDataSmoothing;

    uint32_t value;
};

struct LidarPoint
{
    uint16_t distance;
    uint16_t relativeStartAngle;
    uint8_t intensity;
};

struct CompleteLidarMessage
{
    uint16_t totalPoints;
    uint32_t startAngle;
    uint32_t endAngle;

    struct LidarSensorProperties sensorPropertyFlags;

    uint64_t timestamp;
    uint32_t deviceNumber;

    uint32_t numberOfLidarPoints;
    struct LidarPoint* lidarPoints;
};

void onCompleteLidarMessageCallback(struct CompleteLidarMessage* message);

int parse(const struct BufferData* bufferData);

void reset();

#endif /* PARSER_H_ */
