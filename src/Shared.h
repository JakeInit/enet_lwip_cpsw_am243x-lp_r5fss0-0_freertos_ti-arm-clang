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

#ifndef MAX_BUFFER_LENGTH
#define MAX_BUFFER_LENGTH 8192
#endif

struct BufferData
{
    char buffer[MAX_BUFFER_LENGTH];
    unsigned int length;
};

struct InetAddress
{
    char ipAddress[INET_ADDRSTRLEN];
    unsigned short port;
};

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

    uint64_t timestamp_us;
    uint32_t deviceNumber;

    uint32_t numberOfLidarPoints;
    struct LidarPoint* lidarPoints;
};

//void lidarMessage_set_status_callback(struct CompleteLidarMessage* completeMessage,
//                                      Completed_Message_Callback_fn message_callback);
//
//void lidarMessage_set_remove_callback(struct CompleteLidarMessage *completeMessage,
//                                      Completed_Message_Callback_fn remove_callback);

#endif /* SRC_SHARED_H_ */
