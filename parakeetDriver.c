/*
 * parakeetDriver.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#include "parakeetDriver.h"

#include <string.h>

const int ETHERNET_MESSAGE_DATA_BUFFER_SIZE = 8192;

void open();
void ethernetUpdateThreadFunction();
Bool isConnected();

void onCompleteLidarMessage(const struct CompleteLidarMessage* lidarMessage);

struct SensorConfiguration* SensorConfiguration_new(const char* ipAddress, int dstPort,
                                                    int srcPort, Bool intensity,
                                                    enum ScanningFrequency scanningFrequency_Hz,
                                                    Bool dataSmoothing, Bool dragPointRemoval,
                                                    Bool resampleFilter)
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


