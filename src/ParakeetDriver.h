/*
 * parakeetDriver.h
 *
 *  Created on: Mar 11, 2022
 *      Author: root
 */

#ifndef SRC_PARAKEETDRIVER_H_
#define SRC_PARAKEETDRIVER_H_

#include <stdio.h>
#include <stdarg.h>

#include "lwip/inet.h"

#include "Parser.h"
#include "Bool.h"

/// \brief All currently supported Scanning Frequencies
enum ScanningFrequency
{
    Frequency_7Hz = 7,
    Frequency_10Hz = 10,
    Frequency_15Hz = 15
};

struct SensorConfiguration
{
    char ipAddress[INET_ADDRSTRLEN];
    int dstPort;
    int srcPort;
    bool intensity;
    bool dataSmoothing;
    bool dragPointRemoval;
    bool resampleFilter;
    enum ScanningFrequency scanningFrequency_Hz;
};

/// \brief Create a SensorConfiguration object with the following settings
/// \param[in] intensity - Should the sensor return intensity data
/// \param[in] scanningFrequency_Hz - The speed which the sensor should be spinning at
/// \param[in] dataSmoothing - Should data smoothing be enabled
/// \param[in] dragPointRemoval - Should drag point removal be enabled
struct SensorConfiguration* SensorConfiguration_new(const char* ipAddress, int dstPort,
                                                    int srcPort, bool intensity,
                                                    enum ScanningFrequency scanningFrequency_Hz,
                                                    bool dataSmoothing, bool dragPointRemoval,
                                                    bool resampleFilter);

/// \brief Attempt connection to a Parakeet sensor through a ethernet port
/// \param[in] sensorConfiguration - Sensor settings and ethernet port information
void connectSensor(const struct SensorConfiguration* sensorConfiguration);

/// \brief Start the Driver's processing thread
void startSensor();

/// \brief Stop the Driver's processing thread
void stopSensor();

/// \brief Close the ethernet connection
void closeSensor();

/// \brief Set the scanning frequency on the sensor
/// \param[in] Hz - The scanning frequency to be set
void setScanningFrequency_Hz(enum ScanningFrequency Hz);

/// \brief Gets the scanning frequency
/// \returns The scanning frequency
enum ScanningFrequency getScanningFrequency_Hz();

/// \brief Set the state of intensity data on the sensor
/// \param[in] enable - The state of intensity data
void enableIntensityData(bool enable);

/// \brief Gets the state of intensity data
/// \returns The state of intensity data
bool isIntensityDataEnabled();

/// \brief Set the state of data smoothing on the sensor
/// \param[in] enable - The state of data smoothing
void enableDataSmoothing(bool enable);

/// \brief Gets the state of data smoothing
/// \returns The state of data smoothing
bool isDataSmoothingEnabled();

/// \brief Set the state of drag point removal on the sensor
/// \param[in] enable - The state of drag point removal
void enableRemoveDragPoint(bool enable);

/// \brief Gets the state of drag point removal
/// \returns The state of drag point removal
bool isDragPointRemovalEnabled();

/// \brief Set the state of the resample filter on the sensor
/// \param[in] enable - The state of the resample filter
void enableResampleFilter(bool enable);

#endif /* SRC_PARAKEETDRIVER_H_ */
