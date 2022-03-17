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

#define MAX_NUMBER_OF_POINTS_FROM_SENSOR 1000          // Arbitrary size

const uint16_t START_TIMEOUT_MS = 10000;
const uint16_t MESSAGE_TIMEOUT_MS = 2000;
const uint16_t STOP_TIMEOUT_MS = 1000;

const uint8_t IP_ADDRESS_ARRAY_SIZE = 4;
const uint8_t SUBNET_MASK_ARRAY_SIZE = 4;
const uint8_t GATEWAY_ARRAY_SIZE = 4;
const uint8_t IP_ADDRESS_STRING_LENGTH = 3;
const uint8_t PORT_STRING_LENGTH = 5;

const uint16_t UDP_MESSAGE_SIGN = 0x484C;
const uint16_t UDP_MESSAGE_CMD = 0x0043;
const uint16_t UDP_MESSAGE_SET_PROPERTIES_CMD = 0x0053;

char CW_STOP_ROTATING[7] = "LSTOPH";
char CW_START_NORMALLY[7] = "LSTARH";
char CW_STOP_ROTATING_FIX_DIST[7] = "LMEASH";
char CW_RESET_AND_RESTART[7] = "LRESTH";
char CW_VERSION_NUMBER[7] = "LVERSH";

char CW_REQUEST_OUTPUT_TO_SCAN_THE_CLOUD_POINT[7] = "LGCPSH";
char CW_DISABLE_OUTPUT_SCAN_CLOUD_POINTS[7] = "LTCPSH";
char CW_SAVE_FIRMWARE_TO_FLASH[7] = "LSFAPH";
char CW_SAVE_STRATEGY_0_TO_FLASH[7] = "LSPZ0H";
char CW_SAVE_STRATEGY_1_TO_FLASH[7] = "LSPZ1H";

const unsigned char CW_DATA_UNIT_ACQUISITION[7] = "LSMMDH";

char SW_SET_SPEED_PREFIX[7] = "LSRPM:";
char SW_SET_BIAS_PREFIX[7] = "LSERR:";
char SW_SET_SRC_IPV4_PROPERTIES_PREFIX[7] = "LSUDP:";
char SW_SET_DST_IPV4_PROPERTIES_PREFIX[7] = "LSDST:";
char SW_SET_OUTPUT_UNIT_OF_MEASURE_PREFIX[7] = "LSMMU:";

char SW_SET_DATA_SMOOTHING_PREFIX[5] = "LSSS";
char SW_SET_DRAG_POINT_REMOVAL_PREFIX[5] = "LFFF";
char SW_SET_SMOOTH_PREFIX[5] = "LFFF";
char SW_SET_RESAMPLE_FILTER_PREFIX[9] = "LSRES:00";

char SW_POSTFIX[2] = "H";

char SW_SET_LIDAR_PROPERTIES_DELIMITER = ' ';

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
    uint16_t dstPort;
    uint16_t srcPort;
    bool intensity;
    bool dataSmoothing;
    bool dragPointRemoval;
    bool resampleFilter;
    enum ScanningFrequency scanningFrequency_Hz;
};

struct ScanData
{
    void (*setTime_Now)(struct ScanData*);
    void (*setTime)(struct ScanData*, uint64_t);
    uint64_t timestamp_us;
    uint32_t startAngle_deg;
    uint32_t endAngle_deg;
    uint16_t count;
    uint16_t reserved;
    uint32_t dist_um[MAX_NUMBER_OF_POINTS_FROM_SENSOR];
    unsigned char intensity[MAX_NUMBER_OF_POINTS_FROM_SENSOR];
};

struct ScanData* ScanData_New();

/// \brief Create a SensorConfiguration object with the following settings
/// \param[in] intensity - Should the sensor return intensity data
/// \param[in] scanningFrequency_Hz - The speed which the sensor should be spinning at
/// \param[in] dataSmoothing - Should data smoothing be enabled
/// \param[in] dragPointRemoval - Should drag point removal be enabled
struct SensorConfiguration* SensorConfiguration_new(const char* ipAddress, uint16_t dstPort,
                                                    uint16_t srcPort, bool intensity,
                                                    enum ScanningFrequency scanningFrequency_Hz,
                                                    bool dataSmoothing, bool dragPointRemoval,
                                                    bool resampleFilter);

// start parser and register update thread callback
void initParakeetDriver();

/// \brief Attempt connection to a Parakeet sensor through a ethernet port
/// \param[in] sensorConfiguration - Sensor settings and ethernet port information
void connectSensor(const struct SensorConfiguration* sensorConfiguration);

/// \brief Start the Driver's processing thread
void startSensor();

/// \brief Stop the Driver's processing thread
void stopSensor();

/// \brief Close the ethernet connection
void closeSensor();

/// \brief Gets the scan rate from the sensor
/// \returns The scan rate
uint16_t getScanRate_Hz();

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

/// \brief Set a function to be called when scan data is received from the sensor
/// \param[in] callback - The function to be called when data is received
void registerScanCallback(void (*callback)(struct ScanDataPolar*));

void registerUpdateThreadCallback(void (*callback)());

void assertIsConnected();

bool isConnected();

bool isRunning();

void onScanDataReceived(struct ScanData* scanData, uint32_t scanSize);

#endif /* SRC_PARAKEETDRIVER_H_ */
