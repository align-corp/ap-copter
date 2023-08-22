#pragma once

#include "AP_Proximity.h"

#if HAL_PROXIMITY_ENABLED
#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_N10P_TIMEOUT_MS            300    // requests timeout after 0.3 seconds
#define PACKET_SIZE                          108
#define	PACKAGE_POINTS                       16
#define MEDIAN_DEG                           20
#define DIST_SIZE                            120

class AP_Proximity_N10P : public AP_Proximity_Backend_Serial {

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 12.0f; }
    float distance_min() const override { return 0.1f; }

private:

    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(float angle_deg, uint16_t distance_cm);
    uint8_t crc8_N10(const uint8_t *p, uint8_t len);

    // reply related variables
    uint8_t _buffer[PACKET_SIZE]; // buffer where to store data from serial
    uint8_t _buffer_count = 0;
    uint16_t distances[DIST_SIZE];
    uint16_t distances_count = 0;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    uint32_t _last_request_sent_ms;         // system time of last command set
};

#endif // HAL_PROXIMITY_ENABLED
