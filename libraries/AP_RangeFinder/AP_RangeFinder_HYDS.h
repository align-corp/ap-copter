#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#ifndef AP_RANGEFINDER_HYDS_ENABLED
#define AP_RANGEFINDER_HYDS_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_HYDS_ENABLED

#define HYDS_DIST_MAX_CM 1200
#define HYDS_DIST_MIN_CM 30

class AP_RangeFinder_HYDS : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_HYDS(_state, _params);
    }
    int16_t max_distance_cm() const override { return HYDS_DIST_MAX_CM; }
    int16_t min_distance_cm() const override { return HYDS_DIST_MIN_CM; }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    enum class AP_RangeFinder_HYDS_State
    {
        WAITING_ADDRESS,
        WAITING_FUNCTION,
        WAITING_LENGTH,
        WAITING_BYTES,
        DECODE
    };
    

    AP_RangeFinder_HYDS_State state;
    uint8_t linebuf[10];
    uint8_t linebuf_len;
};
#endif  // AP_RANGEFINDER_HYDS_ENABLED
