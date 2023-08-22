/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_N10P.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>
#include <AP_Common/sorting.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_N10P::update(void)
{
    if (_uart == nullptr) {
        return;
    }
    if (_last_request_sent_ms == 0) {
        _last_request_sent_ms = AP_HAL::millis();
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_N10P_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_N10P::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t nbytes = _uart->available();
    bool message_parsed = false;

    while (nbytes-- > 0) {
        uint8_t byte = _uart->read();
        if (byte == 0xA5 ) {
            _buffer_count = 0;
        }
            
        _buffer[_buffer_count++] = byte;

        if (_buffer_count == PACKET_SIZE){
            _buffer_count = 0;
            //check if message has right CRC and update measure STILL NEED TO DECODE PROTOCOL
            if (crc8_N10(_buffer, PACKET_SIZE-1) == _buffer[PACKET_SIZE-1]){
                // decode message
                float angle_start = UINT16_VALUE(_buffer[5],  _buffer[6]) * 0.01f;
                float angle_stop = UINT16_VALUE(_buffer[105],  _buffer[106]) * 0.01f;
                float angle_step = (angle_stop - angle_start > 0) ?  (angle_stop - angle_start) / PACKAGE_POINTS : (angle_stop+360 - angle_start) / PACKAGE_POINTS;
                float angle = angle_start;

                // Calculate and update distances
                for (uint8_t i = 7; i < 96; i=i+6) {
                    uint16_t distance = UINT16_VALUE(_buffer[i],  _buffer[i+1]);
                    angle = (angle > 360) ? (angle - 360) : angle;
                    update_sector_data(angle, distance);
                    angle = angle + angle_step;
                }
                message_parsed = true;
            } else {
                // wrong crc
            }
        }
    }
    return message_parsed; 
}

// process reply
void AP_Proximity_N10P::update_sector_data(float angle_deg, uint16_t distance_mm)
{
    // Median filter every MEDIAN_DEG = 20
    if (distances_count > DIST_SIZE-1) {
        distances_count = 0;
    }
    if (angle_deg > 337.5) {
        angle_deg -= 360;
    }
    uint8_t sector = UINT8_MAX;

    // understand in which sector we are
    for (uint8_t i=0; i<8; i++) {
        if ((angle_deg > -21 + i*45) && (angle_deg < 21 + i*45)) {
            sector = i;
        }
    }

    // static variable for last sector sent
    static uint8_t last_sector = sector;

    // abort if the angle is in the middle of two sectors
    if (sector == UINT8_MAX) {
        return;
    }

    if (sector == last_sector ) {
        // update sector distances vector
        distances[distances_count++] = distance_mm;
    } else if ((sector == last_sector + 1) || ((sector == 0) && (last_sector == 7))) {
        // moved to next sector, push the sector distance
        insertion_sort_uint16(distances,distances_count);
        uint32_t filtered_distance_mm = 0;
        uint8_t filtered_distance_count = 0;

        // Average minimum distances
        for (uint8_t i = 0; i < distances_count*0.2; i++) {
            filtered_distance_mm += distances[i];
            filtered_distance_count++;
        }
        filtered_distance_mm /= filtered_distance_count;
        float angle_deg_sector = last_sector * 45.0f;

        // update count variables
        distances_count = 0;
        distances[distances_count++] = distance_mm;
        last_sector = sector;

        // Get location on 3-D boundary based on angle to the object
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg_sector);
        //check for target too far, target too close and sensor not connected
        const bool valid = (filtered_distance_mm < distance_max()*1000) && (filtered_distance_mm > distance_min()*1000);
        if (valid && !ignore_reading(angle_deg_sector, filtered_distance_mm * 0.001f, false)) {
            frontend.boundary.set_face_attributes(face, angle_deg_sector, filtered_distance_mm * 0.001f, state.instance);
            // update OA database
            database_push(angle_deg_sector, filtered_distance_mm * 0.001f);
        } else {
            frontend.boundary.reset_face(face, state.instance);
        }
        _last_distance_received_ms = AP_HAL::millis();
    } else {
        // update count variables
        distances_count = 0;
        distances[distances_count++] = distance_mm;
        last_sector = sector;
    }
}

uint8_t AP_Proximity_N10P::crc8_N10(const uint8_t *p, uint8_t len)
	{
		uint16_t sum = 0;
		for (int i = 0; i < len; i++) {
			sum += p[i];
        }
		uint8_t crc = sum & 0xff;
		return crc;
	}

#endif // HAL_PROXIMITY_ENABLED
