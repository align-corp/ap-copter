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

#include "AP_RangeFinder_HYDS.h"

#if AP_RANGEFINDER_HYDS_ENABLED


#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define FRAME_ADDRESS 0x01
#define FRAME_FUNCTION 0x03
#define PAYLOAD_LENGTH 0x04
#define FRAME_LENGTH 9
#define DIST_MAX_CM 1300
#define OUT_OF_RANGE_ADD_CM 1000

// TODO: format of serial packets received from rangefinder
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Address code    0x01
// byte 1               DIST_H          Distance (in mm) high 8 bits
// byte 2               DIST_L          Distance (in mm) low 8 bits
// byte 3               STATUS          Status,Strengh,OverTemp
// byte 4               CRC8            packet CRC

// distance returned in reading_m, set to true if sensor reports a good reading
bool AP_RangeFinder_HYDS::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    bool distance_received = false;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is full go to WAITING_ADDRESS
        if (linebuf_len == FRAME_LENGTH+1) {
            state = AP_RangeFinder_HYDS_State::WAITING_ADDRESS;
        }
        switch (state)
        {
        case AP_RangeFinder_HYDS_State::WAITING_ADDRESS:
            if (c == FRAME_ADDRESS) {
                linebuf_len = 0;
                linebuf[linebuf_len++] = c;
                state = AP_RangeFinder_HYDS_State::WAITING_FUNCTION;
            }
            break;
        
        case AP_RangeFinder_HYDS_State::WAITING_FUNCTION:
            if (c == FRAME_FUNCTION) {
                linebuf[linebuf_len++] = c;
                state = AP_RangeFinder_HYDS_State::WAITING_LENGTH;
            } else {
                state = AP_RangeFinder_HYDS_State::WAITING_ADDRESS;
            }
            break;
        
        case AP_RangeFinder_HYDS_State::WAITING_LENGTH:
            if (c == PAYLOAD_LENGTH) {
                linebuf[linebuf_len++] = c;
                state = AP_RangeFinder_HYDS_State::DECODE;
            } else {
                state = AP_RangeFinder_HYDS_State::WAITING_ADDRESS;
            }
            break;

        case AP_RangeFinder_HYDS_State::DECODE:
            linebuf[linebuf_len++] = c;
            if (linebuf_len == FRAME_LENGTH) {
                // calculate CRC16
                uint16_t crc_calculated = calc_crc_modbus(linebuf,FRAME_LENGTH-2);
                uint16_t crc = ((uint16_t)linebuf[FRAME_LENGTH-2] << 8) | linebuf[FRAME_LENGTH-1];
                // if crc matches, extract contents
                if (crc_calculated == crc) {
                    // calculate distance
                    uint16_t dist_mm = ((uint16_t)linebuf[3] << 8) | linebuf[4];
                    distance_received = true;
                    if (dist_mm == 0) {
                        // reading out of range
                        reading_m = DIST_MAX_CM + OUT_OF_RANGE_ADD_CM;
                    } else {
                        // reading is good
                        reading_m = dist_mm * 0.001f;
                    }
                } else {
                    // wrong checksum
                    hal.console->printf("Wrong checksum: %u - %u", crc_calculated, crc);
                }
                // go to WAITING_ADDRESS
                state = AP_RangeFinder_HYDS_State::WAITING_ADDRESS;
            }
            break;
        
        default:
            break;
        }
    }

    // ask for distance
    const uint8_t packet[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
    uart->write(packet, 8);

    return distance_received;
}

#endif // AP_RANGEFINDER_HYDS_ENABLED
