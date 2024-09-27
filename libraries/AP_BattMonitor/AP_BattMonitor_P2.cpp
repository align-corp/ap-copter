#include "AP_BattMonitor_config.h"

#if AP_BATTERY_P2_ENABLED

#include "AP_BattMonitor_P2.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#if !defined(HAL_BUILD_AP_PERIPH)
#include <AP_Arming/AP_Arming.h>
#endif

#define P2_NOT_HEALTY_MICROS 5000000

#ifdef AP_BATTERY_P2_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_P2::read()
{
    const uint32_t tnow = AP_HAL::micros();

    if (_uart != nullptr)
    {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PCU, 0);
        if (_uart != nullptr) {
            _uart->begin(115200);
            debug("P2: started");
        }
        return;
    }

    // Receive battery informations from P2
    if (parse_message()) {
        // Healthy if messages are received
        _state.healthy = true;

        // get voltage
        _state.voltage = _volt;

        // get temp
        _state.temperature = _temp;

        // calculate time since last current read
        const uint32_t dt_us = tnow - _state.last_time_micros;
        // read current
        _state.current_amps = _amp;
        update_consumed(_state, dt_us);
        // record time
        _state.last_time_micros = tnow;
    }
    // Not healthy if no messages for 5 seconds
    else if (((tnow - _state.last_time_micros) > P2_NOT_HEALTY_MICROS)){
        _state.healthy = false;
    }
}

bool AP_BattMonitor_P2::parse_message()
{
    if (_uart == nullptr) {
        return false;
    }

    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);

    if (nbytes <= 0 ) {
        return false;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // true if a packet was successfully parsed
    bool parsed = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = _uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER:
            if (b == AP_BATT_P2_HEADER) {
                _parsed_msg.state = ParseState::WAITING_FOR_OPERATION;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_OPERATION:
            if (b == AP_BATT_P2_OPERATION_BROADCAST) {
                _parsed_msg.state = ParseState::WAITING_FOR_LENGTH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_LENGTH:
            _parsed_msg.data_len = b;
            if (_parsed_msg.data_len >= AP_BATT_P2_LENGTH_FUEL) {
                _parsed_msg.state = ParseState::WAITING_FOR_SENDER_ID;
            } else {
                reset_parser = true;
                debug("error: data len received: %u", (unsigned)_parsed_msg.data_len);
            }
            break;

        case ParseState::WAITING_FOR_SENDER_ID:
            if (b == AP_BATT_P2_SENDER_P2) {
                _parsed_msg.sender_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_RECEIVER_ID;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_RECEIVER_ID:
                _parsed_msg.receiver_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_PACKET_ID;
            break;

        case ParseState::WAITING_FOR_PACKET_ID:
                _parsed_msg.packet_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
            break;

        case ParseState::WAITING_FOR_PAYLOAD:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                _parsed_msg.checksum = 0;
                _parsed_msg.checksum_bit = 0;
                _parsed_msg.state = ParseState::WAITING_FOR_CRC;
            }
            break;

        case ParseState::WAITING_FOR_CRC:
            _parsed_msg.checksum = _parsed_msg.checksum + (b << (_parsed_msg.checksum_bit*8));
            _parsed_msg.checksum_bit++;
            if (_parsed_msg.checksum_bit == 4) {
                uint32_t checksum = 0;
                checksum = crc_crc32(checksum, _msg_buff, _parsed_msg.data_len+5, 1);
                if (checksum == _parsed_msg.checksum) {
                    switch (_parsed_msg.packet_id) {
                        case AP_BATT_P2_PACKET_ID_BATT:
                            parse_batt();
                            parsed = true;
                            break;
                        case AP_BATT_P2_PACKET_ID_FUEL:
                            parse_fuel();
                            break;
                        default:
                            debug("unknown packet id:%x", _parsed_msg.packet_id);
                            break;
                    }
                } else { 
                    debug("checksum expected:%x, got:%x", checksum, _parsed_msg.checksum);
                }
                reset_parser = true;
            }
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            _parsed_msg.data_bytes_received = 0;
        }
    }
    return parsed;
}

void AP_BattMonitor_P2::parse_batt()
{
    _volt = UINT16_VALUE(_msg_buff[6], _msg_buff[7]) * 0.1f; // 3s voltage
    _volt = UINT16_VALUE(_msg_buff[8], _msg_buff[9]) * 0.1f;
    _amp = UINT16_VALUE(_msg_buff[20], _msg_buff[21]) * 0.1f;
    _temp = (static_cast<float>(static_cast<int16_t>(((_msg_buff[25]) << 8) | (_msg_buff[24]))));
}


void AP_BattMonitor_P2::parse_fuel()
{
    //TODO
}

#endif
