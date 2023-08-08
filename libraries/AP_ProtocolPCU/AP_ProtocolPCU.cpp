#include "AP_ProtocolPCU.h"

#if HAL_PCU_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

void AP_ProtocolPCU::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PCU, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance));
        _init = true;
    }
}

uint32_t AP_ProtocolPCU::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_PCU, serial_instance);
}

/*
   detect if a PCU is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_ProtocolPCU::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_PCU, serial_instance);
}

// Return landing gear PWM value
uint16_t AP_ProtocolPCU::get_landing_gear()
{
    uint16_t lgr_pwm;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_landing_gear_control, lgr_pwm)) {
        return lgr_pwm;
    } else {
        return 0;
    }
}

void AP_ProtocolPCU::set_volt(uint16_t voltage) 
{
    if (voltage < 600) {
        _volts = voltage*0.1f;
    }
}

void AP_ProtocolPCU::set_amps(uint16_t ampere) 
{
    if (ampere < 2000) {
        _amps = ampere*0.1f;
    }
}

void AP_ProtocolPCU::set_temp(uint16_t temperature) 
{
    _temp = temperature*0.1f;
}

bool AP_ProtocolPCU::send_message()
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return false;
    }

    // Get int to send
    uint16_t pwm_LGR = get_landing_gear();

    // Create the serialized message
    bool pre_arm_checks = AP::arming().pre_arm_checks(false);
    _message_to_send[6] = pre_arm_checks ? 1 : 0;    // LED green
    _message_to_send[7] = pre_arm_checks ? 0 : 1;    // LED red
    _message_to_send[8] = static_cast<uint8_t>(pwm_LGR & 0xFF);
    _message_to_send[9] = static_cast<uint8_t>((pwm_LGR >> 8) & 0xFF);

    // Calculate the checksum
    uint8_t checksum = DEVICE_ID ^ static_cast<uint8_t>(Command::PCU_COMMAND);
    for (uint8_t i=4; i<11; i++) {
        checksum ^= _message_to_send[i];
    }
    checksum ^= 0x21;

    // Add checksum
    _message_to_send[1] = checksum;

    // Write the message to the Serial port
    uart->write((const uint8_t*)_message_to_send, sizeof(_message_to_send));
    return true;
}

bool AP_ProtocolPCU::parse_message()
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t packetLength = 12;
    uint8_t packet_start;
    bool go_out;
    bool parse_success = false;
    uint16_t nbytes = uart->available();

    while (nbytes--) {
        go_out = false;
        packet_start = 99;

        // Append incoming byte to message buffer
        _message_buffer[_message_buffer_pos] = uart->read();
        _message_buffer_pos++;

        // Clear buffer if after 30 bytes there are still no useful packet
        if (_message_buffer_pos >= (sizeof(_message_buffer)-1)) {
            _message_buffer_pos = 0;
            go_out = true;
        }

        if ((_message_buffer_pos >= packetLength) && !go_out) {
            // search for the header byte
            for (uint8_t i = 0; i < _message_buffer_pos-2; i++) {
                if ((_message_buffer[i] == HEADER) && (_message_buffer[i+2] == DEVICE_ID)) {
                    // check if there's enough bytes to form a packet
                    if (_message_buffer_pos - i < packetLength) {
                        // not enough bytes, wait for more
                        go_out = true;
                        break;
                    }
                    // potential packet found
                    packet_start = i;
                    break;
                }
            }

            // Go out if no potential packet is found
            go_out = go_out || (packet_start == 99);

            // Check the checksum
            if (!go_out){
                uint8_t checksum = DEVICE_ID ^ _message_buffer[3+packet_start] ^ _message_buffer[4+packet_start] ^ _message_buffer[5+packet_start];
                for (uint8_t i = 6; i < packetLength; i++) {
                    checksum ^= _message_buffer[i+packet_start];
                }
                checksum ^= 0x21;

                if (_message_buffer[1+packet_start] != checksum) {
                    go_out = true;
                    // Clear buffer
                    _message_buffer_pos = 0;
                } else {
                    // deserialize packet
                    set_volt((static_cast<uint8_t>(_message_buffer[packet_start+6]) & 0xFF) |
                                ((static_cast<uint8_t>(_message_buffer[1+packet_start+6]) << 8) & 0xFF00));
                    set_amps((static_cast<uint8_t>(_message_buffer[2+packet_start+6]) & 0xFF) |
                                ((static_cast<uint8_t>(_message_buffer[3+packet_start+6]) << 8) & 0xFF00));
                    set_temp((static_cast<uint8_t>(_message_buffer[4+packet_start+6]) & 0xFF) |
                                ((static_cast<uint8_t>(_message_buffer[5+packet_start+6]) << 8) & 0xFF00));

                    // Clear buffer
                    _message_buffer_pos = 0;
                    parse_success = true;
                }
            }
        }
    }
    return parse_success;
}

#endif