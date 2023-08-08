#pragma once
#include <AP_HAL/AP_HAL.h>

#ifndef HAL_PCU_ENABLED
#define HAL_PCU_ENABLED !HAL_MINIMIZE_FEATURES && (BOARD_FLASH_SIZE > 1024) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if HAL_PCU_ENABLED

#include <cstdint>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Arming/AP_Arming.h>

class AP_ProtocolPCU {
public:
    static const uint8_t HEADER = 0x3F;
    static const uint8_t DEVICE_ID = 0xAA;
    static const uint8_t MESSAGE_MIN_SIZE = 6;

    enum class Command : uint8_t {
        PCU_COMMAND = 0x3E,
        // Add more commands here
    };

    // Constructor
    AP_ProtocolPCU() { /*Do nothing*/ }

    // Serial
    void init_serial(uint8_t serial_instance);
    uint32_t initial_baudrate(const uint8_t serial_instance) const;
    bool detect(uint8_t serial_instance);
    bool is_initialized() {return _init;}

    // Return functions
    float get_volt() {return _volts;} ;
    float get_amps() {return _amps;} ;
    uint16_t get_landing_gear();

    // Set functions
    void set_volt(uint16_t voltage);
    void set_amps(uint16_t ampere);
    void set_temp(uint16_t temperature);

    // Serialize the message, call at 10 Hz
    bool send_message();

    // Receive and parse the message, call at 10 Hz
    bool parse_message();

    private:
    void update_log();
    AP_HAL::UARTDriver *uart = nullptr;
    bool _init = false;
    float _volts = 0;
    float _amps = 0;
    float _temp = 0;
    uint8_t _message_to_send[11] = {HEADER,
                                    0,
                                    DEVICE_ID,
                                    static_cast<uint8_t>(Command::PCU_COMMAND),
                                    static_cast<uint8_t>(5 & 0xFF),
                                    static_cast<uint8_t>((5 >> 8) & 0xFF),
                                    1,1,0x94,0x05,0};
    uint8_t _message_buffer[30];
    uint8_t _message_buffer_pos = 0;
};

#endif