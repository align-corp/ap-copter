#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_P2_ENABLED

#include "AP_BattMonitor.h"

// Protocol 
#define AP_BATT_P2_PACKET_MAX_LENGTH 40
#define AP_BATT_P2_HEADER 0x77
#define AP_BATT_P2_OPERATION_BROADCAST 0xAC
#define AP_BATT_P2_LENGTH_BATT 28
#define AP_BATT_P2_LENGTH_FUEL 12
#define AP_BATT_P2_SENDER_P2 0x02
#define AP_BATT_P2_PACKET_ID_BATT 0x12
#define AP_BATT_P2_PACKET_ID_FUEL 0x11

class AP_BattMonitor_P2 : public AP_BattMonitor_Backend
{
public:
    // inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

    /// returns true if battery monitor provides temperature
    virtual bool has_temperature() const override { return true; }

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return true; }

private:
    // Receive and parse the message, call at 10 Hz
    bool parse_message();

    void parse_batt();

    void parse_fuel();

    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER,
        WAITING_FOR_OPERATION,
        WAITING_FOR_LENGTH,
        WAITING_FOR_SENDER_ID,
        WAITING_FOR_RECEIVER_ID,
        WAITING_FOR_PACKET_ID,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CRC,
    };

    // parser state and unpacked fields
    struct PACKED {
        uint16_t data_len;                          // expected number of data bytes
        uint8_t packet_id;                         // command id
        uint8_t sender_id;
        uint8_t receiver_id;
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint32_t checksum;                           // latest message's crc
        uint8_t checksum_bit;
        ParseState state;                     // state of incoming message processing
    } _parsed_msg;

    AP_HAL::UARTDriver *_uart = nullptr;
    uint8_t _msg_buff[AP_BATT_P2_PACKET_MAX_LENGTH];
    uint8_t _msg_buff_len = 0;
};

class AP_BattMonitor_P2_3s : public AP_BattMonitor_Backend
{
public:

    // inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

private:
    uint32_t _last_update_micros = 0;
};

class AP_BattMonitor_P2_BEC : public AP_BattMonitor_Backend
{
public:

    // inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    // returns true if battery monitor provides individual cell voltages
    virtual bool has_cell_voltages() const override { return true; }

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

private:
    uint32_t _last_update_micros = 0;
};


#endif
