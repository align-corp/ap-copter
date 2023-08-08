#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_PCU.h"

#if HAL_PCU_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#define PCU_NOT_HEALTY_MICROS 5000000
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_PCU::var_info[] = {

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Adjust voltage reading: (PCU_volt) - _volt_offset) * VOLT_MULT;
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 1, AP_BattMonitor_PCU, _volt_multiplier, AP_BATT_PCU_VOLT_MULT),

    // @Param: AMP_MULT
    // @DisplayName: Amps Multiplier
    // @Description: Adjust voltage reading: (PCU_volt) - _volt_offset) * VOLT_MULT;
    // @User: Standard
    AP_GROUPINFO("AMP_MULT", 2, AP_BattMonitor_PCU, _amp_multiplier, AP_BATT_PCU_AMP_MULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 3, AP_BattMonitor_PCU, _amp_offset, AP_BATT_PCU_AMP_OFFS),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 4, AP_BattMonitor_PCU, _volt_offset, AP_BATT_PCU_VOLT_OFFS),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_PCU::AP_BattMonitor_PCU(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// read - read the voltage and current
void
AP_BattMonitor_PCU::read()
{
    const uint32_t tnow = AP_HAL::micros();

    if (!_pcu1.is_initialized())
    {
        _pcu1.init_serial(0);
        return;
    }

    // PCU integration
    _pcu1.send_message();
    if (_pcu1.parse_message()){
        
        // Healthy if messages are received
        _state.healthy = true;

        // get voltage
        _state.voltage = (_pcu1.get_volt() - _volt_offset) * _volt_multiplier;

        // calculate time since last current read
        const uint32_t dt_us = tnow - _state.last_time_micros;
        // read current
        _state.current_amps = (_pcu1.get_amps() - _amp_offset) * _amp_multiplier;
        update_consumed(_state, dt_us);
        // record time
        _state.last_time_micros = tnow;
    }
    // Not healthy if no messages for 5 seconds
    else if (((tnow - _state.last_time_micros) > PCU_NOT_HEALTY_MICROS)){
        _state.healthy = false;
    }
}

#endif
