#pragma once
#include "AP_ProtocolPCU/AP_ProtocolPCU.h"

#if HAL_PCU_ENABLED
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#define AP_BATT_PCU_VOLT_MULT  1
#define AP_BATT_PCU_VOLT_OFFS  0
#define AP_BATT_PCU_AMP_MULT  1
#define AP_BATT_PCU_AMP_OFFS  0

class AP_BattMonitor_PCU : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_PCU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return true; }

    virtual void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:
    
    AP_ProtocolPCU _pcu1;                /// feed voltage and current from the PCU hardware connected to serial2

    // Parameters
    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _amp_multiplier;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Float _volt_offset;              /// offset voltage that is subtracted from voltage pin before conversion

};

#endif
