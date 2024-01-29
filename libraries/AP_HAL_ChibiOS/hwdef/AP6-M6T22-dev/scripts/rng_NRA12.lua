-- version 1.0
local INIT_MILLIS = 3000
local UART_BAUD = uint32_t(115200)
local OUT_OF_RANGE_HIGH = 20
local INSTANCE = 0
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local PARAM_LUA_RFND = 36 -- parameter number for lua rangefinder
local PARAM_TABLE_KEY = 72
local HEADER0 = 0xAA
local HEADER1 = 0xAA
local HEADER2_TARGET_INFO = 0x0C
local HEADER3_TARGET_INFO = 0x07
local END1 = 0x55
local END2 = 0x55
assert(param:add_table(PARAM_TABLE_KEY, "NRA_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, "LOG", 0), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3, "UPDATE", 24), 'could not add param3')
local NRA_DEBUG = Parameter("NRA_DEBUG")
local NRA_LOG = Parameter("NRA_LOG")
local NRA_UPDATE = Parameter("NRA_UPDATE")
local lua_rfnd_backend
local parse_state = 0
local distance = 0
local distance_received = false
local uart = nil
local header_found_ms = uint32_t(0)
local distance_received_ms = uint32_t(0)
local report_ms = uint32_t(0)
local report_count = 0
function init_rng()
        lua_rfnd_backend = rangefinder:get_backend(INSTANCE)
        if lua_rfnd_backend == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("RFND: Configure RNGFND%d_TYPE = %d", INSTANCE+1, PARAM_LUA_RFND))
            return
        end

        if lua_rfnd_backend:type() ~= PARAM_LUA_RFND then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("RFND: Configure RNGFND%d_TYPE = %d", INSTANCE+1, PARAM_LUA_RFND))
            return
        end

        uart = serial:find_serial(INSTANCE)
        if uart == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, "RFND: configure SERIALx_PROTOCOL = 28 and reboot")
            return
        end

        uart:begin(UART_BAUD)
        uart:set_flow_control(0)
        while uart:available()>0 do
            uart:read()
        end
        gcs:send_text(MAV_SEVERITY.INFO, "RFND: succesfully started")
        return update, NRA_UPDATE:get()
end

function read_incoming_bytes()
    local now = millis()
    local n_bytes = uart:available()
    while n_bytes > 0 do
        n_bytes = n_bytes-1
        local byte = uart:read()
        if parse_state == 0 then
            if byte == HEADER0 then
                parse_state = 1
            end
        elseif parse_state == 1 then
            if byte == HEADER1 then
                parse_state = 2
                header_found_ms = now
            else
                parse_state = 0
            end
        elseif parse_state == 2 then
            if byte == HEADER2_TARGET_INFO then
                parse_state = 3
            else
                parse_state = 0
            end
        elseif parse_state == 3 then
            if byte == HEADER3_TARGET_INFO then
                parse_state = 4
            else
                parse_state = 0
            end
        elseif parse_state < 6 then
            parse_state = parse_state + 1
        elseif parse_state == 6 then
            distance = byte * 256
            parse_state = 7
        elseif parse_state == 7 then
            distance = distance + byte
            parse_state = 8
        elseif parse_state < 12 then
            parse_state = parse_state + 1
        elseif parse_state == 12 then
            if byte == END1 then
                parse_state = 13
            else
                parse_state = 0
            end
        elseif parse_state == 13 then
            if byte == END2 then
                distance_received = true
                distance_received_ms = now
            end
            parse_state = 0
        end
    end

end

function send_distance(distance_m)
    if distance_m > 20 then
        distance_m = 25
    end
    local sent_successfully = lua_rfnd_backend:handle_script_msg(distance_m)
    if not sent_successfully then
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("RFND: Lua Script Error"))
    end
end
function update()
    read_incoming_bytes()
    local now = millis()
    if distance_received then
        report_count = report_count +1
        send_distance(distance/100)
        distance_received = false
    elseif now - distance_received_ms > 100 and now - header_found_ms < 50 then
        send_distance(OUT_OF_RANGE_HIGH)
    end

    if NRA_DEBUG:get() > 0 then
        if now - report_ms > 5000 then
            report_ms = now
            gcs:send_text(MAV_SEVERITY.INFO, string.format("RFND: received %d samples, last dist = %d", report_count, distance))
            report_count = 0
        end
    end

    return update, NRA_UPDATE:get()
end
return init_rng, NRA_UPDATE:get()