-- Driver for Custom Serial Rangefinder (Align RADAR) - version 1.0
local UART_BAUD = uint32_t(115200)
local OUT_OF_RANGE_HIGH = 50
local INSTANCE = 1
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local PARAM_LUA_RFND = 36
local PARAM_TABLE_KEY = 73
local HEADER1 = 0x55
assert(param:add_table(PARAM_TABLE_KEY, "RDR_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, "UPDATE", 11), 'could not add param3')
local RDR_DEBUG = Parameter("RDR_DEBUG")
local RDR_UPDATE = Parameter("RDR_UPDATE")
local lua_rfnd_backend
local parse_state = 0
local distance = 0
local distance_received = false
local uart = nil
local report_ms = uint32_t(0)
local report_count = 0
local byte_2
local byte_3
local byte_4
local byte_5
local now = uint32_t(0)
local distances = {0,0,0,0,0,0,0,0,0,0}
local distances_count = 0
local cycle_count = 0
function init_rng()
lua_rfnd_backend = rangefinder:get_backend(INSTANCE)
if lua_rfnd_backend == nil then
gcs:send_text(MAV_SEVERITY.ERROR, string.format("RFND: Configure RNGFND%d_TYPE = %d", INSTANCE+1, PARAM_LUA_RFND))
return
end
if lua_rfnd_backend:type() ~= PARAM_LUA_RFND then
gcs:send_text(MAV_SEVERITY.ERROR,"RFND: Configure RNGFND1_TYPE = " .. PARAM_LUA_RFND)
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
gcs:send_text(MAV_SEVERITY.INFO, "RFND: Align succesfully started")
return update, RDR_UPDATE:get()
end
function read_incoming_bytes()
local n_bytes = uart:available()
if RDR_DEBUG:get() > 2 then
gcs:send_text(0, string.format("bytes: %d", n_bytes:toint() ))
end
while n_bytes > 0 do
n_bytes = n_bytes-1
local byte = uart:read()
if parse_state == 0 then
if byte == HEADER1 then
parse_state = 1
end
elseif parse_state == 1 then
byte_2 = byte
parse_state = 2
elseif parse_state == 2 then
byte_3 = byte
parse_state = 3
elseif parse_state == 3 then
distance = byte
byte_4 = byte
parse_state = 4
elseif parse_state == 4 then
byte_5 = byte
byte = byte * 256
distance = distance + byte
parse_state = 5
elseif parse_state == 5 then
local checksum_calc = (byte_2+byte_3+byte_4+byte_5) & 0xFF
if checksum_calc == byte then
distance_received = true
else
if RDR_DEBUG:get() > 1 then
gcs:send_text(0,string.format("Wrong checksum: %d - %d", byte, checksum_calc))
end
end
parse_state = 0
end
end
end
function send_distance(distance_m)
if distance_m >= OUT_OF_RANGE_HIGH then
distance_m = OUT_OF_RANGE_HIGH+5
end
local sent_successfully = lua_rfnd_backend:handle_script_msg(distance_m)
if not sent_successfully then
gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("RFND: Lua Script Error"))
end
end
function update()
now = millis()
read_incoming_bytes()
cycle_count = cycle_count + 1
if distance_received then
report_count = report_count + 1
distances[distances_count] = distance
distances_count = distances_count + 1
distance_received = false
end
if cycle_count == 11 then
if distances_count > 1 then
table.sort(distances)
local dist_to_send = distances[math.floor(distances_count/2+0.5)]
send_distance(dist_to_send/100)
end
cycle_count = 1
distances_count = 1
end
if RDR_DEBUG:get() > 0 then
if now - report_ms > 5000 then
report_ms = now
gcs:send_text(MAV_SEVERITY.INFO, string.format("RFND: received %d samples, last dist = %d", report_count, distance))
report_count = 0
end
end
local elapsed = (millis() - now):toint()
return update, math.abs(RDR_UPDATE:get() - elapsed)
end
return init_rng, RDR_UPDATE:get()
