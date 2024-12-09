-- control bucket position with RC input - version 1.0
local RELAY_PIN_UP = 1
local RELAY_PIN_DOWN = 2
local RELAY_PIN_B_UP = 3
local RELAY_PIN_B_DOWN = 4
local MILLIS_UPDATE = 150
local PARAM_TABLE_KEY = 42
assert(param:add_table(PARAM_TABLE_KEY, "BUCK_", 2), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "CH", 2), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "CH_B", 4), "could not add param 2")
local BUCK_CH = Parameter("BUCK_CH")
local BUCK_CH_B = Parameter("BUCK_CH_B")
BUCK_CH:init("BUCK_CH")
function init()
gpio:pinMode(RELAY_PIN_UP,1)
gpio:pinMode(RELAY_PIN_DOWN,1)
gpio:pinMode(RELAY_PIN_B_UP,1)
gpio:pinMode(RELAY_PIN_B_DOWN,1)
return update, 100
end
function update()
if BUCK_CH:get() == nil then
return update, 1000
elseif BUCK_CH:get() < 1 or BUCK_CH:get() > 12 then
return update, 1000
elseif BUCK_CH_B:get() < 1 or BUCK_CH_B:get() > 12 then
return update, 1000
else
local channel = math.floor(BUCK_CH:get())
local channel_b = math.floor(BUCK_CH_B:get())
local pwm = rc:get_pwm(channel)
if pwm > 900 and pwm < 1400 then
gpio:write(RELAY_PIN_UP, 0)
gpio:write(RELAY_PIN_DOWN, 1)
elseif pwm > 1600 and pwm < 2100 then
gpio:write(RELAY_PIN_UP, 1)
gpio:write(RELAY_PIN_DOWN, 0)
else
gpio:write(RELAY_PIN_UP, 0)
gpio:write(RELAY_PIN_DOWN, 0)
end
if pwm > 900 and pwm < 1400 then
gpio:write(RELAY_PIN_B_UP, 0)
gpio:write(RELAY_PIN_B_DOWN, 1)
elseif pwm > 1600 and pwm < 2100 then
gpio:write(RELAY_PIN_B_UP, 1)
gpio:write(RELAY_PIN_B_DOWN, 0)
else
gpio:write(RELAY_PIN_B_UP, 0)
gpio:write(RELAY_PIN_B_DOWN, 0)
end
return update, MILLIS_UPDATE
end
end
gcs:send_text(6, "bucket controller is running")
return init, 100
