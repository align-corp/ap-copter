local HOME_RC = 7
local PWM_UP = 1700
local PWM_DOWN = 1300
local butt_state = true

function update_home()
    local loc = ahrs:get_location()
    if loc == nil then
        return update_home, 200
    end

    if butt_state and (rc:get_pwm(HOME_RC) < PWM_DOWN)  then
        ahrs:set_home(loc)
        butt_state = false

    elseif (not butt_state) and (rc:get_pwm(HOME_RC) > PWM_UP)  then
        ahrs:set_home(loc)
        butt_state = true
    end

    return update_home, 200
end

function init_button()
    if rc:has_valid_input() then
        if rc:get_pwm(HOME_RC) < PWM_DOWN then
            butt_state = false
        end
        return update_home()
    end

    return init_button, 500
end

gcs:send_text('6', "set_home.lua is running")
return init_button, 500



