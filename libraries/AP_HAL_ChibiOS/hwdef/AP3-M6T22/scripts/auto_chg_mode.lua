local LOITER_MODE = 5
local RTL_MODE = 6
local AUTO_MODE = 3
local LAND_MODE = 9

function update()
    if vehicle:get_mode() == AUTO_MODE then
        if mission:state() == mission.MISSION_COMPLETE then
            vehicle:set_mode(LOITER_MODE)
        end
    end
    return update, 1000
end

gcs:send_text('6', "auto change mode is running")
return update, 1000