-- auto config GPS for rovers

function gps_set()
    param:set_and_save("ARMING_REQUIRE", 0)
    param:set_and_save("GPS_AUTO_CONFIG", 0)
    gcs:send_text("4", "GPS modules set up completed, please reboot vehicle")
end

function get_gps_autoconfig()
    if param:get("GPS_AUTO_CONFIG") == nil then
        return get_gps_autoconfig, 500
    elseif param:get("GPS_AUTO_CONFIG") == 0 then
        -- die here, gps already set
        return
    elseif param:get("GPS_AUTO_CONFIG") == 1 then
        -- wait for gps to configure and toggle parameter
        if param:get("ARMING_REQUIRE") == 0 then
            param:set("ARMING_REQUIRE", 1)
        end
        gcs:send_text("4", "GPS modules are setting up, don't turn off vehicle")
        return gps_set, 60000
    end
end

gcs:send_text("6", "gps_config.lua is running")
return get_gps_autoconfig, 500

