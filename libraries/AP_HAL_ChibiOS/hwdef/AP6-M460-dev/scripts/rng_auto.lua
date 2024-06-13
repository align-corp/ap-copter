-- switch to loiter mode if getting near obstacles - version 1.0
local UPDATE_MS = 100
local AUTO_MODE = 3
local GUIDED_MODE = 4
local LOITER_MODE = 5
local PARAM_TABLE_KEY = 45
local distances = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}
local distances_count = {0, 0, 0, 0, 0, 0, 0, 0, 0}
local last_gcs_string_sent = uint32_t(0)
assert(param:add_table(PARAM_TABLE_KEY, "OAL_", 5), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "could not add param1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "DIST_CM", 600), "could not add param1")
assert(param:add_param(PARAM_TABLE_KEY, 3, "NEAR_MS", 400), "could not add param3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "MIN_SPD", 1.0), "could not add param4")
assert(param:add_param(PARAM_TABLE_KEY, 5, "DEBUG", 0), "could not add param5")
local ENABLE = Parameter("OAL_ENABLE")
local DIST_CM = Parameter("OAL_DIST_CM")
local NEAR_MS = Parameter("OAL_NEAR_MS")
local MIN_SPD = Parameter("OAL_MIN_SPD")
local DEBUG = Parameter("OAL_DEBUG")
function distance_check()
local count_limit = NEAR_MS:get() / UPDATE_MS
if vehicle:get_mode() ~= AUTO_MODE and vehicle:get_mode() ~= GUIDED_MODE then
return distance_check, UPDATE_MS
end
if ahrs:groundspeed_vector():length() <= MIN_SPD:get() then
return distance_check, UPDATE_MS
end
local closest_angle, closest_distance = proximity:get_closest_object()
if closest_angle == nil then
if DEBUG:get() > 0 and millis() - last_gcs_string_sent > 1000 then
gcs:send_text(6, string.format(" status: %d, num_sensors: %d", proximity:get_status(), proximity:num_sensors()))
last_gcs_string_sent = millis()
end
return distance_check, UPDATE_MS
end
closest_distance = math.floor(closest_distance*100)
local sector = math.floor(closest_angle / 45) + 1
if closest_distance < DIST_CM:get() and closest_distance < distances[sector] then
distances_count[sector] = distances_count[sector] + 1
else
distances_count[sector] = 0
end
distances[sector] = closest_distance
if distances_count[sector] >= count_limit then
vehicle:set_mode(LOITER_MODE)
gcs:send_text(2, string.format("Obstacle detected at %d cm, switch to Loiter", closest_distance))
distances_count[sector] = 0
end
if DEBUG:get() > 0 and millis() - last_gcs_string_sent > 1000 then
gcs:send_text(6, string.format("Angle: %f, Sector: %d, Distance: %d, count: %d", closest_angle, sector, closest_distance, distances_count[sector]))
last_gcs_string_sent = millis()
end
return distance_check, UPDATE_MS
end
if ENABLE == 0 then
return
end
if DIST_CM:get() < 10 or NEAR_MS:get() < 100 then
gcs:send_text(2, string.format("OAL: check parameters"))
return
end
gcs:send_text(6, string.format("OAL is starting, dist=%f cm, timeout=%f ms,\nmin speed=%f m/s", DIST_CM:get(), NEAR_MS:get(), MIN_SPD:get()))
return distance_check, 100
