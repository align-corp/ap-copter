-- mount-G3P-driver.lua: Align G3P mount/gimbal driver - version 1.3
local PARAM_TABLE_KEY = 41
assert(param:add_table(PARAM_TABLE_KEY, "G3P_", 6), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add G3P_DEBUG param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "MS", 100), "could not add G3P_MS param")
assert(param:add_param(PARAM_TABLE_KEY, 3, "CENTER_CH", 9), "could not add G3P_CENTER_CH param")
assert(param:add_param(PARAM_TABLE_KEY, 4, "RC_EXPO", 2), "could not add G3P_RC_EXPO param")
assert(param:add_param(PARAM_TABLE_KEY, 5, "CAM", 2), "could not add G3P_CAM param")
local G3P_DEBUG = Parameter("G3P_DEBUG")
local G3P_MS = Parameter("G3P_MS")
local G3P_CENTER_CH = Parameter("G3P_CENTER_CH")
local G3P_RC_EXPO = Parameter("G3P_RC_EXPO")
local G3P_CAM = Parameter("G3P_CAM")
local MNT1_TYPE = Parameter("MNT1_TYPE")
local MNT1_PITCH_MAX = Parameter("MNT1_PITCH_MAX")
local MNT1_PITCH_MIN = Parameter("MNT1_PITCH_MIN")
local MNT1_ROLL_MAX = Parameter("MNT1_ROLL_MAX")
local MNT1_ROLL_MIN = Parameter("MNT1_ROLL_MIN")
local MNT1_YAW_MAX = Parameter("MNT1_YAW_MAX")
local MNT1_YAW_MIN = Parameter("MNT1_YAW_MIN")
local MNT1_RC_RATE = Parameter("MNT1_RC_RATE")
local CAM1_TYPE = Parameter("CAM1_TYPE")
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local MOUNT_INSTANCE = 0
local GPS_INSTANCE = gps:primary_sensor()
local MOUNT_RC_CENTER = 9
local MOUNT_RC_RATE = 30
local MOUNT_RC_EXPO = 1
local HEADER_SEND = 0x18
local HEADER_RECEIVE = 0x19
local MOUNT_CMD_PARAM_SET = 0x01
local MOUNT_CMD_PARAM_GET = 0x02
local MOUNT_CMD_ANGLE_SET = 0x03
local MOUNT_CMD_CALIBRATE = 0x04
local MOUNT_CMD_ANGLE_REQUEST = 0x05
local MOUNT_LENGTH_ANGLE_REQUEST = 0x06
local DV_HEADER = 0xAE
local DV_HEADER2_SEND = 0xA1
local DV_HEADER2_RECEIVE = 0xA2
local DV_CMD1 = 0x00
local DV_CMD2_LATH = 0xA8
local DV_CMD2_LATL = 0xA9
local DV_CMD2_LONH = 0xAA
local DV_CMD2_LONL = 0xAB
local DV_CMD2_ALT = 0xAC
local DV_CMD2_CAPTURE = 0xCE
local DV_DATA1_CAPTURE = 0x06
local DV_DATA2_CAPTURE = 0x20
local DV_CRC1_CAPTURE = 0x01
local DV_CRC2_CAPTURE = 0x95
local DV_END = 0xEA
local uart_gimbal
local uart_dv
local use_camera = false
local cam_pic_count = 0
local mount_buff = {}
local mount_center_switch = false
local yaw_deg = 0
local roll_deg = 0
local pitch_deg = 0
local last_print_ms = uint32_t(0)
local last_angle_received_ms = uint32_t(0)
local angle_control_started_ms = uint32_t(0)
function lowbyte(num)
return num & 0xFF
end
function highbyte(num)
return (num >> 8) & 0xFF
end
function byte1(num)
return num & 0xFF
end
function byte2(num)
return (num >> 8) & 0xFF
end
function byte3(num)
return (num >> 16) & 0xFF
end
function byte4(num)
return (num >> 24) & 0xFF
end
function uint16_value(hbyte, lbyte)
return ((hbyte & 0xFF) << 8) | (lbyte & 0xFF)
end
function int16_value(hbyte, lbyte)
local uret = uint16_value(hbyte, lbyte)
if uret <= 0x8000 then
return uret
else
return uret - 0x10000
end
end
function wrap_360(angle)
local res = math.fmod(angle, 360.0)
if res < 0 then
res = res + 360.0
end
return res
end
function wrap_180(angle_deg)
local res = wrap_360(angle_deg)
if res > 180 then
res = res - 360
end
return res
end
function checksum_mount(packet, len)
local ck_a = 0;
local ck_b = 0;
for i = 2, len-2, 1 do
ck_a = ck_a + packet[i];
ck_b = ck_b + ck_a;
end
return lowbyte(ck_a), lowbyte(ck_b)
end
function checksum_dv(packet, len)
local ck = packet[2] + packet[3] + packet[4] + packet[5] + packet[6];
local ck_1 = highbyte(ck);
local ck_2 = lowbyte(ck)
return ck_1, ck_2
end
function init()
local need_reboot = false
if G3P_CAM:get() > 1 then
use_camera = true
elseif G3P_CAM:get() > 0 then
use_camera = false
else
return
end
if MNT1_TYPE:get() ~= 9 then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: setting MNT1_TYPE=9")
MNT1_TYPE:set_and_save(9)
need_reboot = true
end
if use_camera and CAM1_TYPE:get() ~= 4 then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: setting CAM1_TYPE=4")
CAM1_TYPE:set_and_save(4)
need_reboot = true
end
if MNT1_PITCH_MAX:get() == nil or MNT1_PITCH_MIN:get() == nil or MNT1_ROLL_MAX:get() == nil or MNT1_ROLL_MIN:get() == nil or MNT1_YAW_MAX:get() == nil or MNT1_YAW_MIN:get() == nil then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: check MNT1_ parameters")
need_reboot = true
end
local rc_rate = MNT1_RC_RATE:get()
if rc_rate == nil or rc_rate <= 5 then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: setting MNT1_RC_RATE > 5")
MNT1_RC_RATE:set_and_save(60)
need_reboot = true
else
MOUNT_RC_RATE = rc_rate
end
local center_rc = G3P_CENTER_CH:get()
if center_rc == nil then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: set G3P_CENTER_CH with RC centering channel")
need_reboot = true
else
MOUNT_RC_CENTER = math.floor(center_rc)
local center_param_string = "RC" .. MOUNT_RC_CENTER .. "_OPTION"
if param:get(center_param_string) == 19 then
param:set(center_param_string, 0)
end
end
local expo_rc = G3P_RC_EXPO:get()
if expo_rc == nil or expo_rc > 4 then
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: G3P_RC_EXPO out of range")
need_reboot = true
elseif expo_rc < 1 then
MOUNT_RC_EXPO = 1
else
MOUNT_RC_EXPO = math.floor(expo_rc + 0.5)
end
if rc:find_channel_for_option(213) == nil or rc:find_channel_for_option(214) == nil then
param:set_and_save("RC11_OPTION", 213)
param:set_and_save("RC13_OPTION", 214)
param:set_and_save("RC13_REVERSED", 1)
gcs:send_text(MAV_SEVERITY.CRITICAL, "G3P: setting RC input")
need_reboot = true
end
uart_gimbal = serial:find_serial(0)
if uart_gimbal == nil then
gcs:send_text(3, "G3P: setting SERIAL5_PROTOCOL = 28 for gimbal")
param:set_and_save("SERIAL5_PROTOCOL", 28)
need_reboot = true
end
if use_camera then
uart_dv = serial:find_serial(1)
if uart_dv == nil then
gcs:send_text(3, "G3P: setting SERIAL7_PROTOCOL = 28 for DV")
param:set_and_save("SERIAL7_PROTOCOL", 28)
gcs:send_text(3, "G3P: need reboot")
return
end
uart_dv:begin(uint32_t(115200))
uart_dv:set_flow_control(0)
end
if need_reboot then
gcs:send_text(3, "G3P: need reboot")
return
end
uart_gimbal:begin(uint32_t(120000))
uart_gimbal:set_flow_control(0)
gcs:send_text(MAV_SEVERITY.INFO, "G3P: started")
last_angle_received_ms = millis()
return update, G3P_MS:get()
end
function expo(input)
local input_negative = input < 0
local normalized_input = math.abs(input / MOUNT_RC_RATE)
local output = normalized_input^MOUNT_RC_EXPO * MOUNT_RC_RATE
if input_negative then
output = -output
end
return output
end
function read_incoming_packets()
local n_bytes = uart_gimbal:available()
while n_bytes > 0 do
n_bytes = n_bytes - 1
mount_buff[#mount_buff+1] = uart_gimbal:read()
end
if #mount_buff >= 11 then
for i = 1, #mount_buff-10, 1 do
if mount_buff[i] == HEADER_RECEIVE then
parse_bytes(i)
mount_buff = {}
break
end
end
end
end
function parse_bytes(start)
local buffer = {}
for i = start, #mount_buff do
buffer[#buffer+1] = mount_buff[i]
end
if buffer[2] == MOUNT_CMD_ANGLE_REQUEST then
if buffer[3] == MOUNT_LENGTH_ANGLE_REQUEST then
local ck_a, ck_b = checksum_mount(buffer, #buffer)
if ck_a == buffer[10] and ck_b == buffer[11] then
yaw_deg = int16_value(buffer[4], buffer[5])/182.0444
roll_deg = int16_value(buffer[6], buffer[7])/182.0444
pitch_deg = int16_value(buffer[8], buffer[9])/182.0444
mount:set_attitude_euler(MOUNT_INSTANCE, roll_deg, pitch_deg, yaw_deg)
last_angle_received_ms = millis()
else
gcs:send_text(MAV_SEVERITY.ERROR, "G3P: wrong CHECKSUM")
end
else
gcs:send_text(MAV_SEVERITY.ERROR, "G3P: wrong LENGTH received")
end
else
gcs:send_text(MAV_SEVERITY.ERROR, "G3P: wrong CMD received")
end
end
function write_bytes(buff,len, uart)
if #buff == 0 or #buff < len then
gcs:send_text(MAV_SEVERITY.ERROR, "G3P: failed to write byte")
return false
end
local packet_string = "packet send at uart " .. uart .. ": "
for i = 1, len, 1 do
local byte_to_write = buff[i] & 0xFF
if uart == 0 then
uart_gimbal:write(byte_to_write)
elseif uart == 1 then
uart_dv:write(byte_to_write)
end
end
if G3P_DEBUG:get() > 3 then
gcs:send_text(MAV_SEVERITY.INFO, packet_string)
end
return true
end
function send_GPS()
if gps:status(GPS_INSTANCE) < 3 and G3P_DEBUG:get() ~= -1 then
return false
end
local location = gps:location(GPS_INSTANCE)
local lat = location:lat()
local lng = location:lng()
local alt = location:alt() // 10
if G3P_DEBUG:get() == -1 then
lng = 1140384850
lat = 226384700
alt = 500
end
if G3P_DEBUG:get() > 1 then
gcs:send_text(MAV_SEVERITY.INFO, string.format("Long: %d, Lat: %d, Alt: %d", lng, lat, alt))
end
local packet1 = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_LATH,
byte4(lat),
byte3(lat),
0,
0,
DV_END
}
local packet2 = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_LATL,
byte2(lat),
byte1(lat),
0,
0,
DV_END
}
local packet3 = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_LONH,
byte4(lng),
byte3(lng),
0,
0,
DV_END
}
local packet4 = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_LONL,
byte2(lng),
byte1(lng),
0,
0,
DV_END
}
local packet5 = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_ALT,
highbyte(alt),
lowbyte(alt),
0,
0,
DV_END
}
packet1[7], packet1[8] = checksum_dv(packet1, #packet1)
packet2[7], packet2[8] = checksum_dv(packet2, #packet2)
packet3[7], packet3[8] = checksum_dv(packet3, #packet3)
packet4[7], packet4[8] = checksum_dv(packet4, #packet4)
packet5[7], packet5[8] = checksum_dv(packet5, #packet5)
write_bytes(packet1, #packet1, 1)
write_bytes(packet2, #packet2, 1)
write_bytes(packet3, #packet3, 1)
write_bytes(packet4, #packet4, 1)
write_bytes(packet5, #packet5, 1)
return true
end
function send_target_angles(pitch_angle_deg, roll_angle_deg, yaw_angle_deg)
if G3P_DEBUG:get() > 1 then
gcs:send_text(MAV_SEVERITY.INFO, string.format("G3P send angles P: %f R: %f Y; %f", pitch_angle_deg, roll_angle_deg, yaw_angle_deg))
end
local roll_angle_output = math.floor(roll_angle_deg * 182.0444 + 0.5)
local pitch_angle_output = math.floor(pitch_angle_deg * 182.0444 + 0.5)
local yaw_angle_output = math.floor(yaw_angle_deg * 182.0444 + 0.5)
local packet_to_send = {HEADER_SEND,
MOUNT_CMD_ANGLE_SET,
0x06,
highbyte(yaw_angle_output),
lowbyte(yaw_angle_output),
highbyte(roll_angle_output),
lowbyte(roll_angle_output),
highbyte(pitch_angle_output),
lowbyte(pitch_angle_output),
0,
0 }
local ck_a, ck_b = checksum_mount(packet_to_send, #packet_to_send)
packet_to_send[10] = ck_a
packet_to_send[11] = ck_b
write_bytes(packet_to_send, #packet_to_send, 0)
end
function check_picture()
local pic_count, rec_video, zoom_step, focus_step, auto_focus = mount:get_camera_state(MOUNT_INSTANCE)
if pic_count and pic_count ~= cam_pic_count then
cam_pic_count = pic_count
local packet = {
DV_HEADER,
DV_HEADER2_SEND,
DV_CMD1,
DV_CMD2_CAPTURE,
DV_DATA1_CAPTURE,
DV_DATA2_CAPTURE,
DV_CRC1_CAPTURE,
DV_CRC2_CAPTURE,
DV_END
}
write_bytes(packet, #packet, 1)
if G3P_DEBUG:get() > 0 then
gcs:send_text(MAV_SEVERITY.INFO, string.format("G3P: took pic %u", cam_pic_count))
end
end
end
function request_angles()
local packet = {
HEADER_SEND,
MOUNT_CMD_ANGLE_REQUEST,
0x00,
0x05,
0x0A
}
write_bytes(packet, #packet, 0)
end
function center_gimbal()
local packet = {
HEADER_SEND,
MOUNT_CMD_CALIBRATE,
0x01,
0x5E,
0x00,
0x00
}
local ck_a, ck_b = checksum_mount(packet, #packet)
packet[5] = ck_a
packet[6] = ck_b
write_bytes(packet, #packet, 0)
end
function check_centering()
if mount_center_switch then
if rc:get_pwm(MOUNT_RC_CENTER) < 1200 then
mount_center_switch = false
return true
end
else
if rc:get_pwm(MOUNT_RC_CENTER) > 1800 then
mount_center_switch = true
return true
end
end
return false
end
function angle_controller(pitch_des_deg)
if angle_control_started_ms == uint32_t(0) then
angle_control_started_ms = millis()
end
local pitch_error = pitch_des_deg - pitch_deg
local pitch_rate = 10 * pitch_error
if pitch_rate > 120 then
pitch_rate = 120
elseif pitch_rate < -120 then
pitch_rate = -120
elseif pitch_rate < 6 and pitch_rate >= 0 then
pitch_rate = 6
elseif pitch_rate > -6 and pitch_rate < 0 then
pitch_rate = -6
end
send_target_angles(pitch_rate, 0, 0)
if G3P_DEBUG:get() > 0 then
gcs:send_text(MAV_SEVERITY.INFO, string.format("G3P pitch controller: angle = %f, error = %f, rate = %f", pitch_deg, pitch_error, pitch_rate))
end
if (math.abs(pitch_error) < 0.2) or (millis() - angle_control_started_ms) > 10000 then
angle_control_started_ms = uint32_t(0)
mount:set_mode(MOUNT_INSTANCE,3)
end
end
function update()
local now_ms = millis()
if now_ms - last_angle_received_ms > 20000 then
gcs:send_text(MAV_SEVERITY.ERROR, string.format("G3P: can't get gimbal angles, freezing"))
return
end
if use_camera then
send_GPS()
check_picture()
end
read_incoming_packets()
request_angles()
local des_roll_deg, des_pitch_deg, des_yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
local des_roll_degs, des_pitch_degs, des_yaw_degs, yaw_is_ef_rate = mount:get_rate_target(MOUNT_INSTANCE)
if check_centering() then
center_gimbal()
elseif des_roll_deg and des_pitch_deg and des_yaw_deg then
angle_controller(des_pitch_deg)
return update, 40
elseif des_roll_degs and des_pitch_degs and des_yaw_degs then
des_roll_degs = expo(des_roll_degs)
des_pitch_degs = expo(des_pitch_degs)
des_yaw_degs = expo(des_yaw_degs)
send_target_angles(des_pitch_degs, des_roll_degs, des_yaw_degs)
else
gcs:send_text(MAV_SEVERITY.ERROR, "G3P: can't get target angles")
return update, 2000
end
if des_roll_degs == nil or des_pitch_degs == nil or des_yaw_degs == nil then
des_roll_degs = 0
des_pitch_degs = 0
des_yaw_degs = 0
end
if (G3P_DEBUG:get() > 0) and (now_ms - last_print_ms > 5000) then
last_print_ms = now_ms
gcs:send_text(MAV_SEVERITY.INFO, string.format("G3P angle: R = %f P = %f Y = %f, speed: R = %f P = %f Y = %f", roll_deg, pitch_deg, yaw_deg, des_roll_degs, des_pitch_degs, des_yaw_degs))
debug_count = 0
end
return update, G3P_MS:get()
end
return init, 1000
