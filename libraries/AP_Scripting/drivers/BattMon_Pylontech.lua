--[[
 Pylontech CAN battery monitor driver

 Implements PYLON CANBUS Protocol v1.2 using ArduPilot's scripting CAN
 and battery monitor interfaces. See BattMon_Pylontech.md for setup and
 commissioning instructions.
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: missing-parameter

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7,
}

local PARAM_TABLE_KEY = 146
local PARAM_TABLE_PREFIX = "PYLON_"
local PARAM_TABLE_SIZE = 6

local HEARTBEAT_ID = 0x305
local LIMITS_ID = 0x351
local SOC_SOH_ID = 0x355
local MEASUREMENTS_ID = 0x356
local FAULTS_ID = 0x359
local ENABLES_ID = 0x35C
local MANUFACTURER_ID = 0x35E

local HEARTBEAT_PERIOD_MS = 1000
local PUBLISH_PERIOD_MS = 200
local UPDATE_PERIOD_MS = 50
local WRITE_TIMEOUT_US = 10000
local DEBUG_PERIOD_MS = 5000
local WARNING_PERIOD_MS = 5000
local CAN_BUFFER_LEN = 20

local function bind_add_param(name, index, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, index, name, default_value),
           string.format("PYLON: could not add parameter %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE),
       "PYLON: could not add parameter table")

--[[
  // @Param: PYLON_ENABLE
  // @DisplayName: Pylontech battery monitor enable
  // @Description: Enable the Pylontech CAN battery monitor script
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PYLON_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: PYLON_CANDRV
  // @DisplayName: Pylontech scripting CAN driver
  // @Description: Select CAN scripting protocol 10 or 12; this is not the physical CAN port number
  // @Values: 1:Scripting protocol 10,2:Scripting2 protocol 12
  // @User: Advanced
--]]
local PYLON_CANDRV = bind_add_param("CANDRV", 2, 1)

--[[
  // @Param: PYLON_BATT_IDX
  // @DisplayName: Pylontech battery monitor index
  // @Description: ArduPilot battery monitor instance that receives Pylontech data
  // @Range: 1 9
  // @User: Standard
--]]
local PYLON_BATT_IDX = bind_add_param("BATT_IDX", 3, 1)

--[[
  // @Param: PYLON_CURR_MUL
  // @DisplayName: Pylontech current multiplier
  // @Description: Multiplier applied to reported battery current; use -1 when the BMS reports discharge as negative
  // @Range: -10 10
  // @User: Advanced
--]]
local PYLON_CURR_MUL = bind_add_param("CURR_MUL", 4, -1)

--[[
  // @Param: PYLON_TIMEOUT
  // @DisplayName: Pylontech measurement timeout
  // @Description: Maximum age of both SOC and measurement frames before the battery monitor becomes unhealthy
  // @Units: ms
  // @Range: 1500 10000
  // @User: Advanced
--]]
local PYLON_TIMEOUT = bind_add_param("TIMEOUT", 5, 3000)

--[[
  // @Param: PYLON_DEBUG
  // @DisplayName: Pylontech debug output
  // @Description: Emit periodic Pylontech status messages to the ground station
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local PYLON_DEBUG = bind_add_param("DEBUG", 6, 0)

if PYLON_ENABLE:get() == 0 then
    gcs:send_text(MAV_SEVERITY.INFO, "PYLON: disabled")
    return
end

local driver
if PYLON_CANDRV:get() == 1 then
    driver = CAN:get_device(CAN_BUFFER_LEN)
elseif PYLON_CANDRV:get() == 2 then
    driver = CAN:get_device2(CAN_BUFFER_LEN)
end

if not driver then
    gcs:send_text(MAV_SEVERITY.ERROR, "PYLON: scripting CAN driver unavailable")
    return
end

local FILTER_MASK = 0x7FF
local receive_ids = {
    LIMITS_ID,
    SOC_SOH_ID,
    MEASUREMENTS_ID,
    FAULTS_ID,
    ENABLES_ID,
    MANUFACTURER_ID,
}
for _, id in ipairs(receive_ids) do
    assert(driver:add_filter(uint32_t(FILTER_MASK), uint32_t(id)),
           "PYLON: could not add CAN filter")
end

local battery_index = math.floor(PYLON_BATT_IDX:get()) - 1
if battery_index < 0 or battery_index >= battery:num_instances() then
    gcs:send_text(MAV_SEVERITY.ERROR, "PYLON: invalid battery monitor index")
    return
end

local state = {
    got_soc = false,
    got_measurements = false,
    soc = nil,
    soh = nil,
    voltage = nil,
    current = nil,
    temperature = nil,
    last_soc_ms = uint32_t(0),
    last_measurements_ms = uint32_t(0),
    protection = nil,
    alarm = nil,
    enable_flags = nil,
    module_count = nil,
    manufacturer_checked = false,
}

local last_heartbeat_ms = uint32_t(0)
local last_publish_ms = uint32_t(0)
local last_debug_ms = uint32_t(0)
local last_warning_ms = uint32_t(0)
local heartbeat_sent = false
local last_health = nil
local backend_error_reported = false

local function get_uint16(frame, offset)
    return frame:data(offset) | (frame:data(offset + 1) << 8)
end

local function get_int16(frame, offset)
    local value = get_uint16(frame, offset)
    if value >= 0x8000 then
        value = value - 0x10000
    end
    return value
end

local function warn_throttled(message)
    local now_ms = millis()
    if last_warning_ms == uint32_t(0) or now_ms - last_warning_ms >= WARNING_PERIOD_MS then
        last_warning_ms = now_ms
        gcs:send_text(MAV_SEVERITY.WARNING, message)
    end
end

local function parse_limits(frame)
    if frame:dlc() < 6 then
        return
    end

    local charge_voltage = get_uint16(frame, 0) * 0.1
    local charge_current = get_int16(frame, 2) * 0.1
    local discharge_current = get_int16(frame, 4) * 0.1

    logger:write("PYLL", "ChgV,ChgA,DsgA", "fff",
                 charge_voltage, charge_current, discharge_current)
end

local function parse_soc_soh(frame, now_ms)
    if frame:dlc() < 4 then
        return
    end

    local soc = get_uint16(frame, 0)
    local soh = get_uint16(frame, 2)
    if soc > 100 or soh > 100 then
        warn_throttled("PYLON: rejected invalid SOC/SOH frame")
        return
    end

    state.soc = soc
    state.soh = soh
    state.got_soc = true
    state.last_soc_ms = now_ms
end

local function parse_measurements(frame, now_ms)
    if frame:dlc() < 6 then
        return
    end

    local voltage = get_int16(frame, 0) * 0.01
    local current = get_int16(frame, 2) * 0.1 * PYLON_CURR_MUL:get()
    local temperature = get_int16(frame, 4) * 0.1

    if voltage <= 0 or voltage > 1000 or math.abs(current) > 10000 or
       temperature < -100 or temperature > 200 then
        warn_throttled("PYLON: rejected implausible measurement frame")
        return
    end

    state.voltage = voltage
    state.current = current
    state.temperature = temperature
    state.got_measurements = true
    state.last_measurements_ms = now_ms
end

local function report_fault_change(protection, alarm)
    if state.protection ~= protection then
        if protection ~= 0 then
            gcs:send_text(MAV_SEVERITY.CRITICAL,
                          string.format("PYLON: protection 0x%04X", protection))
        elseif state.protection ~= nil and state.protection ~= 0 then
            gcs:send_text(MAV_SEVERITY.NOTICE, "PYLON: protection cleared")
        end
    end

    if state.alarm ~= alarm then
        if alarm ~= 0 then
            gcs:send_text(MAV_SEVERITY.WARNING,
                          string.format("PYLON: alarm 0x%04X", alarm))
        elseif state.alarm ~= nil and state.alarm ~= 0 then
            gcs:send_text(MAV_SEVERITY.NOTICE, "PYLON: alarm cleared")
        end
    end
end

local function parse_faults(frame)
    if frame:dlc() < 7 then
        return
    end

    local protection = frame:data(0) | (frame:data(1) << 8)
    local alarm = frame:data(2) | (frame:data(3) << 8)
    local module_count = frame:data(4)

    report_fault_change(protection, alarm)
    state.protection = protection
    state.alarm = alarm
    state.module_count = module_count

    logger:write("PYLF", "Prot,Alarm,Flags,Mods", "HHBB",
                 protection, alarm, state.enable_flags or 0, module_count)
end

local function parse_enables(frame)
    if frame:dlc() < 1 then
        return
    end

    local flags = frame:data(0)
    if state.enable_flags ~= flags then
        local charge_enabled = (flags >> 7) & 1
        local discharge_enabled = (flags >> 6) & 1
        local requests = (flags >> 3) & 0x07
        gcs:send_text(MAV_SEVERITY.INFO,
                      string.format("PYLON: charge:%u discharge:%u requests:0x%X",
                                    charge_enabled, discharge_enabled, requests))
    end
    state.enable_flags = flags

    logger:write("PYLF", "Prot,Alarm,Flags,Mods", "HHBB",
                 state.protection or 0, state.alarm or 0, flags,
                 state.module_count or 0)
end

local function parse_manufacturer(frame)
    if state.manufacturer_checked or frame:dlc() < 5 then
        return
    end
    state.manufacturer_checked = true

    local manufacturer = string.char(frame:data(0), frame:data(1), frame:data(2),
                                     frame:data(3), frame:data(4))
    if manufacturer == "PYLON" then
        gcs:send_text(MAV_SEVERITY.INFO, "PYLON: manufacturer confirmed")
    else
        gcs:send_text(MAV_SEVERITY.WARNING, "PYLON: unexpected manufacturer frame")
    end
end

local function handle_frame(frame, now_ms)
    if frame:isExtended() or frame:isRemoteTransmissionRequest() or frame:isErrorFrame() then
        return
    end

    local id = frame:id_signed()
    if id == LIMITS_ID then
        parse_limits(frame)
    elseif id == SOC_SOH_ID then
        parse_soc_soh(frame, now_ms)
    elseif id == MEASUREMENTS_ID then
        parse_measurements(frame, now_ms)
    elseif id == FAULTS_ID then
        parse_faults(frame)
    elseif id == ENABLES_ID then
        parse_enables(frame)
    elseif id == MANUFACTURER_ID then
        parse_manufacturer(frame)
    end
end

local function read_frames(now_ms)
    while true do
        local frame = driver:read_frame()
        if not frame then
            return
        end
        handle_frame(frame, now_ms)
    end
end

local function send_heartbeat(now_ms)
    if heartbeat_sent and now_ms - last_heartbeat_ms < HEARTBEAT_PERIOD_MS then
        return
    end

    local frame = CANFrame()
    frame:id(uint32_t(HEARTBEAT_ID))
    frame:dlc(8)
    for i = 0, 7 do
        frame:data(i, 0)
    end

    heartbeat_sent = true
    last_heartbeat_ms = now_ms
    if not driver:write_frame(frame, WRITE_TIMEOUT_US) then
        warn_throttled("PYLON: heartbeat write failed")
        return
    end
end

local function measurements_healthy(now_ms)
    local timeout_ms = math.max(1500, PYLON_TIMEOUT:get())
    return state.got_soc and state.got_measurements and
           now_ms - state.last_soc_ms <= timeout_ms and
           now_ms - state.last_measurements_ms <= timeout_ms
end

local function publish_battery(now_ms)
    if last_publish_ms ~= uint32_t(0) and now_ms - last_publish_ms < PUBLISH_PERIOD_MS then
        return
    end
    last_publish_ms = now_ms

    if not state.got_soc and not state.got_measurements then
        return
    end

    local healthy = measurements_healthy(now_ms)
    local batt_state = BattMonitorScript_State()
    batt_state:healthy(healthy)

    if state.voltage ~= nil then
        batt_state:voltage(state.voltage)
    end
    if state.current ~= nil then
        batt_state:current_amps(state.current)
    end
    if state.temperature ~= nil then
        batt_state:temperature(state.temperature)
    end
    if state.soc ~= nil then
        batt_state:capacity_remaining_pct(state.soc)
        local capacity_mah = battery:pack_capacity_mah(battery_index)
        if capacity_mah > 10 then
            batt_state:consumed_mah(capacity_mah * (100 - state.soc) * 0.01)
        end
    end
    if state.soh ~= nil then
        batt_state:state_of_health_pct(state.soh)
    end

    if not battery:handle_scripting(battery_index, batt_state) then
        if not backend_error_reported then
            backend_error_reported = true
            gcs:send_text(MAV_SEVERITY.ERROR,
                          "PYLON: battery index is not a scripting monitor")
        end
        return
    end

    if last_health == nil or last_health ~= healthy then
        if healthy then
            gcs:send_text(MAV_SEVERITY.INFO, "PYLON: battery monitor healthy")
        elseif last_health == true then
            gcs:send_text(MAV_SEVERITY.WARNING, "PYLON: battery measurements stale")
        end
        last_health = healthy
    end

    if healthy then
        logger:write("PYLB", "Volt,Curr,Temp,SOC,SOH", "fffBB",
                     state.voltage, state.current, state.temperature,
                     state.soc, state.soh)
    end
end

local function send_debug(now_ms)
    if PYLON_DEBUG:get() == 0 or
       (last_debug_ms ~= uint32_t(0) and now_ms - last_debug_ms < DEBUG_PERIOD_MS) then
        return
    end
    last_debug_ms = now_ms

    local soc_age = state.got_soc and (now_ms - state.last_soc_ms):toint() or -1
    local measurement_age = state.got_measurements and
                            (now_ms - state.last_measurements_ms):toint() or -1
    gcs:send_text(MAV_SEVERITY.DEBUG,
                  string.format("PYLON: SOC age:%dms measurement age:%dms",
                                soc_age, measurement_age))
end

local function update()
    local now_ms = millis()
    send_heartbeat(now_ms)
    read_frames(now_ms)
    publish_battery(now_ms)
    send_debug(now_ms)
    return update, UPDATE_PERIOD_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "PYLON: CAN battery monitor started")
return update()
