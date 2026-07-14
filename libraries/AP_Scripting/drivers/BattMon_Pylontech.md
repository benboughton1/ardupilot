# Pylontech CAN Battery Monitor

This Lua driver reads a Pylontech battery using PYLON CANBUS Protocol v1.2 and publishes the result through ArduPilot's scripting battery monitor. It is intended for an 11-bit CAN bus running at 500 kbit/s.

The driver sends the required inverter heartbeat (`0x305`) once per second and consumes these battery frames:

| CAN ID | Data used |
|---:|---|
| `0x351` | Recommended charge voltage and charge/discharge current limits (logged) |
| `0x355` | State of charge and state of health |
| `0x356` | Battery voltage, current, and average cell temperature |
| `0x359` | Protection flags, alarm flags, and module count |
| `0x35C` | Charge/discharge enables and charge requests |
| `0x35E` | Manufacturer identification |

## CAN2 and Battery Setup

For an installation with EZKontrol motor controllers on CAN1 and the battery on physical CAN2, set:

```text
SCR_ENABLE       1
SCR_HEAP_SIZE    120000
CAN_P1_DRIVER    0
CAN_P2_DRIVER    2
CAN_P2_BITRATE   500000
CAN_D2_PROTOCOL  10
BATT_MONITOR     29
PYLON_ENABLE     1
PYLON_CANDRV     1
PYLON_BATT_IDX   1
PYLON_CURR_MUL   -1
PYLON_TIMEOUT    3000
```

`PYLON_CANDRV` selects an ArduPilot scripting protocol, not a physical CAN connector. Value `1` selects `CAN_Dx_PROTOCOL=10`; value `2` selects `CAN_Dx_PROTOCOL=12`. In the configuration above, physical CAN2 is assigned to virtual driver 2, and virtual driver 2 runs scripting protocol 10, so `PYLON_CANDRV=1` is correct.

`CAN_P1_DRIVER=0` leaves CAN1 unmanaged by `AP_CANManager` because the Agrover EZKontrol backend initializes CAN1 directly. Do not assign the Pylontech scripting driver to CAN1.

Set `BATT_CAPACITY` to the installed system's usable capacity in mAh. The script converts BMS SOC into consumed mAh using this value so ArduPilot capacity arming checks and failsafes remain meaningful.

Copy `BattMon_Pylontech.lua` to `APM/scripts` on the flight controller's SD card and reboot after changing the CAN, scripting, or battery monitor parameters.

### Optional Analog Redundancy

To keep the existing analog battery monitor as BATT2:

```text
BATT2_MONITOR 4
```

Move the existing analog voltage/current pin, multiplier, offset, and calibration values from `BATT_*` to their corresponding `BATT2_*` parameters. Confirm both monitors are healthy before arming; ArduPilot checks every configured battery monitor.

## Driver Parameters

| Parameter | Meaning |
|---|---|
| `PYLON_ENABLE` | Enables the script. |
| `PYLON_CANDRV` | Selects scripting protocol 10 (`1`) or Scripting2 protocol 12 (`2`). |
| `PYLON_BATT_IDX` | One-based ArduPilot battery instance to update. That instance must use monitor type 29. |
| `PYLON_CURR_MUL` | Current multiplier and polarity. The default `-1` assumes the BMS reports discharge current as negative. |
| `PYLON_TIMEOUT` | Maximum age of both `0x355` and `0x356` before the monitor becomes unhealthy. |
| `PYLON_DEBUG` | Enables a periodic frame-age status message. |

## Health and Failsafe Behavior

The monitor is healthy only when valid `0x355` and `0x356` frames have both arrived within `PYLON_TIMEOUT`. Malformed frames, extended frames, remote frames, error frames, out-of-range SOC/SOH, and implausible measurements do not refresh health.

An unhealthy monitor blocks arming through ArduPilot's normal battery checks. While armed, ArduPilot reports a battery-monitor communication failure but does not automatically stop the vehicle for an unhealthy monitor. Configure ordinary low/critical voltage and remaining-capacity failsafe actions separately.

Protection, alarm, enable, and charge-request changes are reported to the ground station. They are monitoring outputs only: this script does not command a charger, generator, contactor, emergency stop, or vehicle mode.

Additional dataflash messages are:

| Message | Contents |
|---|---|
| `PYLB` | Voltage, current, temperature, SOC, and SOH |
| `PYLL` | Recommended charge voltage and charge/discharge limits |
| `PYLF` | Raw protection, alarm, enable/request flags, and module count |

## Wiring and Commissioning

Before connecting the battery, confirm its connector pinout, CAN isolation/grounding requirements, and termination arrangement from the installed model's documentation. A conventional powered-off CAN bus with one 120-ohm terminator at each physical end measures approximately 60 ohms between CAN-H and CAN-L.

Perform commissioning with the drive disabled:

1. Verify a CAN analyzer sees standard ID `0x305`, DLC 8, eight zero bytes, once per second.
2. Confirm the battery transmits the six documented response IDs at approximately one-second intervals.
3. Compare voltage, temperature, SOC, and SOH with the battery's own display or service tool.
4. Apply a controlled discharge and confirm ArduPilot current is positive. Set `PYLON_CURR_MUL=1` if the installed firmware reports discharge as positive already.
5. Apply a controlled charge and verify the current sign reverses.
6. Disconnect CAN and verify BATT1 becomes unhealthy after `PYLON_TIMEOUT` and fails the pre-arm battery check.
7. If analog BATT2 is retained, compare both monitors over a controlled load cycle before relying on CAN as the primary source.

The protocol summary does not establish the installed battery model, firmware, connector pinout, current polarity, module aggregation behavior, or whether the heartbeat is required for contactor operation. Record those results for the rover after bench validation.
