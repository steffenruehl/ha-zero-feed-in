# Lovelace Dashboards

This repository includes three pre-built Lovelace dashboards for monitoring and debugging the Zero Feed-In system.

## Quick Setup

1. In Home Assistant, go to **Dashboards** → Create a new dashboard (or edit existing)
2. Switch to **YAML edit mode** (top-right menu)
3. Copy the entire contents of one of the dashboard YAML files from `config/`
4. Paste into the YAML editor
5. **Save** and view

### Replacing Entity IDs

If you customized your sensor prefix (default: `sensor.zfi`), replace:
- `sensor.zfi_*` → your custom prefix + entity name

Example: if your prefix is `sensor.battery_*`, change `sensor.zfi_desired_power` to `sensor.battery_desired_power`.

---

## 1. Operations Dashboard

**File**: `config/lovelace_zfi_operations.yaml`

**Requirements**: None (works with normal and debug modes)

**Purpose**: Day-to-day operational overview. Shows the main control loop in action.

### What It Shows

- **Grid Status**: Current grid power (target ≈ +30W import)
- **Solar Surplus**: Estimated PV surplus available for charging
- **Battery State**: SOC (%) and actual battery power
- **Operating Mode**: Current mode (charging / discharging)
- **Controller Output**: Desired power from the direct calculation controller
- **Device Commands**: Limits sent to SolarFlow, relay state
- **SOC Limits**: Min/max guardrails, forecast-adjusted dynamic min SOC
- **Trends**: 24-hour history of control loop performance

### When to Use

- **Daily monitoring**: "Is the system working?"
- **Mode transitions**: "Why did it switch to charging/discharging?"
- **Checking target**: "Is grid power tracking the 30W target?"
- **Quick troubleshooting**: "What are the device limits right now?"

### Example Scenarios

**Scenario 1**: Grid power is +200W, desired power shows +150W
- Good: Controller is working, attempting to reduce grid draw
- Next step: Check if device commands are being sent

**Scenario 2**: Surplus shows -300W but desired power shows +100W
- Issue: Mode switching logic may have issues
- Next step: Enable debug, check Controller Debug dashboard

**Scenario 3**: Relay state changes frequently, grid power oscillates
- Issue: Possible tuning problem or relay lockout too short
- Next step: Check Relay SM Debug dashboard for lockout details

---

## 2. Controller Debug Dashboard

**File**: `config/lovelace_zfi_controller_debug.yaml`

**Requirements**: `debug: true` in `apps.yaml` → Controller section

**Purpose**: Deep dive into direct calculation controller behavior. Understanding why the controller chose a particular desired power.

### What It Shows

- **Error Signal**: Grid power vs target, actual error value
- **Muting State**: Whether the controller is muted after a recent command, and remaining muting time
- **Drift Accumulator**: Sub-hysteresis error accumulation
- **Desired Power**: Current output and decision reason
- **Operating Mode**: Current mode (charging / discharging) and 24-hour history
- **Guards**: SOC limits, surplus guards, direction switches

### Key Signals Explained

| Signal | Meaning |
|--------|---------|
| **Error** | (grid_power − target). Positive = net draw, negative = net feed-in |
| **Muting** | 1 = controller is waiting for device to respond, 0 = active |
| **Muting remaining** | Seconds until muting expires |
| **Drift accumulator** | Sub-hysteresis errors accumulated. Fires correction when exceeding hysteresis |
| **Target** | 30 W (discharge) or 0 W (charge). One of two fixed targets |
| **Reason** | Human-readable explanation of the last decision |

### Tuning Guidance

**If you see**: Desired power oscillates ±50W repeatedly
- **Root cause**: `muting` too short — controller reacts before device has responded
- **Fix**: Increase `muting` from 8 to 10–12 seconds

**If you see**: Grid slowly drifts and takes too long to correct
- **Root cause**: `ki` too low or `hysteresis` too high
- **Fix**: Raise `ki` toward 1.0 or lower `hysteresis` from 15 to 10 W

**If you see**: Drift accumulator grows but never fires a correction
- **Root cause**: Error keeps changing sign before drift exceeds hysteresis
- **Check**: Is the error oscillating around zero? Lower `hysteresis` if the drift never triggers

**If you see**: Sudden jump to ±800W desired power
- **Root cause**: Emergency clamping (feed-in detected > max_feed_in)
- **Check**: Did grid power exceed safe limits? Is SOC at max?

### When to Use

- **Tuning ki / hysteresis / muting**: Understand step response and steady-state behavior
- **Oscillation investigation**: "Why does the grid power wiggle?"
- **Mode flapping**: "Why does it keep switching modes?"
- **Muting validation**: "Is 8 s enough for the device to respond?"

---

## 3. Relay State Machine Debug Dashboard

**File**: `config/lovelace_zfi_relay_sm_debug.yaml`

**Requirements**: `debug: true` in `apps.yaml` → Driver section

**Purpose**: Relay transition behavior. Understanding why the relay does/doesn't switch.

### What It Shows

- **Current State**: IDLE, CHARGING, or DISCHARGING
- **Pending Target**: What state it's trying to reach
- **Lockout Progress**: Overall transition energy accumulation (%)
- **Direction-Specific Timers**: Charge/discharge/idle progress independently
- **Accumulated Energy**: W·s accumulated toward transition threshold
- **Physical Relay**: Current relay position from device
- **Integration History**: 24-hour state transitions and lockout activity

### Key Signals Explained

| Signal | Meaning |
|--------|---------|
| **SM State** | Current relay state. Drives the AC mode (Input/Output) |
| **SM Pending** | Target state being worked toward, or "none" |
| **Lockout %** | Progress toward completing the transition (0–100%) |
| **Accumulated W·s** | Energy integrated since start of transition |
| **Threshold W·s** | Total energy needed to complete transition |
| **Charge/Discharge/Idle %** | Direction-specific progress (independent per direction) |
| **Relay Locked** | `true` = SM or driver blocking writes, transition in progress |

### Lockout Timing Examples

With default config (`relay_lockout_ws=10000`, `relay_lockout_cutoff_w=25`):

| Desired Power | Lockout Time |
|---|---|
| 400 W sustained | ~25 s |
| 200 W sustained | ~50 s |
| 100 W sustained | ~100 s |
| 25 W or less | ~400 s (floor prevents infinite wait) |

The accumulator uses `max(|power|, 25W)`, so very low power levels take much longer.

### Troubleshooting Relay Issues

**If you see**: Relay switches very frequently (10+ times per minute)
- **Cause**: Lockout too short, desired power hovering near state boundary
- **Fix**: Increase `relay_lockout_ws` from 10000 to 15000–20000 W·s

**If you see**: Relay won't switch even though desired power clearly calls for it
- **Cause**: Lockout still accumulating, idle lockout active (90s holdoff)
- **Check**: Is lockout_pct < 100%? Is idle_pct < 100%?
- **Fix**: If idle is the issue, lower `relay_lockout_idle_s` to 60s

**If you see**: Relay switches, then immediately switches back
- **Cause**: Desired power is right at the state boundary, relay noise causes oscillation
- **Fix**: Increase `mode_hysteresis` in controller from 50W to 100W

**If you see**: Relay stuck in transition, lockout_pct frozen at 99%
- **Cause**: Desired power dropped below cutoff (25W) before accumulation finished
- **Check**: Did desired power fall to near-zero? That resets the accumulator
- **Fix**: Check if this is expected behavior; if desired power is legitimately low, this is correct

### When to Use

- **Relay chatter investigation**: "Why 50 switches per day?"
- **Lockout tuning**: "How long should transitions take?"
- **Transition timing**: "When will the relay actually switch?"
- **State transitions history**: "What sequence did the relay go through?"

---

## FAQ

**Q: Do all dashboards require `debug: true`?**

A: No.
- **Operations** dashboard: works always
- **Controller Debug** and **Relay SM** dashboards: require `debug: true`

**Q: Can I have multiple dashboards active at once?**

A: Yes. Create separate dashboard tabs and add different YAML files to each.

**Q: What if my sensor names are different?**

A: Find-and-replace in the YAML:
- `sensor.zfi` → your prefix (e.g., `sensor.battery`)
- If using an unsigned battery sensor, also update the sensor ID for battery_power

**Q: The dashboard is missing data / shows "unavailable"**

A:
1. Verify sensors exist: In HA, go to **Developer Tools** → **States**, search for `zfi`
2. Verify `debug: true` is set if using a debug dashboard
3. Wait 5–10 minutes for sensors to populate after restart
4. Check AppDaemon logs for errors

**Q: Can I customize the dashboards?**

A: Absolutely. The YAML is standard Lovelace. You can:
- Add/remove cards
- Adjust `hours_to_show` for history graphs
- Change card order
- Recolor or rename entities
- Combine multiple dashboards into one

**Q: Why doesn't optional `heartbeat_mqtt_topic` appear?**

A: The Watchdog (ESP8266 independent safe-state) publishes to MQTT, not HA sensors. To monitor heartbeats in HA, check the native API entities that Watchdog creates (`heizung_zfi_controller_heartbeat_age`, etc.)

---

## Dashboard Comparison

| Feature | Operations | Controller Debug | Relay SM |
|---------|-----------|-----------------|----------|
| **Requires debug** | No | Yes | Yes |
| **Time graphs** | Yes | Yes | Yes |
| **Live entity cards** | No | Yes | Yes |
| **Controller tuning info** | Basic | Detailed | No |
| **Relay troubleshooting** | Basic | No | Detailed |
| **Forecast info** | Yes | No | No |
| **Self-contained** | Yes | Yes | Yes |

---

## Typical Workflow

### Initial Setup (Days 1–3)
1. Enable `debug: true`
2. Open **Operations** dashboard
3. Check that desired power roughly follows grid power with 30W offset
4. Look for mode transitions at sunrise/sunset

### Tuning (Weeks 1–4)
1. Enable **Controller Debug** dashboard alongside Operations
2. Adjust `ki`, `hysteresis`, and `muting` parameters
3. Check **Operations** for grid target tracking
4. Iterate until oscillations < 50W

### Relay Optimization (Weeks 2–6)
1. Open **Relay SM Debug** dashboard
2. Monitor relay switches per day
3. Adjust `relay_lockout_ws` if needed
4. Validate mode hysteresis prevents flapping

### Forecast Validation (Optional)
1. Check `sensor.zfi_dynamic_min_soc` updates at scheduled times
2. Confirm forecast-adjusted min SOC prevents unintended discharge

### Maintenance (Ongoing)
1. Disable `debug: true` to reduce HA sensor churn
2. Use **Operations** dashboard for monitoring
3. Re-enable `debug: true` only when troubleshooting
