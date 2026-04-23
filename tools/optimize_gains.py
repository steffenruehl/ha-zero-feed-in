#!/usr/bin/env python3
"""Optimize zero-feed-in controller gains using simulation replay.

Reads a plant model + disturbance traces (from identify_plant.py),
replays them through the real ControlLogic in closed loop, and
optimizes PI + deadband + FF gains to minimize feed-in.

Usage:
    # First, identify the plant:
    python3 tools/identify_plant.py data/controller_2026-04-21.csv data/controller_2026-04-22.csv

    # Then optimize (validates simulation first):
    python3 tools/optimize_gains.py

    # Or just validate without optimizing:
    python3 tools/optimize_gains.py --validate-only
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from src.zero_feed_in_controller import (
    Config,
    ControlLogic,
    FeedForwardSource,
    Measurement,
)
from src.zendure_solarflow_driver import (
    AdaptiveLockout,
    RelayStateMachine,
    RELAY_SWITCH_DELAY_S,
)


# ── Plant model ──────────────────────────────────────────────

IDENT_DT_S = 5.0
"""Sampling interval used during plant identification (seconds).

The discrete coefficients (a, b, c) were fitted from data filtered to
4–6 s ticks.  Continuous-time parameters are derived assuming this
nominal interval.
"""


class PlantModel:
    """First-order plant model with dt-aware discretisation.

    Stores continuous-time parameters (tau, dc_gain, c_cont) derived from
    the discrete coefficients (a, b, c) identified at ``IDENT_DT_S``.
    Each ``step()`` recomputes the discrete coefficients for the actual
    elapsed time, making the simulation robust to timing jitter.

    Continuous model:  τ · ḃ = K · desired + c_cont − batt
    Discrete at dt:    batt[k] = e^(−dt/τ) · batt[k−1]
                                + K(1−e^(−dt/τ)) · desired[k]
                                + c_cont(1−e^(−dt/τ))
    """

    def __init__(self, a: float, b: float, c: float, delay_steps: int = 0):
        self.a_nominal = a
        self.b_nominal = b
        self.c_nominal = c
        # Derive continuous-time parameters from identification at IDENT_DT_S.
        self.tau: float = -IDENT_DT_S / np.log(a) if 0 < a < 1 else float("inf")
        self.dc_gain: float = b / (1 - a) if abs(1 - a) > 1e-9 else float("inf")
        self.c_cont: float = c / (1 - a) if abs(1 - a) > 1e-9 else 0.0
        self.delay_steps: int = delay_steps
        self._delay_buf: list[float] = [0.0] * delay_steps
        self.battery_w: float = 0.0

    def step(self, desired_w: float, dt: float = IDENT_DT_S) -> float:
        """Advance plant state by *dt* seconds.

        When ``delay_steps > 0``, the plant uses the desired value from
        *d* calls ago (pure transport delay), modelling the device's
        command-to-effect latency.
        """
        if self.delay_steps > 0:
            effective = self._delay_buf[0]
            self._delay_buf.pop(0)
            self._delay_buf.append(desired_w)
        else:
            effective = desired_w
        alpha = math.exp(-dt / self.tau)
        self.battery_w = (
            alpha * self.battery_w
            + self.dc_gain * (1 - alpha) * effective
            + self.c_cont * (1 - alpha)
        )
        return self.battery_w

    def reset(self, battery_w: float = 0.0) -> None:
        """Reset battery state and delay buffer."""
        self.battery_w = battery_w
        self._delay_buf = [0.0] * self.delay_steps


# ── Pre-converted trace arrays (set once in main) ────────────


@dataclass
class TraceArrays:
    """Pre-converted numpy arrays from the raw JSON traces.

    Created once in ``main()`` and reused for every simulation/cost
    evaluation, avoiding repeated ``np.array()`` conversions.
    """

    disturbance: np.ndarray
    soc: np.ndarray
    dt: np.ndarray
    grid_actual: np.ndarray
    n: int


# ── Gain vector ↔ Config mapping ─────────────────────────────


# Parameter order in optimization vector:
#  0: kp_discharge_up      4: ki_discharge_up
#  1: kp_discharge_down    5: ki_discharge_down
#  2: kp_charge_up         6: ki_charge_up
#  3: kp_charge_down       7: ki_charge_down
#  8: deadband_w
#  9: ff_filter_tau_s     10: ff_deadband_w
# 11: ff_gain_pv          12: ff_gain_load

PARAM_NAMES = [
    "kp_discharge_up", "kp_discharge_down", "kp_charge_up", "kp_charge_down",
    "ki_discharge_up", "ki_discharge_down", "ki_charge_up", "ki_charge_down",
    "deadband_w",
    "ff_filter_tau_s", "ff_deadband_w",
    "ff_gain_pv", "ff_gain_load",
    "relay_lockout_ws", "relay_lockout_cutoff_w",
    "relay_lockout_idle_s", "min_active_power_w",
]

BOUNDS = [
    (0.01, 0.8),  # kp_discharge_up
    (0.05, 0.8),  # kp_discharge_down
    (0.05, 0.8),  # kp_charge_up
    (0.05, 0.8),  # kp_charge_down
    (0.001, 0.3),  # ki_discharge_up
    (0.005, 0.3),  # ki_discharge_down
    (0.005, 0.3),  # ki_charge_up
    (0.005, 0.3),  # ki_charge_down
    (5, 60),       # deadband_w
    (3, 120),      # ff_filter_tau_s
    (0, 60),       # ff_deadband_w
    (0.1, 1.5),   # ff_gain_pv
    (0.1, 1.5),   # ff_gain_load
    (500, 20000),  # relay_lockout_ws
    (10, 200),     # relay_lockout_cutoff_w
    (10, 300),     # relay_lockout_idle_s
    (10, 100),     # min_active_power_w
]


@dataclass
class SystemConfig:
    """Non-optimized system parameters (fixed for a given installation)."""
    max_discharge_w: float = 1200.0
    max_charge_w: float = 800.0
    max_feed_in_w: float = 800.0
    min_soc_pct: float = 15.0
    max_soc_pct: float = 85.0
    discharge_target_w: float = 30.0
    charge_target_w: float = 0.0
    mode_hysteresis_w: float = 50.0
    charge_confirm_s: float = 20.0
    deadband_leak_ws: float = 250.0
    interval_s: int = 5
    delay_steps: int = 0
    """Command-to-effect transport delay in sampling steps.

    The device does not respond instantly to setpoint changes.
    A delay of d steps means the plant sees desired[k-d], not desired[k].
    Identified by `identify_plant.py` alongside (a, b, c).
    """
    ff_pv_entity: str = "sensor.pv"
    ff_load_entities: tuple[str, ...] = ("sensor.load",)


def params_to_config(x: np.ndarray, sys_cfg: SystemConfig) -> Config:
    """Build a Config from an optimization vector + fixed system config."""
    ff_sources = [
        FeedForwardSource(
            entity=sys_cfg.ff_pv_entity, gain=float(x[11]),
            sign=-1.0, name="pv",
        ),
    ]
    for entity in sys_cfg.ff_load_entities:
        ff_sources.append(FeedForwardSource(
            entity=entity, gain=float(x[12]),
            sign=1.0, name=entity.split(".")[-1],
        ))

    return Config(
        grid_sensor="sensor.grid",
        soc_sensor="sensor.soc",
        battery_power_sensor="sensor.battery",
        kp_discharge_up=float(x[0]),
        kp_discharge_down=float(x[1]),
        kp_charge_up=float(x[2]),
        kp_charge_down=float(x[3]),
        ki_discharge_up=float(x[4]),
        ki_discharge_down=float(x[5]),
        ki_charge_up=float(x[6]),
        ki_charge_down=float(x[7]),
        deadband_w=float(x[8]),
        deadband_leak_ws=sys_cfg.deadband_leak_ws,
        ff_enabled=True,
        ff_deadband_w=float(x[10]),
        ff_filter_tau_s=float(x[9]),
        ff_sources=tuple(ff_sources),
        max_discharge_w=sys_cfg.max_discharge_w,
        max_charge_w=sys_cfg.max_charge_w,
        max_feed_in_w=sys_cfg.max_feed_in_w,
        min_soc_pct=sys_cfg.min_soc_pct,
        max_soc_pct=sys_cfg.max_soc_pct,
        discharge_target_w=sys_cfg.discharge_target_w,
        charge_target_w=sys_cfg.charge_target_w,
        mode_hysteresis_w=sys_cfg.mode_hysteresis_w,
        charge_confirm_s=sys_cfg.charge_confirm_s,
        interval_s=sys_cfg.interval_s,
    )


# ── Simulation ───────────────────────────────────────────────


@dataclass
class SimResult:
    grid_w: np.ndarray
    desired_w: np.ndarray
    battery_w: np.ndarray
    p_term_w: np.ndarray       # proportional term per step (W)
    i_term_w: np.ndarray       # integral term per step (W)
    feed_in_energy_wh: float   # total energy fed into grid (Wh)
    iae: float                 # integral absolute error (W·s)
    max_feed_in_w: float       # worst-case feed-in spike (W)
    control_effort: float      # Σ|Δdesired| (W)
    relay_switches: int        # number of relay direction changes


def simulate(
    params: np.ndarray,
    plant_abc: tuple[float, float, float],
    traces: dict | TraceArrays,
    sys_cfg: SystemConfig,
) -> SimResult:
    """Run closed-loop simulation with given gains on disturbance trace.

    Uses ``simulate_fast`` internally (inlined PI loop for speed).
    Falls back to the full ``ControlLogic`` path when ``traces`` is a
    raw dict (for validation against real data where exact parity with
    the production controller matters).
    """
    if isinstance(traces, TraceArrays):
        return simulate_fast(params, plant_abc, traces, sys_cfg)
    return _simulate_full(params, plant_abc, traces, sys_cfg)


def _simulate_full(
    params: np.ndarray,
    plant_abc: tuple[float, float, float],
    traces: dict,
    sys_cfg: SystemConfig,
) -> SimResult:
    """Slow but exact simulation using the real ControlLogic class.

    Feed-forward uses the combined disturbance signal as a synthetic PV
    reading (load entities left absent → contribute 0).  This exercises
    ``ff_gain_pv`` exactly; ``ff_gain_load`` is not exercised here but
    *is* exercised in ``simulate_fast`` via asymmetric gain selection.

    Includes a relay state machine that gates direction changes behind
    an adaptive energy lockout, matching the production driver.
    """
    cfg = params_to_config(params, sys_cfg)
    logic = ControlLogic(cfg, log=None)
    logic.seed(None)
    plant = PlantModel(*plant_abc, delay_steps=sys_cfg.delay_steps)

    # Relay state machine (uses real driver classes).
    relay_lockout_ws = float(params[13])
    relay_lockout_cutoff_w = float(params[14])
    relay_lockout_idle_s = float(params[15])
    min_active_power_w = float(params[16])

    sm = RelayStateMachine(
        idle_lockout_s=relay_lockout_idle_s,
        charge_lockout=AdaptiveLockout(cutoff_w=relay_lockout_cutoff_w),
        discharge_lockout=AdaptiveLockout(cutoff_w=relay_lockout_cutoff_w),
        threshold_ws=relay_lockout_ws,
        min_active_power_w=min_active_power_w,
    )

    disturbance = np.array(traces["disturbance_w"])
    soc = np.array(traces["soc_pct"])
    dt_arr = np.array(traces["dt_s"])
    n = len(disturbance)

    grid_out = np.empty(n)
    desired_out = np.empty(n)
    battery_out = np.empty(n)
    p_term_out = np.empty(n)
    i_term_out = np.empty(n)
    now = 0.0
    relay_switches = 0
    relay_locked_until = 0.0

    for k in range(n):
        grid_w = disturbance[k] - plant.battery_w
        grid_out[k] = grid_w

        # Relay locked feedback → freeze integral in controller.
        relay_locked = now < relay_locked_until

        # Synthetic FF reading: feed disturbance as the PV entity so
        # FeedForward reacts to PV-dominated disturbance ramps.
        ff_readings = {sys_cfg.ff_pv_entity: float(disturbance[k])}

        m = Measurement(
            grid_power_w=grid_w,
            soc_pct=float(soc[k]),
            battery_power_w=plant.battery_w,
            ff_readings=ff_readings,
            relay_locked=relay_locked,
        )

        output = logic.compute(m, now=now)
        desired_w = output.desired_power_w

        # Pass through relay SM.
        old_state = sm.state
        allowed_w = sm.update(desired_w, now)
        if sm.state != old_state:
            relay_switches += 1
            relay_locked_until = now + RELAY_SWITCH_DELAY_S

        desired_out[k] = allowed_w
        p_term_out[k] = output.p_term
        i_term_out[k] = output.i_term
        step_dt = float(dt_arr[k]) if k + 1 < n else IDENT_DT_S
        battery_out[k] = plant.step(allowed_w, step_dt)
        now += step_dt

    # Metrics
    target = np.where(
        grid_out > sys_cfg.discharge_target_w,
        sys_cfg.discharge_target_w,
        sys_cfg.charge_target_w,
    )
    error = grid_out - target
    iae = float(np.sum(np.abs(error) * dt_arr))

    feed_in_mask = grid_out < 0
    feed_in_power = np.where(feed_in_mask, -grid_out, 0.0)
    feed_in_energy_wh = float(np.sum(feed_in_power * dt_arr) / 3600)
    max_feed_in = float(np.max(feed_in_power)) if feed_in_mask.any() else 0.0

    effort = float(np.sum(np.abs(np.diff(desired_out))))

    return SimResult(
        grid_w=grid_out,
        desired_w=desired_out,
        battery_w=battery_out,
        p_term_w=p_term_out,
        i_term_w=i_term_out,
        feed_in_energy_wh=feed_in_energy_wh,
        iae=iae,
        max_feed_in_w=max_feed_in,
        control_effort=effort,
        relay_switches=relay_switches,
    )


# Mode constants for the fast simulation (avoid Enum overhead).
_MODE_DISCHARGING = 0
_MODE_CHARGING = 1


def simulate_fast(
    params: np.ndarray,
    plant_abc: tuple[float, float, float],
    ta: TraceArrays,
    sys_cfg: SystemConfig,
) -> SimResult:
    """Fast closed-loop simulation with inlined PI logic.

    Replicates the exact behavior of ``ControlLogic.compute()``
    including feed-forward, but avoids per-step overhead: no dataclass
    creation, no method dispatch, no Enum.

    Feed-forward uses the combined disturbance signal as a single source
    with ``sign=-1`` and ``ff_gain_pv``.  This matches ``_simulate_full``
    where the disturbance is fed as a synthetic PV reading.
    ``ff_gain_load`` is not exercised because the trace only contains the
    combined disturbance (PV − load), not individual signals.

    ~4-5× faster than ``_simulate_full`` on 33 K timesteps.
    """
    # Unpack params to locals (avoid repeated array indexing).
    kp_discharge_up = float(params[0])
    kp_discharge_down = float(params[1])
    kp_charge_up = float(params[2])
    kp_charge_down = float(params[3])
    ki_discharge_up = float(params[4])
    ki_discharge_down = float(params[5])
    ki_charge_up = float(params[6])
    ki_charge_down = float(params[7])
    deadband_w = float(params[8])
    ff_tau = float(params[9])
    ff_deadband = float(params[10])
    ff_gain_pv = float(params[11])
    ff_gain_load = float(params[12])
    relay_lockout_ws = float(params[13])
    relay_lockout_cutoff_w = float(params[14])
    relay_lockout_idle_s = float(params[15])
    min_active_power_w = float(params[16])

    # Fixed system params.
    max_discharge_w = sys_cfg.max_discharge_w
    max_charge_w = sys_cfg.max_charge_w
    max_feed_in_w = sys_cfg.max_feed_in_w
    discharge_target = sys_cfg.discharge_target_w
    charge_target = sys_cfg.charge_target_w
    hysteresis = sys_cfg.mode_hysteresis_w
    charge_confirm = sys_cfg.charge_confirm_s
    min_soc = sys_cfg.min_soc_pct
    max_soc = sys_cfg.max_soc_pct
    deadband_leak = sys_cfg.deadband_leak_ws
    interval_s = sys_cfg.interval_s
    emergency_margin = 50.0  # EMERGENCY_SAFETY_MARGIN_W

    # Derive continuous-time plant parameters from discrete (a, b, c)
    # identified at IDENT_DT_S, then precompute per-step coefficients.
    pa, pb, pc = plant_abc
    tau = -IDENT_DT_S / np.log(pa) if 0 < pa < 1 else float("inf")
    dc_gain = pb / (1 - pa) if abs(1 - pa) > 1e-9 else float("inf")
    c_cont = pc / (1 - pa) if abs(1 - pa) > 1e-9 else 0.0

    # Trace arrays — convert to Python lists for fast per-element access
    # (avoids numpy scalar overhead and keeps all loop arithmetic in
    # native Python floats, which is ~3× faster than numpy scalars).
    n = ta.n
    disturbance = ta.disturbance.tolist()
    soc = ta.soc.tolist()
    dt_l: list[float] = ta.dt.tolist()
    dt_arr = ta.dt  # keep numpy array for vectorised metrics at end

    # Per-step plant coefficients accounting for timing jitter.
    plant_dt = dt_arr.copy()
    plant_dt[-1] = IDENT_DT_S          # last step: no next measurement
    alpha = np.exp(-plant_dt / tau)
    pa_l: list[float] = alpha.tolist()
    pb_l: list[float] = (dc_gain * (1.0 - alpha)).tolist()
    pc_l: list[float] = (c_cont * (1.0 - alpha)).tolist()

    # Output arrays.
    grid_out = np.empty(n)
    desired_out = np.empty(n)
    battery_out = np.empty(n)
    p_term_out = np.empty(n)
    i_term_out = np.empty(n)

    # Controller state.
    mode = _MODE_DISCHARGING
    integral = 0.0
    last_computed = 0.0
    battery_w = 0.0
    charge_pending_since: float | None = None
    db_acc = 0.0
    now = 0.0
    ff_ema: float | None = None  # feed-forward EMA state
    surplus_ema: float | None = None  # surplus clamp EMA (τ=10s)

    # Relay SM state (inlined from RelayStateMachine for speed).
    # States: 0=IDLE, 1=CHARGING, 2=DISCHARGING
    _SM_IDLE = 0
    _SM_CHARGING = 1
    _SM_DISCHARGING = 2
    _DIRECTION_THRESHOLD = 10.0
    _RELAY_SWITCH_DELAY_S = 8.0

    sm_state = _SM_IDLE
    sm_charge_acc_ws = 0.0   # adaptive lockout accumulator for CHARGING
    sm_discharge_acc_ws = 0.0  # adaptive lockout accumulator for DISCHARGING
    sm_idle_acc_s = 0.0      # time accumulator for IDLE transition
    sm_charge_last_t = 0.0
    sm_discharge_last_t = 0.0
    sm_idle_last_t = 0.0
    sm_departure_since = 0.0
    relay_switches = 0
    relay_locked_until = 0.0

    # Transport delay buffer (FIFO: oldest at index 0).
    delay_d = sys_cfg.delay_steps
    delay_buf = [0.0] * delay_d

    # Inline relay SM update as a local function (closure over SM state).
    # This avoids class/method overhead while keeping the logic readable.
    def _sm_update(desired_w: float) -> float:
        """Apply relay SM: returns allowed power, may change sm_state."""
        nonlocal sm_state, sm_charge_acc_ws, sm_discharge_acc_ws
        nonlocal sm_idle_acc_s, sm_charge_last_t, sm_discharge_last_t
        nonlocal sm_idle_last_t, sm_departure_since, relay_switches
        nonlocal relay_locked_until

        # Classify desired direction.
        if desired_w > _DIRECTION_THRESHOLD:
            sm_target = _SM_DISCHARGING
        elif desired_w < -_DIRECTION_THRESHOLD:
            sm_target = _SM_CHARGING
        else:
            sm_target = _SM_IDLE

        if sm_target == sm_state:
            # Stable — reset all transition accumulators.
            sm_charge_acc_ws = 0.0
            sm_discharge_acc_ws = 0.0
            sm_idle_acc_s = 0.0
            sm_charge_last_t = 0.0
            sm_discharge_last_t = 0.0
            sm_idle_last_t = 0.0
            sm_departure_since = 0.0
        else:
            # Track departure time for safety timeout.
            if sm_departure_since == 0.0:
                sm_departure_since = now

            # Tick only the target's accumulator.
            can_switch = False
            if sm_target == _SM_IDLE:
                if sm_idle_last_t > 0.0:
                    idt = now - sm_idle_last_t
                    if idt > 0:
                        sm_idle_acc_s += idt
                sm_idle_last_t = now
                sm_charge_last_t = 0.0
                sm_discharge_last_t = 0.0
                can_switch = sm_idle_acc_s >= relay_lockout_idle_s
            elif sm_target == _SM_CHARGING:
                if sm_charge_last_t > 0.0:
                    cdt = now - sm_charge_last_t
                    if cdt > 0:
                        eff_w = max(abs(desired_w), relay_lockout_cutoff_w)
                        sm_charge_acc_ws += eff_w * cdt
                sm_charge_last_t = now
                sm_discharge_last_t = 0.0
                sm_idle_last_t = 0.0
                can_switch = sm_charge_acc_ws >= relay_lockout_ws
            elif sm_target == _SM_DISCHARGING:
                if sm_discharge_last_t > 0.0:
                    ddt = now - sm_discharge_last_t
                    if ddt > 0:
                        eff_w = max(abs(desired_w), relay_lockout_cutoff_w)
                        sm_discharge_acc_ws += eff_w * ddt
                sm_discharge_last_t = now
                sm_charge_last_t = 0.0
                sm_idle_last_t = 0.0
                can_switch = sm_discharge_acc_ws >= relay_lockout_ws

            # Safety cap.
            if not can_switch and (now - sm_departure_since) >= 600.0:
                can_switch = True

            if can_switch:
                sm_state = sm_target
                # Reset all accumulators.
                sm_charge_acc_ws = 0.0
                sm_discharge_acc_ws = 0.0
                sm_idle_acc_s = 0.0
                sm_charge_last_t = 0.0
                sm_discharge_last_t = 0.0
                sm_idle_last_t = 0.0
                sm_departure_since = 0.0
                relay_switches += 1
                relay_locked_until = now + _RELAY_SWITCH_DELAY_S

        # Clamp to current relay state (inlined _sm_clamp).
        if sm_state == _SM_IDLE:
            return 0.0
        if sm_state == _SM_CHARGING:
            return min(-min_active_power_w, desired_w)
        # DISCHARGING
        return max(min_active_power_w, desired_w)

    for k in range(n):
        # ── Grid measurement ──
        grid_w = disturbance[k] - battery_w
        grid_out[k] = grid_w
        soc_k = soc[k]

        # ── Elapsed time ──
        if k > 0:
            dt = dt_l[k - 1]
            if dt > interval_s * 3:
                dt = interval_s * 3
            elif dt < 0:
                dt = 0.0
        else:
            dt = interval_s
        now_next = now + dt

        surplus = -battery_w - grid_w

        # ── Surplus EMA (filter for clamp, τ=10s) ──
        _surplus_tau = 10.0
        _surplus_alpha = dt / (_surplus_tau + dt)
        if surplus_ema is None:
            surplus_ema = surplus
        else:
            surplus_ema = _surplus_alpha * surplus + (1.0 - _surplus_alpha) * surplus_ema

        # ── Relay locked? (freeze integral during relay switch delay) ──
        relay_locked = now < relay_locked_until

        # ── Emergency protection ──
        feed_in = -grid_w if grid_w < 0 else 0.0
        if feed_in > max_feed_in_w:
            excess = feed_in - max_feed_in_w
            forced = last_computed - excess - emergency_margin
            if forced < 0.0:
                forced = 0.0
            integral = forced
            last_computed = forced

            # Pass emergency output through relay SM.
            forced = _sm_update(forced)

            desired_out[k] = forced
            p_term_out[k] = 0.0
            i_term_out[k] = integral
            if delay_d > 0:
                effective = delay_buf[0]
                del delay_buf[0]
                delay_buf.append(forced)
            else:
                effective = forced
            battery_w = pa_l[k] * battery_w + pb_l[k] * effective + pc_l[k]
            battery_out[k] = battery_w
            # Update FF EMA even during emergency (match production).
            _alpha_ff = dt / (ff_tau + dt)
            if ff_ema is None:
                ff_ema = disturbance[k]
            else:
                ff_ema = _alpha_ff * disturbance[k] + (1 - _alpha_ff) * ff_ema
            now = now_next
            continue

        # ── Mode switching (Schmitt trigger) ──
        old_mode = mode
        if mode == _MODE_DISCHARGING:
            if surplus > hysteresis:
                if charge_pending_since is None:
                    charge_pending_since = now
                elif now - charge_pending_since >= charge_confirm:
                    mode = _MODE_CHARGING
                    charge_pending_since = None
            else:
                charge_pending_since = None
        else:  # CHARGING
            if surplus < -hysteresis:
                mode = _MODE_DISCHARGING

        if mode != old_mode:
            charge_pending_since = None
            db_acc = 0.0
            if mode == _MODE_CHARGING:
                integral = -surplus if -surplus > -max_charge_w else -max_charge_w
            else:
                integral = 0.0

        # ── Target ──
        target = charge_target if mode == _MODE_CHARGING else discharge_target
        error = grid_w - target

        # ── PI with deadband ──
        abs_error = error if error >= 0 else -error
        in_deadband = False
        p_term = 0.0
        new_integral = integral

        if abs_error <= deadband_w:
            in_deadband = True
            if deadband_leak > 0:
                db_acc += error * dt
                abs_db = db_acc if db_acc >= 0 else -db_acc
                if abs_db >= deadband_leak:
                    db_acc = 0.0
                    in_deadband = False  # leak fired — run PI below

        if in_deadband:
            # Frozen: hold last output, integral unchanged, p_term = 0.
            pi_out = last_computed
            new_integral = integral  # don't integrate when in deadband
        else:
            if abs_error > deadband_w:
                db_acc = 0.0
            # Gain selection (4-quadrant)
            if mode == _MODE_DISCHARGING:
                if error >= 0:
                    kp, ki = kp_discharge_up, ki_discharge_up
                else:
                    kp, ki = kp_discharge_down, ki_discharge_down
            else:
                if error < 0:
                    kp, ki = kp_charge_up, ki_charge_up
                else:
                    kp, ki = kp_charge_down, ki_charge_down

            p_term = kp * error
            new_integral = integral + ki * error * dt
            pi_out = p_term + new_integral

            # PI anti-windup
            if pi_out > max_discharge_w:
                new_integral = max_discharge_w - p_term
                pi_out = max_discharge_w
            elif pi_out < -max_charge_w:
                new_integral = -max_charge_w - p_term
                pi_out = -max_charge_w

        # ── Feed-forward (filtered derivative of disturbance) ──
        ff_out = 0.0
        if ff_ema is not None:
            alpha_ff = dt / (ff_tau + dt)
            ff_delta = alpha_ff * (disturbance[k] - ff_ema)
            raw_ff = -ff_gain_pv * ff_delta
            ff_out = raw_ff if abs(raw_ff) >= ff_deadband else 0.0

        desired_w = pi_out + ff_out

        # ── Guards (always checked) ──
        guard_fired = False
        if desired_w > 0 and soc_k <= min_soc:
            desired_w = 0.0
            integral = 0.0
            guard_fired = True
        elif desired_w < 0:
            if soc_k >= max_soc:
                desired_w = 0.0
                # Transient guard — freeze integral to avoid large
                # transients when surplus returns.
                guard_fired = True

        if not guard_fired:
            integral = new_integral

            # ── Clamp (always applied) ──
            clamped = desired_w
            if desired_w > 0:
                if desired_w > max_discharge_w:
                    clamped = max_discharge_w
            elif desired_w < 0:
                max_safe_charge = surplus_ema if surplus_ema > 0 else 0.0
                limit = -max_charge_w if max_charge_w < max_safe_charge else -max_safe_charge
                if desired_w < limit:
                    clamped = limit

            desired_w = clamped

        last_computed = desired_w

        # ── Relay SM (inlined adaptive lockout) ──
        allowed_w = _sm_update(desired_w)

        desired_out[k] = allowed_w
        p_term_out[k] = p_term
        i_term_out[k] = integral
        if delay_d > 0:
            effective = delay_buf[0]
            del delay_buf[0]
            delay_buf.append(allowed_w)
        else:
            effective = allowed_w
        battery_w = pa_l[k] * battery_w + pb_l[k] * effective + pc_l[k]
        battery_out[k] = battery_w
        # Update FF EMA (always, matching production update_previous).
        _alpha_ff = dt / (ff_tau + dt)
        if ff_ema is None:
            ff_ema = disturbance[k]
        else:
            ff_ema = _alpha_ff * disturbance[k] + (1 - _alpha_ff) * ff_ema
        now = now_next

    # ── Metrics (vectorised) ──
    target_arr = np.where(
        grid_out > discharge_target,
        discharge_target,
        charge_target,
    )
    err_arr = grid_out - target_arr
    iae = float(np.sum(np.abs(err_arr) * dt_arr))

    feed_in_power = np.where(grid_out < 0, -grid_out, 0.0)
    feed_in_energy_wh = float(np.sum(feed_in_power * dt_arr) / 3600)
    max_feed_in_val = float(feed_in_power.max()) if feed_in_power.any() else 0.0

    effort = float(np.sum(np.abs(np.diff(desired_out))))

    return SimResult(
        grid_w=grid_out,
        desired_w=desired_out,
        battery_w=battery_out,
        p_term_w=p_term_out,
        i_term_w=i_term_out,
        feed_in_energy_wh=feed_in_energy_wh,
        iae=iae,
        max_feed_in_w=max_feed_in_val,
        control_effort=effort,
        relay_switches=relay_switches,
    )


# ── Cost function (module-level for pickling) ────────────────

# These globals are set in main() before optimization starts.
# Required because multiprocessing workers need picklable callables.
_plant_abc: tuple[float, float, float] = (0, 0, 0)
_traces: TraceArrays | dict = {}
_sys_cfg: SystemConfig = SystemConfig()


# ── Cost model (all terms in euros) ──────────────────────────

COST_FEED_IN_EUR_PER_WH = 0.20 / 1000   # €0.20/kWh = €0.0002/Wh
"""Feed-in: electricity purchased but exported to the grid."""

COST_RELAY_EUR_PER_SWITCH = 700.0 / 300_000  # €0.00233/switch
"""Relay wear: 300 000 switches = one battery replacement (€700)."""

COST_IAE_EUR_PER_WS = 0.20 / 3_600_000  # €0.20/kWh → €/W·s
"""Tracking error proxy: each W·s of absolute grid error ≈ misplaced
energy at electricity price.  Small vs feed-in to avoid double-counting."""

COST_EFFORT_EUR_PER_W = 1e-6
"""Control effort regulariser: minor penalty for actuator churn."""


def cost_euros(result: SimResult) -> float:
    """Compute total cost in euros from simulation metrics."""
    return (
        COST_FEED_IN_EUR_PER_WH * result.feed_in_energy_wh
        + COST_RELAY_EUR_PER_SWITCH * result.relay_switches
        + COST_IAE_EUR_PER_WS * result.iae
        + COST_EFFORT_EUR_PER_W * result.control_effort
    )


def cost(params: np.ndarray) -> float:
    """Optimization objective (euros).  Lower = better.

    Primary: minimize feed-in energy (€0.20/kWh).
    Secondary: minimize relay switches (€0.007/switch).
    Minor: tracking error + actuator smoothness.
    """
    result = simulate(params, _plant_abc, _traces, _sys_cfg)
    return cost_euros(result)


def _init_worker(plant_abc, traces: TraceArrays, sys_cfg):
    """Initialize globals in worker processes."""
    global _plant_abc, _traces, _sys_cfg
    _plant_abc = plant_abc
    _traces = traces
    _sys_cfg = sys_cfg


# ── Validation ───────────────────────────────────────────────


def validate(params: np.ndarray, plant_abc, traces: dict,
             sys_cfg: SystemConfig):
    """Compare simulation with current gains against actual logged data."""
    result = simulate(params, plant_abc, traces, sys_cfg)
    actual_grid = np.array(traces["grid_actual_w"])

    valid_mask = np.isfinite(actual_grid) & np.isfinite(result.grid_w)
    if valid_mask.sum() < 100:
        print("  WARNING: too few valid points for validation")
        return False

    sim = result.grid_w[valid_mask]
    act = actual_grid[valid_mask]

    r = np.corrcoef(sim, act)[0, 1]
    rmse = np.sqrt(np.mean((sim - act) ** 2))
    mae = np.mean(np.abs(sim - act))
    bias = np.mean(sim - act)

    dt_arr = np.array(traces["dt_s"])
    act_feedin = float(np.sum(np.where(actual_grid < 0, -actual_grid, 0) * dt_arr) / 3600)

    print(f"  Correlation (r):   {r:.4f}")
    print(f"  RMSE:              {rmse:.1f} W")
    print(f"  MAE:               {mae:.1f} W")
    print(f"  Bias (sim - act):  {bias:.1f} W")
    print(f"  Feed-in energy:    sim={result.feed_in_energy_wh:.1f} Wh  actual={act_feedin:.1f} Wh")

    ok = r > 0.6
    if not ok:
        print("\n  !! Simulation correlation too low (< 0.6) — model may not be reliable!")
        print("     Consider using more data or checking for config changes in the log period.")
    else:
        print("  >> Simulation tracks real data well enough for optimization.")
    return ok


# ── Sensitivity / stability analysis ─────────────────────────


def sensitivity_analysis(optimized: np.ndarray, plant_abc, traces: dict | TraceArrays,
                         sys_cfg: SystemConfig, baseline_cost: float):
    """Perturb each parameter by +/-5% and +/-20% and measure cost change."""
    print("\n  Parameter               -20%     -5%   nominal    +5%    +20%")
    print("  " + "-" * 68)

    # Collect ±5% costs for reuse in the ranking below.
    cost_at_neg5: list[float] = []
    cost_at_pos5: list[float] = []

    for i, name in enumerate(PARAM_NAMES):
        costs = []
        for pct in [-0.20, -0.05, 0.0, 0.05, 0.20]:
            perturbed = optimized.copy()
            perturbed[i] *= (1 + pct)
            # Clamp to bounds
            lo, hi = BOUNDS[i]
            perturbed[i] = np.clip(perturbed[i], lo, hi)
            r = simulate(perturbed, plant_abc, traces, sys_cfg)
            c = cost_euros(r)
            costs.append(c)

        cost_at_neg5.append(costs[1])  # -5% is index 1
        cost_at_pos5.append(costs[3])  # +5% is index 3

        # Show as % change from nominal
        nom = costs[2]
        parts = []
        for j, c in enumerate(costs):
            if j == 2:
                parts.append(f"{c:8.1f}")
            else:
                delta = (c - nom) / nom * 100 if nom != 0 else 0
                arrow = "+" if delta > 0 else ""
                parts.append(f"{arrow}{delta:5.1f}%")
        print(f"  {name:<22s}  {'  '.join(parts)}")

    # Classify sensitivity — reuse ±5% costs already computed above.
    print("\n  Sensitivity ranking (cost increase from +/-5% perturbation):")
    sensitivities = []
    for i, name in enumerate(PARAM_NAMES):
        worst = max(abs(cost_at_neg5[i] - baseline_cost),
                    abs(cost_at_pos5[i] - baseline_cost))
        sensitivities.append((name, worst))

    sensitivities.sort(key=lambda x: x[1], reverse=True)
    for name, delta in sensitivities:
        pct = delta / baseline_cost * 100 if baseline_cost != 0 else 0
        bar = "#" * min(40, int(pct * 4))
        label = "HIGH" if pct > 5 else "medium" if pct > 1 else "low"
        print(f"    {name:<22s}  {pct:5.2f}%  {bar}  [{label}]")


# ── Progress tracking ────────────────────────────────────────


# Default checkpoint path (next to the plant model in data/).
DEFAULT_CHECKPOINT = Path("data/optimizer_checkpoint.json")

# Default optimization result path.
DEFAULT_RESULT = Path("data/optimization_result.json")


def save_checkpoint(
    path: Path,
    best_x: np.ndarray,
    best_cost: float,
    gen: int,
    baseline_cost: float,
    baseline_feedin: float,
    current_gains: np.ndarray,
) -> None:
    """Persist optimizer state to JSON so the run can be resumed.

    Saves the best parameter vector found so far.  On resume, a new
    population is seeded around ``best_x`` (warm restart).
    """
    data = {
        "generation": gen,
        "best_cost": best_cost,
        "baseline_cost": baseline_cost,
        "baseline_feedin": baseline_feedin,
        "best_x": best_x.tolist(),
        "current_gains": current_gains.tolist(),
        "param_names": PARAM_NAMES,
        "bounds": BOUNDS,
    }
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    tmp.rename(path)  # atomic on POSIX


def load_checkpoint(path: Path) -> dict:
    """Load a previously saved checkpoint."""
    with open(path) as f:
        return json.load(f)


def save_result(
    path: Path,
    *,
    optimized_params: np.ndarray,
    current_params: np.ndarray,
    baseline: SimResult,
    optimized: SimResult,
    baseline_cost: float,
    optimized_cost: float,
    plant_abc: tuple[float, float, float],
    sys_cfg: SystemConfig,
    generations: int,
    converged: bool,
    elapsed_s: float,
) -> None:
    """Write structured optimization result to JSON.

    This is the canonical output consumed by ``plot_response.py`` and
    other downstream tools.  The checkpoint file is for resume only.
    """
    def _metrics(r: SimResult) -> dict:
        """Extract scalar metrics from a SimResult."""
        return {
            "feed_in_energy_wh": r.feed_in_energy_wh,
            "max_feed_in_w": r.max_feed_in_w,
            "iae": r.iae,
            "control_effort": r.control_effort,
            "relay_switches": r.relay_switches,
        }

    params_dict = {
        name: float(optimized_params[i]) for i, name in enumerate(PARAM_NAMES)
    }
    current_dict = {
        name: float(current_params[i]) for i, name in enumerate(PARAM_NAMES)
    }

    data = {
        "version": 1,
        "optimized_params": params_dict,
        "current_params": current_dict,
        "param_vector": optimized_params.tolist(),
        "current_vector": current_params.tolist(),
        "bounds": {name: list(BOUNDS[i]) for i, name in enumerate(PARAM_NAMES)},
        "baseline_metrics": _metrics(baseline),
        "optimized_metrics": _metrics(optimized),
        "baseline_cost": baseline_cost,
        "optimized_cost": optimized_cost,
        "improvement_pct": {
            "feed_in": (optimized.feed_in_energy_wh - baseline.feed_in_energy_wh)
            / baseline.feed_in_energy_wh * 100
            if baseline.feed_in_energy_wh else 0.0,
            "cost": (optimized_cost - baseline_cost) / baseline_cost * 100
            if baseline_cost else 0.0,
        },
        "plant": {
            "a": plant_abc[0], "b": plant_abc[1], "c": plant_abc[2],
            "ident_dt_s": IDENT_DT_S,
            "tau_s": -IDENT_DT_S / np.log(plant_abc[0])
            if 0 < plant_abc[0] < 1 else None,
            "dc_gain": plant_abc[1] / (1 - plant_abc[0])
            if abs(1 - plant_abc[0]) > 1e-9 else None,
            "c_cont": plant_abc[2] / (1 - plant_abc[0])
            if abs(1 - plant_abc[0]) > 1e-9 else None,
            "delay_steps": sys_cfg.delay_steps,
        },
        "system_config": {
            "max_discharge_w": sys_cfg.max_discharge_w,
            "max_charge_w": sys_cfg.max_charge_w,
            "max_feed_in_w": sys_cfg.max_feed_in_w,
            "min_soc_pct": sys_cfg.min_soc_pct,
            "max_soc_pct": sys_cfg.max_soc_pct,
            "discharge_target_w": sys_cfg.discharge_target_w,
            "charge_target_w": sys_cfg.charge_target_w,
            "delay_steps": sys_cfg.delay_steps,
        },
        "optimization": {
            "generations": generations,
            "converged": converged,
            "elapsed_s": round(elapsed_s, 1),
        },
    }
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    tmp.rename(path)


def load_result(path: Path) -> dict:
    """Load a previously saved optimization result."""
    with open(path) as f:
        return json.load(f)


class ProgressTracker:
    """Tracks, displays, and checkpoints optimization progress."""

    def __init__(
        self,
        baseline_cost: float,
        baseline_feedin: float,
        current_gains: np.ndarray,
        checkpoint_path: Path | None = None,
    ):
        self.baseline_cost = baseline_cost
        self.baseline_feedin = baseline_feedin
        self.current_gains = current_gains
        self.checkpoint_path = checkpoint_path
        self.t0 = time.time()
        self.gen = 0
        self.best_cost = baseline_cost
        self.best_feedin = baseline_feedin
        self.best_x: np.ndarray = current_gains.copy()
        self.history: list[tuple[int, float, float]] = []

    def callback(self, xk, convergence):
        """Display progress after each generation and save checkpoint."""
        self.gen += 1
        r = simulate(xk, _plant_abc, _traces, _sys_cfg)
        c = cost_euros(r)
        feedin = r.feed_in_energy_wh
        elapsed = time.time() - self.t0

        if c < self.best_cost:
            self.best_cost = c
            self.best_feedin = feedin
            self.best_x = np.array(xk)
        self.history.append((self.gen, c, feedin))

        cost_pct = (c - self.baseline_cost) / self.baseline_cost * 100
        feedin_pct = (feedin - self.baseline_feedin) / self.baseline_feedin * 100

        print(
            f"  gen {self.gen:3d}  "
            f"cost=€{c:.4f} ({cost_pct:+5.1f}%)  "
            f"feed-in={feedin:6.1f} Wh ({feedin_pct:+5.1f}%)  "
            f"best=€{self.best_cost:.4f}  "
            f"[{elapsed:5.0f}s]"
        )

        # Save checkpoint every generation.
        if self.checkpoint_path is not None:
            save_checkpoint(
                self.checkpoint_path,
                best_x=self.best_x,
                best_cost=self.best_cost,
                gen=self.gen,
                baseline_cost=self.baseline_cost,
                baseline_feedin=self.baseline_feedin,
                current_gains=self.current_gains,
            )


# ── Main ─────────────────────────────────────────────────────


def print_comparison(current: np.ndarray, optimized: np.ndarray):
    print("\n  Parameter               Current  Optimized  Change")
    print("  " + "-" * 55)
    for i, name in enumerate(PARAM_NAMES):
        cur, opt = current[i], optimized[i]
        pct = (opt - cur) / cur * 100 if cur != 0 else 0
        arrow = "^" if opt > cur else "v" if opt < cur else "="
        print(f"  {name:<22s}  {cur:7.4f}   {opt:7.4f}  {arrow} {abs(pct):5.1f}%")


def print_metrics(label: str, result: SimResult,
                  trace_days: float = 1.0) -> float:
    """Print simulation metrics with euro cost breakdown.

    *trace_days* normalises relay switches to a per-day rate so the
    displayed count is independent of trace length.
    """
    c = cost_euros(result)
    feed_eur = COST_FEED_IN_EUR_PER_WH * result.feed_in_energy_wh
    relay_eur = COST_RELAY_EUR_PER_SWITCH * result.relay_switches
    iae_eur = COST_IAE_EUR_PER_WS * result.iae
    effort_eur = COST_EFFORT_EUR_PER_W * result.control_effort
    sw_per_day = result.relay_switches / trace_days if trace_days > 0 else 0
    print(f"  {label}:")
    print(f"    Feed-in energy:  {result.feed_in_energy_wh:8.1f} Wh  "
          f"(€{feed_eur:.4f})")
    print(f"    Max feed-in:     {result.max_feed_in_w:8.0f} W")
    print(f"    Relay switches:  {result.relay_switches:8d}      "
          f"(€{relay_eur:.4f})  "
          f"= {sw_per_day:.0f}/day")
    print(f"    IAE:             {result.iae:11.0f} W*s  "
          f"(€{iae_eur:.4f})")
    print(f"    Control effort:  {result.control_effort:11.0f} W    "
          f"(€{effort_eur:.4f})")
    print(f"    Total cost:      €{c:.4f}")
    return c


def main():
    parser = argparse.ArgumentParser(description="Optimize ZFI controller gains")
    parser.add_argument("-m", "--model", type=Path, default=Path("data/plant_model.json"),
                        help="Plant model JSON from identify_plant.py")
    parser.add_argument("--validate-only", action="store_true",
                        help="Only validate simulation, don't optimize")
    parser.add_argument("--maxiter", type=int, default=50,
                        help="Max generations for differential evolution (default: 50)")
    parser.add_argument("--popsize", type=int, default=20,
                        help="Population size multiplier (default: 20)")
    parser.add_argument("--workers", type=int, default=0,
                        help="Parallel workers (0 = auto = CPU count, 1 = serial)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", type=Path, default=None,
                        help="Resume from checkpoint JSON (default: auto-detect data/optimizer_checkpoint.json)")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT,
                        help="Path to save checkpoints (default: data/optimizer_checkpoint.json)")
    parser.add_argument("--result", type=Path, default=DEFAULT_RESULT,
                        help="Path to save optimization result JSON (default: data/optimization_result.json)")
    args = parser.parse_args()

    if args.workers == 0:
        args.workers = min(multiprocessing.cpu_count(), 8)

    with open(args.model) as f:
        model_data = json.load(f)

    p = model_data["plant"]
    traces = model_data["traces"]
    plant_abc = (p["a"], p["b"], p["c"])
    delay_steps = p.get("delay_steps", 0)

    print("=" * 65)
    print("  Zero Feed-In Controller — Gain Optimizer")
    print("=" * 65)
    print(f"\n  Plant model (identified at dt={IDENT_DT_S}s):")
    print(f"    batt[k] = {p['a']:.4f} * batt[k-1] + {p['b']:.4f} * desired[k-{delay_steps}] + {p['c']:.2f}")
    pm = PlantModel(*plant_abc, delay_steps=delay_steps)
    print(f"    tau={pm.tau:.1f}s  K={pm.dc_gain:.3f}  c_cont={pm.c_cont:.2f}  R²={p['r_squared']:.4f}")
    print(f"    Transport delay: {delay_steps} steps ({delay_steps * IDENT_DT_S:.0f}s)")
    print(f"  Trace: {len(traces['disturbance_w'])} timesteps")
    if args.workers > 1:
        print(f"  Workers: {args.workers}")

    kp_seen = traces.get("kp_values_seen", [])
    ki_seen = traces.get("ki_values_seen", [])
    print(f"\n  Gains seen in log data:")
    print(f"    Kp: {kp_seen}")
    print(f"    Ki: {ki_seen}")

    sys_cfg = SystemConfig(
        max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
        min_soc_pct=15, max_soc_pct=85,
        discharge_target_w=30, charge_target_w=0,
        mode_hysteresis_w=50, charge_confirm_s=20,
        deadband_leak_ws=250, interval_s=5,
        delay_steps=delay_steps,
        ff_pv_entity="sensor.pv",
        ff_load_entities=("sensor.load",),
    )

    # Gains from the log period for validation
    logged_gains = np.array([
        0.25, 0.35, 0.17, 0.23,     # kp
        0.025, 0.036, 0.025, 0.036,  # ki (from log period)
        35.0,                         # deadband
        30.0, 30.0,                   # ff_tau, ff_deadband
        0.8, 0.6,                     # ff_gain_pv, ff_gain_load
        10000, 25, 90, 25,           # relay params (production defaults)
    ])

    # ── Validate (uses raw dict → _simulate_full for exact parity) ──
    print("\n" + "=" * 65)
    print("  Step 1: Validation — simulation vs real data (logged gains)")
    print("=" * 65)
    ok = validate(logged_gains, plant_abc, traces, sys_cfg)

    if args.validate_only:
        return

    if not ok:
        print("\nAborted: fix the simulation model before optimizing.")
        return

    # Pre-convert traces to numpy once for the fast simulation path.
    trace_arrays = TraceArrays(
        disturbance=np.array(traces["disturbance_w"], dtype=np.float64),
        soc=np.array(traces["soc_pct"], dtype=np.float64),
        dt=np.array(traces["dt_s"], dtype=np.float64),
        grid_actual=np.array(traces.get("grid_actual_w", []), dtype=np.float64),
        n=len(traces["disturbance_w"]),
    )

    # ── Baseline ──
    current_gains = np.array([
        0.061, 0.794, 0.233, 0.323,  # kp (optimized 2026-04-23)
        0.005, 0.300, 0.261, 0.020,  # ki (optimized 2026-04-23)
        5.1,                          # deadband
        30.6, 49.4,                   # ff_tau, ff_deadband
        1.41, 0.57,                   # ff_gain_pv, ff_gain_load
        10000, 25, 90, 25,           # relay: lockout_ws, cutoff_w, idle_s, min_active_w
    ])

    trace_days = float(trace_arrays.dt.sum()) / 86400.0
    print(f"  Trace duration: {trace_days:.2f} days")

    print("\n" + "=" * 65)
    print("  Step 2: Baseline — current production gains")
    print("=" * 65)
    baseline = simulate(current_gains, plant_abc, trace_arrays, sys_cfg)
    baseline_cost = print_metrics("Current gains", baseline, trace_days)

    print("\n  Current parameter values:")
    for i, name in enumerate(PARAM_NAMES):
        print(f"    {name:<22s}  {current_gains[i]:7.4f}")

    # ── Set globals for multiprocessing ──
    global _plant_abc, _traces, _sys_cfg
    _plant_abc = plant_abc
    _traces = trace_arrays
    _sys_cfg = sys_cfg

    # ── Optimize ──
    n_params = len(PARAM_NAMES)
    pop_total = args.popsize * n_params

    # Check for resume
    resume_data = None
    resume_path = args.resume
    if resume_path is None and args.checkpoint.exists():
        resume_path = args.checkpoint
    if resume_path is not None and resume_path.exists():
        resume_data = load_checkpoint(resume_path)
        # Discard checkpoint if param count changed (e.g. new relay params).
        if len(resume_data.get("best_x", [])) != n_params:
            print(f"\n  Checkpoint {resume_path} has "
                  f"{len(resume_data.get('best_x', []))} params, "
                  f"expected {n_params} — ignoring.")
            resume_data = None
        else:
            print(f"\n  Resuming from checkpoint: {resume_path}")
            print(f"    Generation: {resume_data['generation']}")
            print(f"    Best cost:  {resume_data['best_cost']:.1f}")

    print("\n" + "=" * 65)
    print(f"  Step 3: Optimization — {n_params} params, "
          f"pop={pop_total}, maxiter={args.maxiter}, workers={args.workers}")
    print("=" * 65)

    from scipy.optimize import differential_evolution

    tracker = ProgressTracker(
        baseline_cost, baseline.feed_in_energy_wh,
        current_gains, checkpoint_path=args.checkpoint,
    )

    # Build initial population.
    rng = np.random.RandomState(args.seed)
    init_pop = np.empty((pop_total, n_params))

    if resume_data is not None:
        # Warm restart: seed population around previous best.
        best_prev = np.array(resume_data["best_x"])
        init_pop[0] = best_prev
        tracker.gen = resume_data["generation"]
        tracker.best_cost = resume_data["best_cost"]
        tracker.best_x = best_prev.copy()
        for i in range(1, pop_total):
            for j in range(n_params):
                lo, hi = BOUNDS[j]
                spread = 0.15 * best_prev[j]  # tighter spread for resume
                init_pop[i, j] = np.clip(rng.normal(best_prev[j], spread), lo, hi)
        print(f"  Warm restart: population seeded around previous best "
              f"(gen {resume_data['generation']})")
    else:
        # Fresh start: seed most around current gains, rest random.
        init_pop[0] = current_gains
        for i in range(1, pop_total):
            for j in range(n_params):
                lo, hi = BOUNDS[j]
                if i < int(0.7 * pop_total):
                    center = current_gains[j]
                    spread = 0.3 * center
                    init_pop[i, j] = np.clip(rng.normal(center, spread), lo, hi)
                else:
                    init_pop[i, j] = rng.uniform(lo, hi)

    print(f"\n  {'gen':>5s}  {'cost':>12s} {'change':>9s}  "
          f"{'feed-in':>10s} {'change':>9s}  {'best':>12s}  {'time':>7s}")
    print("  " + "-" * 72)

    result = differential_evolution(
        cost,
        bounds=BOUNDS,
        init=init_pop,
        maxiter=args.maxiter,
        seed=args.seed,
        tol=1e-4,
        callback=tracker.callback,
        workers=args.workers,
        updating="deferred" if args.workers > 1 else "immediate",
        disp=False,
        polish=False,
    )

    elapsed = time.time() - tracker.t0
    print(f"\n  Finished in {elapsed:.0f}s  converged={result.success}")
    if not result.success:
        print(f"  Message: {result.message}")

    optimized = result.x
    opt_result = simulate(optimized, plant_abc, trace_arrays, sys_cfg)

    # Save final checkpoint.
    save_checkpoint(
        args.checkpoint,
        best_x=np.array(result.x),
        best_cost=float(result.fun),
        gen=tracker.gen,
        baseline_cost=baseline_cost,
        baseline_feedin=baseline.feed_in_energy_wh,
        current_gains=current_gains,
    )
    print(f"  Checkpoint saved to {args.checkpoint}")

    # ── Results ──
    print("\n" + "=" * 65)
    print("  Step 4: Results")
    print("=" * 65)
    print_comparison(current_gains, optimized)

    print()
    opt_cost = print_metrics("Optimized", opt_result, trace_days)

    sw_cur_d = baseline.relay_switches / trace_days
    sw_opt_d = opt_result.relay_switches / trace_days
    print(f"\n  Improvement:")
    for name, cur, opt in [
        ("Feed-in (Wh)", baseline.feed_in_energy_wh, opt_result.feed_in_energy_wh),
        ("Max feed-in (W)", baseline.max_feed_in_w, opt_result.max_feed_in_w),
        ("Relay sw/day", sw_cur_d, sw_opt_d),
        ("IAE (W*s)", baseline.iae, opt_result.iae),
        ("Cost (€)", baseline_cost, opt_cost),
    ]:
        delta = opt - cur
        pct = delta / cur * 100 if cur != 0 else 0
        arrow = "^" if delta > 0 else "v" if delta < 0 else "="
        print(f"    {name:<22s}  {cur:10.1f} -> {opt:10.1f}  {arrow} {abs(pct):.1f}%")

    # ── Sensitivity ──
    print("\n" + "=" * 65)
    print("  Step 5: Sensitivity analysis")
    print("=" * 65)
    sensitivity_analysis(optimized, plant_abc, trace_arrays, sys_cfg, opt_cost)

    # ── apps.yaml ──
    print("\n" + "=" * 65)
    print("  Recommended apps.yaml snippet")
    print("=" * 65)
    print("  # Controller:")
    print(f"  kp_discharge_up: {optimized[0]:.3f}")
    print(f"  kp_discharge_down: {optimized[1]:.3f}")
    print(f"  kp_charge_up: {optimized[2]:.3f}")
    print(f"  kp_charge_down: {optimized[3]:.3f}")
    print(f"  ki_discharge_up: {optimized[4]:.4f}")
    print(f"  ki_discharge_down: {optimized[5]:.4f}")
    print(f"  ki_charge_up: {optimized[6]:.4f}")
    print(f"  ki_charge_down: {optimized[7]:.4f}")
    print(f"  deadband: {optimized[8]:.1f}")
    print(f"  ff_filter_tau_s: {optimized[9]:.1f}")
    print(f"  ff_deadband: {optimized[10]:.1f}")
    print(f"  # ff_gain_pv: {optimized[11]:.2f}")
    print(f"  # ff_gain_load: {optimized[12]:.2f}")
    print("  # Driver:")
    print(f"  relay_lockout_ws: {optimized[13]:.0f}")
    print(f"  relay_lockout_cutoff_w: {optimized[14]:.0f}")
    print(f"  relay_lockout_idle_s: {optimized[15]:.0f}")
    print(f"  min_active_power_w: {optimized[16]:.0f}")

    # ── Save result JSON ──
    save_result(
        args.result,
        optimized_params=optimized,
        current_params=current_gains,
        baseline=baseline,
        optimized=opt_result,
        baseline_cost=baseline_cost,
        optimized_cost=opt_cost,
        plant_abc=plant_abc,
        sys_cfg=sys_cfg,
        generations=tracker.gen,
        converged=result.success,
        elapsed_s=elapsed,
    )
    print(f"\n  Result saved to {args.result}")


if __name__ == "__main__":
    main()
