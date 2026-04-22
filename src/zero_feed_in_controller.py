"""
Zero Feed-In Controller – Device-Agnostic PI Controller

Bidirectional PI controller that keeps the grid meter at ~0 W.
Publishes a signed desired-power value to a HA sensor entity.
A separate device driver reads that sensor and translates it
into hardware-specific commands.

Output convention (sensor.zfi_desired_power):
    positive = discharge (feed into house)
    negative = charge (absorb surplus into battery)
"""

from __future__ import annotations

import json
import os
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import TYPE_CHECKING

try:
    from .csv_logger import CsvLogger
except ImportError:
    from csv_logger import CsvLogger  # type: ignore[no-redef]

if TYPE_CHECKING:
    import appdaemon.plugins.hass.hassapi as hass


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi"
"""Prefix for HA entities published by this controller."""

EMERGENCY_SAFETY_MARGIN_W = 50
"""Extra margin subtracted from the forced output during an emergency clamp (W)."""

CONTROLLER_CSV_COLUMNS = [
    "grid_w", "soc_pct", "battery_power_w",
    "surplus_w", "mode", "desired_power_w",
    "p_term", "i_term", "ff_term",
    "integral", "target_w", "error_w",
    "in_deadband", "relay_locked", "charge_pending",
    "kp_used", "ki_used",
    "reason",
]
"""Column names for controller CSV log (excluding timestamp)."""


# ═══════════════════════════════════════════════════════════
#  Data types
# ═══════════════════════════════════════════════════════════


class OperatingMode(Enum):
    """Which direction the controller is currently regulating.

    DISCHARGING — house is consuming more than PV produces; battery
        feeds the deficit.  Target is ``discharge_target_w`` (default 30 W).
    CHARGING — PV surplus exceeds house load; excess is absorbed into
        the battery.  Target is ``charge_target_w`` (default 0 W).

    Transitions use a Schmitt trigger with hysteresis to avoid relay
    chatter.  See ``ControlLogic._update_operating_mode``.
    """

    CHARGING = auto()
    DISCHARGING = auto()


@dataclass
class PIGains:
    """Kp/Ki pair for one physical quadrant."""

    kp: float
    """Proportional gain."""
    ki: float
    """Integral gain."""


@dataclass
class PIGainSet:
    """Four gain sets indexed by mode × error-sign quadrant.

    Selection logic::

                      error >= 0           error < 0
        DISCHARGING   discharge_up         discharge_down
        CHARGING      charge_down          charge_up

    Note CHARGING is flipped: error >= 0 means "charging too much,
    reduce" → charge_down.  Error < 0 means "surplus available,
    charge more" → charge_up.
    """

    discharge_up: PIGains
    """Increase discharge (error >= 0 in DISCHARGING)."""
    discharge_down: PIGains
    """Decrease discharge (error < 0 in DISCHARGING)."""
    charge_up: PIGains
    """Increase charge (error < 0 in CHARGING)."""
    charge_down: PIGains
    """Decrease charge (error >= 0 in CHARGING)."""

    def select(self, mode: OperatingMode, error: float) -> PIGains:
        """Return the gain pair for the current mode and error sign."""
        if mode == OperatingMode.DISCHARGING:
            return self.discharge_up if error >= 0 else self.discharge_down
        return self.charge_up if error < 0 else self.charge_down


@dataclass
class FeedForwardSource:
    """A single feed-forward input source.

    Each source tracks a HA sensor entity and applies a gain-scaled
    delta to the control output.  The ``sign`` field determines the
    effect direction:

    * ``-1.0`` for generation (PV): drop → increase discharge.
    * ``+1.0`` for loads (wallbox, dryer): increase → increase discharge.
    """

    entity: str
    """HA sensor entity ID."""
    gain: float
    """Fraction of delta to apply (0.0–1.0)."""
    sign: float
    """+1.0 for loads, -1.0 for generation."""
    filtered_w: float | None = None
    """EMA filter state. None until first reading; bootstrapped on first cycle."""
    name: str | None = None
    """Short label used for HA sensor names (e.g. 'pv'). Defaults to 'src{index}'."""


@dataclass
class FeedForwardSourceDebug:
    """Per-source feed-forward snapshot after one control cycle."""

    name: str
    """Source label (matches FeedForwardSource.name)."""
    raw_w: float | None
    """Current sensor reading (W)."""
    ema_w: float | None
    """EMA filter state before this cycle's update (W)."""
    delta_w: float
    """EMA derivative: α × (raw − EMA). Zero when EMA not seeded."""
    contrib_w: float
    """Per-source contribution: sign × gain × delta_w (pre-deadband, W)."""


@dataclass
class Config:
    """Typed, validated configuration loaded from ``apps.yaml``.

    Covers three domains:

    * **Sensor entity IDs** — grid power, SOC, battery power.
    * **Controller tuning** — PI gains, deadband, targets, limits.
    * **Protection** — max feed-in, emergency multiplier.

    Constructed via the ``from_args`` classmethod which maps the
    free-form ``args`` dict from AppDaemon into typed fields with
    sensible defaults.
    """

    # Sensors
    grid_sensor: str
    """HA entity ID for grid power sensor (W). Positive = importing."""
    soc_sensor: str
    """HA entity ID for battery state-of-charge sensor (%)."""
    battery_power_sensor: str
    """HA entity ID for battery power sensor. Sign: +discharge / -charge."""

    # Battery sensor mode: "signed" or "unsigned"
    #   signed   – sensor already has correct sign (+discharge/-charge)
    #   unsigned – sensor is always positive; sign derived from ac_mode_entity
    battery_sensor_mode: str = "signed"
    ac_mode_entity: str | None = None
    """Required when battery_sensor_mode='unsigned'. AC mode select entity
    whose state ('Input mode'/'Output mode') determines charge/discharge sign."""

    # Optional switches (input_boolean) to enable/disable directions
    charge_switch: str | None = None
    discharge_switch: str | None = None

    # Relay lockout feedback from driver
    relay_locked_sensor: str | None = None
    """HA entity published by the driver when the relay SM is clamping output.
    When 'true', the controller freezes the integral to prevent windup."""

    # Dynamic min SOC from forecast automation
    dynamic_min_soc_entity: str | None = None
    """``input_number`` entity that carries the forecast-adjusted min SOC (%).

    Read every cycle.  When unavailable, falls back to the static
    ``min_soc_pct`` from apps.yaml.  The dynamic value is clamped to
    ``[min_soc_pct, max_soc_pct]`` so it can never violate hard limits.
    """

    # Controller tuning
    discharge_target_w: float = 30.0
    charge_target_w: float = 0.0
    kp_discharge_up: float = 0.50
    """Kp for increasing discharge (DISCHARGING, error >= 0)."""
    kp_discharge_down: float = 0.71
    """Kp for decreasing discharge (DISCHARGING, error < 0)."""
    kp_charge_up: float = 0.33
    """Kp for increasing charge (CHARGING, error < 0)."""
    kp_charge_down: float = 0.45
    """Kp for decreasing charge (CHARGING, error >= 0)."""
    ki_discharge_up: float = 0.025
    """Ki for increasing discharge."""
    ki_discharge_down: float = 0.051
    """Ki for decreasing discharge."""
    ki_charge_up: float = 0.011
    """Ki for increasing charge."""
    ki_charge_down: float = 0.021
    """Ki for decreasing charge."""
    deadband_w: float = 25.0
    deadband_leak_ws: float = 500.0
    """Error×time threshold (W·s) for in-deadband leak correction.

    Accumulates ``error × dt`` while the PI is frozen inside the deadband.
    When the magnitude reaches this threshold the PI is allowed to run
    once (bypassing the deadband) and the accumulator resets.  Set to 0
    to disable.  Default 500 W·s ≈ 26 s at a persistent 19 W error.
    """
    interval_s: int = 5

    # Feed-forward
    ff_enabled: bool = True
    """Master switch to enable/disable feed-forward compensation."""
    ff_deadband_w: float = 20.0
    """Ignore total FF correction below this threshold (W)."""
    ff_filter_tau_s: float = 30.0
    """EMA filter time constant for the FF derivative (s). Higher = smoother but slower response."""
    ff_sources: tuple[FeedForwardSource, ...] = ()
    """Feed-forward input sources (PV, loads, etc.). Empty = FF disabled."""

    # Power limits
    max_discharge_w: float = 800.0
    max_charge_w: float = 2400.0
    min_soc_pct: float = 10.0
    max_soc_pct: float = 100.0

    # Mode switching (Schmitt trigger)
    mode_hysteresis_w: float = 50.0
    charge_confirm_s: float = 15.0

    # Protection
    max_feed_in_w: float = 800.0
    emergency_kp_mult: float = 4.0

    # Debug
    dry_run: bool = True
    debug: bool = False
    """If True, publish internal PI sensors (p_term, i_term, integral, etc.)."""
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX

    # File logging
    log_dir: str = ""
    """Directory for CSV log files. Empty = file logging disabled."""

    @classmethod
    def from_args(cls, args: dict[str, object]) -> Config:
        """Create a ``Config`` from the raw ``args`` dict provided by AppDaemon.

        Maps apps.yaml keys (e.g. ``target_grid_power``) to dataclass
        fields (``discharge_target_w``).  Falls back to class-level
        defaults for any omitted key.

        Backward compatibility: if old-style ``kp``/``ki`` keys are
        present (without quadrant suffixes), they are mapped to all
        four quadrants.
        """
        gains = cls._parse_gains(args)
        return cls(
            grid_sensor=args["grid_power_sensor"],
            soc_sensor=args["soc_sensor"],
            battery_power_sensor=args["battery_power_sensor"],
            battery_sensor_mode=args.get(
                "battery_sensor_mode", cls.battery_sensor_mode
            ),
            ac_mode_entity=args.get("ac_mode_entity"),
            charge_switch=args.get("charge_switch"),
            discharge_switch=args.get("discharge_switch"),
            relay_locked_sensor=args.get("relay_locked_sensor"),
            dynamic_min_soc_entity=args.get("dynamic_min_soc_entity"),
            discharge_target_w=float(
                args.get("target_grid_power", cls.discharge_target_w)
            ),
            charge_target_w=float(
                args.get("charge_target_power", cls.charge_target_w)
            ),
            **gains,
            deadband_w=float(args.get("deadband", cls.deadband_w)),
            deadband_leak_ws=float(args.get("deadband_leak_ws", cls.deadband_leak_ws)),
            interval_s=int(args.get("interval", cls.interval_s)),
            max_discharge_w=float(args.get("max_output", cls.max_discharge_w)),
            max_charge_w=float(args.get("max_charge", cls.max_charge_w)),
            min_soc_pct=float(args.get("min_soc", cls.min_soc_pct)),
            max_soc_pct=float(args.get("max_soc", cls.max_soc_pct)),
            mode_hysteresis_w=float(
                args.get("mode_hysteresis", cls.mode_hysteresis_w)
            ),
            charge_confirm_s=float(
                args.get("charge_confirm", cls.charge_confirm_s)
            ),
            max_feed_in_w=float(args.get("max_feed_in", cls.max_feed_in_w)),
            emergency_kp_mult=float(
                args.get("emergency_kp_multiplier", cls.emergency_kp_mult)
            ),
            ff_enabled=bool(args.get("ff_enabled", cls.ff_enabled)),
            ff_deadband_w=float(args.get("ff_deadband", cls.ff_deadband_w)),
            ff_filter_tau_s=float(args.get("ff_filter_tau_s", cls.ff_filter_tau_s)),
            ff_sources=cls._parse_ff_sources(args),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            log_dir=args.get("log_dir", cls.log_dir),
        )

    @classmethod
    def _parse_gains(cls, args: dict[str, object]) -> dict[str, float]:
        """Extract four-quadrant PI gains from args, with legacy fallback.

        New-style keys (``kp_discharge_up``, etc.) take priority.
        If absent, falls back to legacy ``kp``/``ki`` applied uniformly.
        """
        if "kp_discharge_up" in args:
            return {
                "kp_discharge_up": float(args.get("kp_discharge_up", cls.kp_discharge_up)),
                "kp_discharge_down": float(args.get("kp_discharge_down", cls.kp_discharge_down)),
                "kp_charge_up": float(args.get("kp_charge_up", cls.kp_charge_up)),
                "kp_charge_down": float(args.get("kp_charge_down", cls.kp_charge_down)),
                "ki_discharge_up": float(args.get("ki_discharge_up", cls.ki_discharge_up)),
                "ki_discharge_down": float(args.get("ki_discharge_down", cls.ki_discharge_down)),
                "ki_charge_up": float(args.get("ki_charge_up", cls.ki_charge_up)),
                "ki_charge_down": float(args.get("ki_charge_down", cls.ki_charge_down)),
            }
        if "kp" not in args and "ki" not in args:
            return {}
        # Legacy: single kp/ki applied to all quadrants
        kp = float(args.get("kp", cls.kp_discharge_up))
        ki = float(args.get("ki", cls.ki_discharge_up))
        return {
            "kp_discharge_up": kp,
            "kp_discharge_down": kp,
            "kp_charge_up": kp,
            "kp_charge_down": kp,
            "ki_discharge_up": ki,
            "ki_discharge_down": ki,
            "ki_charge_up": ki,
            "ki_charge_down": ki,
        }

    @classmethod
    def _parse_ff_sources(
        cls, args: dict[str, object],
    ) -> tuple[FeedForwardSource, ...]:
        """Parse feed-forward sources from args.

        Supports the new ``feed_forward_sources`` list format and the
        legacy ``pv_sensor`` / ``ff_pv_gain`` / ``ff_pv_deadband``
        single-source shorthand.
        """
        raw = args.get("feed_forward_sources")
        if raw is not None:
            return tuple(
                FeedForwardSource(
                    entity=src["entity"],
                    gain=float(src.get("gain", 0.6)),
                    sign=float(src.get("sign", -1.0)),
                    name=src.get("name"),
                )
                for src in raw
            )
        # Backward compat: old pv_sensor key
        pv_sensor = args.get("pv_sensor", "")
        if pv_sensor:
            return (
                FeedForwardSource(
                    entity=pv_sensor,
                    gain=float(args.get("ff_pv_gain", 0.6)),
                    sign=-1.0,
                ),
            )
        return ()


@dataclass
class Measurement:
    """Snapshot of all external inputs needed for one control cycle.

    Assembled by the HA adapter from sensor readings and switch states.
    Passed into ``ControlLogic.compute()`` as a single immutable bundle
    so the control logic never touches HA directly.

    Power sign convention (project-wide):
        positive = discharge (feed into house)
        negative = charge   (absorb surplus into battery)
    """

    grid_power_w: float
    """Grid power in watts.  Positive = importing from grid."""
    soc_pct: float
    """Battery state of charge in percent (0–100)."""
    battery_power_w: float
    """Actual battery power (W).  +discharge / -charge."""
    ff_readings: dict[str, float | None] = field(default_factory=dict)
    """Sensor readings for feed-forward sources, keyed by entity ID."""
    discharge_enabled: bool = True
    """Whether the user's discharge switch (``input_boolean``) is on."""
    charge_enabled: bool = True
    """Whether the user's charge switch (``input_boolean``) is on."""
    relay_locked: bool = False
    """Whether the driver's relay SM is clamping output (lockout active)."""
    dynamic_min_soc_pct: float | None = None
    """Dynamic minimum SOC (%) from ``input_number.zfi_min_soc``.

    When set, overrides ``Config.min_soc_pct`` for the SOC-too-low guard.
    Clamped to ``[Config.min_soc_pct, Config.max_soc_pct]`` so the
    dynamic value can never violate the hard safety limits from apps.yaml.
    ``None`` means no dynamic override — use ``Config.min_soc_pct``.
    """


@dataclass
class ControlOutput:
    """Result of a single control cycle.

    Carries the desired power setpoint together with the PI terms
    and a human-readable reason string for logging / HA sensor
    publishing.
    """

    desired_power_w: float
    """Signed desired power (W).  +discharge / -charge."""
    p_term: float
    """Proportional term of the PI output (W)."""
    i_term: float
    """Integral term of the PI output (W)."""
    ff_term: float
    """Feed-forward term (W). Sum of all FF sources."""
    reason: str
    """Human-readable decision reason (e.g. 'EMERGENCY', 'SOC too low')."""

    @staticmethod
    def from_raw(
        raw: float,
        p_term: float,
        i_term: float,
        ff_term: float,
        reason: str,
    ) -> ControlOutput:
        """Construct from a raw (possibly clamped) power value."""
        return ControlOutput(
            desired_power_w=raw,
            p_term=p_term,
            i_term=i_term,
            ff_term=ff_term,
            reason=reason,
        )

    @staticmethod
    def idle(
        p_term: float, i_term: float, ff_term: float, reason: str,
    ) -> ControlOutput:
        """Shorthand for a zero-power (idle) output with a guard reason."""
        return ControlOutput.from_raw(0.0, p_term, i_term, ff_term, reason)


@dataclass
class ControllerState:
    """Mutable state carried across control cycles by ``ControlLogic``.

    Reset semantics:
        * ``integral`` is reset to 0 on mode transitions.
        * ``last_computed_w`` is set by each cycle's output (not
          explicitly reset on mode change).
        * ``charge_pending_since`` is set when a charge-mode candidate
          is first detected and cleared on confirmation or cancellation.
    """

    integral: float = 0.0
    """PI integral accumulator (W).  Committed only on normal output."""
    last_computed_w: float = 0.0
    """Most recent output power (W).  Used as hold value in deadband."""
    mode: OperatingMode = OperatingMode.DISCHARGING
    """Current operating mode (Schmitt trigger output)."""
    charge_pending_since: float | None = None
    """Monotonic timestamp when charge-mode candidacy started, or None."""


# ═══════════════════════════════════════════════════════════
#  Feed-Forward (pure computation)
# ═══════════════════════════════════════════════════════════


class FeedForward:
    """Multi-source feed-forward compensation using a filtered derivative.

    Each source tracks an EMA of its sensor value. The FF term for each
    source is ``sign × gain × α × (current − EMA_state)``, i.e. the EMA
    derivative scaled by gain. This is the standard derivative-with-filter
    form (ISA PID D-filter) and attenuates high-frequency noise by
    1/(1 + jωτ) while preserving slow ramps.

    ``τ = filter_tau_s``, ``α = interval_s / (τ + interval_s)``.

    A deadband is applied to the sum of all sources after filtering.
    First-cycle bootstrap: the EMA is seeded from the first reading to
    produce zero delta on the second cycle.
    """

    def __init__(
        self,
        sources: tuple[FeedForwardSource, ...],
        deadband_w: float = 20.0,
        enabled: bool = True,
        filter_tau_s: float = 30.0,
        interval_s: float = 5.0,
    ) -> None:
        self._sources: list[FeedForwardSource] = [
            FeedForwardSource(
                entity=s.entity,
                gain=s.gain,
                sign=s.sign,
                name=s.name if s.name is not None else f"src{i}",
            )
            for i, s in enumerate(sources)
        ]
        self._deadband_w = deadband_w
        self._enabled = enabled
        self._alpha = interval_s / (filter_tau_s + interval_s)
        self.last_debug: list[FeedForwardSourceDebug] = []

    @property
    def entities(self) -> list[str]:
        """Return entity IDs that the HA adapter must read each cycle."""
        return [s.entity for s in self._sources]

    def compute(self, readings: dict[str, float | None]) -> float:
        """Compute total feed-forward correction.

        For each source, the filtered derivative (α × (current − EMA_state))
        is scaled by ``sign × gain`` and summed. A deadband is applied to the
        total. Returns 0.0 when disabled or on first cycle (EMA not seeded).
        """
        if not self._enabled:
            self.last_debug = []
            return 0.0
        debug: list[FeedForwardSourceDebug] = []
        total = 0.0
        for src in self._sources:
            current = readings.get(src.entity)
            if current is None or src.filtered_w is None:
                debug.append(FeedForwardSourceDebug(
                    name=src.name,
                    raw_w=current,
                    ema_w=src.filtered_w,
                    delta_w=0.0,
                    contrib_w=0.0,
                ))
                continue
            delta = self._alpha * (current - src.filtered_w)
            contrib = src.sign * src.gain * delta
            debug.append(FeedForwardSourceDebug(
                name=src.name,
                raw_w=current,
                ema_w=src.filtered_w,
                delta_w=delta,
                contrib_w=contrib,
            ))
            total += contrib
        self.last_debug = debug
        if abs(total) < self._deadband_w:
            return 0.0
        return total

    def update_previous(self, readings: dict[str, float | None]) -> None:
        """Advance the EMA filter state for all sources."""
        for src in self._sources:
            current = readings.get(src.entity)
            if current is not None:
                if src.filtered_w is None:
                    src.filtered_w = current  # bootstrap: zero delta next cycle
                else:
                    src.filtered_w = (
                        self._alpha * current + (1.0 - self._alpha) * src.filtered_w
                    )


# ═══════════════════════════════════════════════════════════
#  PI Controller (pure computation)
# ═══════════════════════════════════════════════════════════


class PIController:
    """PI controller with anti-windup back-calculation.

    Anti-windup uses *back-calculation*: when the output saturates at
    ``output_min`` or ``output_max``, the integral term is
    back-calculated so that ``p_term + integral == saturated_output``.
    This prevents integral build-up during sustained saturation.

    Gains are supplied per call via ``PIGains`` so the caller can
    switch gains every cycle (four-quadrant selection).
    """

    def __init__(
        self,
        output_min: float,
        output_max: float,
    ) -> None:
        self.output_min: float = output_min
        self.output_max: float = output_max

    def update(
        self,
        error: float,
        integral: float,
        dt: float,
        gains: PIGains,
    ) -> tuple[float, float, float]:
        """Compute one PI step.

        Args:
            error: Regulation error (W).  Positive = grid importing.
            integral: Current integral accumulator value (W).
            dt: Time since last update (s).
            gains: Kp/Ki pair for this cycle.

        Returns:
            ``(output, p_term, new_integral)`` — the clamped output
            power, the proportional contribution, and the
            (possibly back-calculated) new integral value.
        """
        p_term = gains.kp * error
        new_integral = integral + gains.ki * error * dt

        output = p_term + new_integral

        if output > self.output_max:
            new_integral = self.output_max - p_term
            output = self.output_max
        elif output < self.output_min:
            new_integral = self.output_min - p_term
            output = self.output_min

        return output, p_term, new_integral


# ═══════════════════════════════════════════════════════════
#  Control Logic (pure computation, no HA dependencies)
# ═══════════════════════════════════════════════════════════


class ControlLogic:
    """Device/HA-agnostic control logic.  Fully testable without mocks.

    Owns the PI controller instance, the ``ControllerState``, and all
    decision logic (emergency protection, mode switching, guards,
    surplus clamping).  Receives a ``Measurement`` each cycle and
    returns a ``ControlOutput``.

    The optional *log* callback is used for human-readable status
    messages.  If omitted, messages are silently discarded.
    """

    def __init__(self, cfg: Config, log: Callable[[str], None] | None = None) -> None:
        self.cfg = cfg
        self.state = ControllerState()
        self.gains = PIGainSet(
            discharge_up=PIGains(cfg.kp_discharge_up, cfg.ki_discharge_up),
            discharge_down=PIGains(cfg.kp_discharge_down, cfg.ki_discharge_down),
            charge_up=PIGains(cfg.kp_charge_up, cfg.ki_charge_up),
            charge_down=PIGains(cfg.kp_charge_down, cfg.ki_charge_down),
        )
        self.pi = PIController(
            output_min=-cfg.max_charge_w,
            output_max=cfg.max_discharge_w,
        )
        self.ff = FeedForward(
            cfg.ff_sources,
            deadband_w=cfg.ff_deadband_w,
            enabled=cfg.ff_enabled,
            filter_tau_s=cfg.ff_filter_tau_s,
            interval_s=cfg.interval_s,
        )
        self._last_p = 0.0
        self._last_i = 0.0
        self._last_ff = 0.0
        self._last_in_deadband: bool = False
        self._last_kp: float = 0.0
        self._last_ki: float = 0.0
        self._db_acc: float = 0.0
        self._log = log or (lambda msg: None)

    def seed(self, battery_power_w: float | None) -> None:
        """Seed PI from current battery power to avoid step on startup."""
        current_w = battery_power_w if battery_power_w is not None else 0.0
        self.state.last_computed_w = current_w
        self.state.integral = current_w
        if current_w < 0:
            self.state.mode = OperatingMode.CHARGING
        self._log(f"Seeded from battery_power: {current_w:.0f}W")

    def state_snapshot(self) -> dict[str, object]:
        """Return a dict of state fields suitable for JSON persistence."""
        return {
            "integral": self.state.integral,
            "last_computed_w": self.state.last_computed_w,
            "mode": self.state.mode.name,
            "db_acc": self._db_acc,
        }

    def restore_from_snapshot(self, data: dict[str, object]) -> bool:
        """Restore internal state from a snapshot dict.

        Returns True on success, False if the data is invalid or
        incomplete.  On failure the state is left unchanged.
        """
        try:
            integral = float(data["integral"])
            last_computed_w = float(data["last_computed_w"])
            mode = OperatingMode[data["mode"]]
            db_acc = float(data.get("db_acc", 0.0))
        except (KeyError, ValueError, TypeError):
            return False
        self.state.integral = integral
        self.state.last_computed_w = last_computed_w
        self.state.mode = mode
        self._db_acc = db_acc
        return True

    @staticmethod
    def estimate_surplus(m: Measurement) -> float:
        """Estimate PV minus house load.

        Energy balance at AC bus:
            PV + battery_power + grid_power = house_load
            surplus = PV - house = -battery_power - grid_power
        """
        return -m.battery_power_w - m.grid_power_w

    def compute(self, m: Measurement, now: float | None = None) -> ControlOutput:
        """Run one full control cycle and update internal state.

        Pipeline::

            emergency check → mode update → PI + FF → guards → clamp

        Args:
            m: Current sensor snapshot.
            now: Monotonic timestamp (seconds).  Defaults to
                ``time.monotonic()`` when omitted.

        Returns:
            A ``ControlOutput`` with the desired power, PID/FF terms,
            and the decision reason.
        """
        if now is None:
            now = time.monotonic()

        self._last_in_deadband = False
        self._last_kp = 0.0
        self._last_ki = 0.0

        surplus = self.estimate_surplus(m)

        emergency = self._check_emergency(m)
        if emergency is not None:
            self.state.last_computed_w = emergency.desired_power_w
            self.ff.update_previous(m.ff_readings)
            return emergency

        self._update_operating_mode(surplus, now)
        target = self.target_for_mode()

        error = m.grid_power_w - target
        pi_out, p_term, new_integral = self._run_pi(error, self.cfg.interval_s)
        self._last_p = p_term

        ff = self.ff.compute(m.ff_readings)
        self._last_ff = ff

        combined = pi_out + ff

        guarded = self._apply_guards(combined, m, surplus)
        if guarded is not None:
            # Guard forces idle — reset integral so PI starts fresh when
            # the guard clears.  The integral is only meaningful in a
            # closed loop; while the output is blocked there is no feedback.
            self.state.integral = 0.0
            self._last_i = 0.0
            self.state.last_computed_w = guarded.desired_power_w
            self.ff.update_previous(m.ff_readings)
            return guarded

        self.state.integral = new_integral
        self._last_i = new_integral

        clamped = self._clamp(combined, surplus)

        # Anti-windup: back-calculate when output was clamped.
        # During relay lock the PI freeze above already prevents normal windup.
        # But if the surplus clamp fires while relay is locked the integral can
        # be more aggressive than what the current surplus allows — cap it by
        # applying anti-windup only when it would bring the integral closer to
        # zero (less extreme), never when it would push it further away.
        if clamped != combined:
            target_integral = clamped - p_term
            if abs(target_integral) < abs(self.state.integral):
                self.state.integral = target_integral
                self._last_i = self.state.integral

        suffix = " (relay locked)" if m.relay_locked else ""
        reason = f"{'Charge' if clamped < 0 else 'Discharge'} ({self.state.mode.name}){suffix}"
        output = ControlOutput.from_raw(
            clamped, self._last_p, self._last_i,
            self._last_ff, reason,
        )
        self.state.last_computed_w = output.desired_power_w
        self.ff.update_previous(m.ff_readings)
        return output

    def _check_emergency(self, m: Measurement) -> ControlOutput | None:
        """Detect excessive grid feed-in and force a reduced setpoint.

        If instantaneous feed-in exceeds ``max_feed_in_w``, the output
        is slashed by the excess plus a safety margin.  The integral
        is back-calculated so that the PI resumes smoothly.

        Returns:
            An ``EMERGENCY`` ``ControlOutput`` if triggered, else ``None``.
        """
        feed_in = max(0.0, -m.grid_power_w)
        if feed_in <= self.cfg.max_feed_in_w:
            return None

        excess = feed_in - self.cfg.max_feed_in_w
        forced = max(0.0, self.state.last_computed_w - excess - EMERGENCY_SAFETY_MARGIN_W)
        # Back-calculate integral so PI resumes smoothly (p_term = 0)
        self.state.integral = forced
        return ControlOutput.from_raw(forced, 0.0, forced, 0.0, "EMERGENCY")

    def _run_pi(
        self, error: float, dt: float,
    ) -> tuple[float, float, float]:
        """Compute PI output without committing state.

        If the error is within the deadband, the PI is frozen and
        the last computed output is held.  A secondary accumulator
        tracks ``error × dt`` while frozen; when its magnitude reaches
        ``deadband_leak_ws`` the deadband is bypassed for one tick so
        the integral can correct a persistent steady-state offset.

        Returns:
            ``(output, p_term, new_integral)`` — caller decides
            whether to commit ``new_integral`` to ``self.state``.
        """
        if abs(error) <= self.cfg.deadband_w:
            self._last_in_deadband = True
            leak = self.cfg.deadband_leak_ws
            if leak > 0:
                self._db_acc += error * dt
                if abs(self._db_acc) < leak:
                    return self.state.last_computed_w, 0.0, self.state.integral
                self._db_acc = 0.0
                self._last_in_deadband = False  # leak fired — allow PI this tick
            else:
                return self.state.last_computed_w, 0.0, self.state.integral
        else:
            self._db_acc = 0.0

        gains = self.gains.select(self.state.mode, error)
        self._last_kp = gains.kp
        self._last_ki = gains.ki
        pi_output, p_term, new_integral = self.pi.update(
            error=error,
            integral=self.state.integral,
            dt=self.cfg.interval_s,
            gains=gains,
        )
        return pi_output, p_term, new_integral

    def _update_operating_mode(self, surplus: float, now: float) -> None:
        """Schmitt trigger with charge-confirmation delay.

        DISCHARGING → CHARGING:
            Requires surplus to stay above ``+hysteresis`` for at least
            ``charge_confirm_s`` consecutive seconds.
        CHARGING → DISCHARGING:
            Instant when surplus drops below ``-hysteresis``.

        On every mode transition the integral accumulator is reset to
        zero so the PI starts fresh in the new regime.
        """
        h = self.cfg.mode_hysteresis_w
        old_mode = self.state.mode

        if self.state.mode == OperatingMode.DISCHARGING:
            if surplus > h:
                if self.state.charge_pending_since is None:
                    self.state.charge_pending_since = now
                    self._log(
                        f"Charge pending: surplus={surplus:.0f}W > "
                        f"hysteresis={h:.0f}W, confirming for "
                        f"{self.cfg.charge_confirm_s:.0f}s"
                    )
                elif now - self.state.charge_pending_since >= self.cfg.charge_confirm_s:
                    self.state.mode = OperatingMode.CHARGING
                    self.state.charge_pending_since = None
            else:
                if self.state.charge_pending_since is not None:
                    self._log("Charge pending cancelled: surplus dropped")
                    self.state.charge_pending_since = None
        else:
            if surplus < -h:
                self.state.mode = OperatingMode.DISCHARGING

        if self.state.mode != old_mode:
            self.state.charge_pending_since = None
            self._db_acc = 0.0
            old_integral = self.state.integral
            if self.state.mode == OperatingMode.CHARGING:
                # Seed the integral to the expected steady-state value so we
                # don't waste minutes winding up from zero.  The surplus has
                # been stable for charge_confirm_s seconds, so -surplus is a
                # reliable estimate of the charge rate we'll need.
                self.state.integral = max(-surplus, -self.cfg.max_charge_w)
            else:
                self.state.integral = 0.0
            self._log(
                f"Mode {old_mode.name} → {self.state.mode.name} "
                f"(integral: {old_integral:.0f} → {self.state.integral:.0f})"
            )

    def target_for_mode(self) -> float:
        """Return the grid-power target (W) for the current operating mode.

        DISCHARGING: ``discharge_target_w`` (default 30 W — small grid draw
        as safety buffer).
        CHARGING: ``charge_target_w`` (default 0 W — absorb all surplus).
        """
        if self.state.mode == OperatingMode.CHARGING:
            return self.cfg.charge_target_w
        return self.cfg.discharge_target_w

    def _apply_guards(
        self, raw_limit: float, m: Measurement, surplus: float
    ) -> ControlOutput | None:
        """Check safety conditions that override PI output.

        Guards are evaluated *before* the integral is committed, so the
        PI state freezes while any guard blocks the output.

        Returns:
            An idle ``ControlOutput`` if a guard fires, else ``None``.
        """
        if raw_limit > 0 and not m.discharge_enabled:
            return ControlOutput.idle(
                self._last_p, self._last_i,
                self._last_ff, "Discharge disabled",
            )
        if raw_limit < 0 and not m.charge_enabled:
            return ControlOutput.idle(
                self._last_p, self._last_i,
                self._last_ff, "Charge disabled",
            )

        effective_min_soc = self.cfg.min_soc_pct
        if m.dynamic_min_soc_pct is not None:
            effective_min_soc = max(
                self.cfg.min_soc_pct,
                min(m.dynamic_min_soc_pct, self.cfg.max_soc_pct),
            )
        if raw_limit > 0 and m.soc_pct <= effective_min_soc:
            return ControlOutput.idle(
                self._last_p, self._last_i,
                self._last_ff, "SOC too low",
            )

        if raw_limit < 0:
            if surplus <= 0:
                return ControlOutput.idle(
                    self._last_p, self._last_i,
                    self._last_ff, "No surplus, charge blocked",
                )
            if m.soc_pct >= self.cfg.max_soc_pct:
                return ControlOutput.idle(
                    self._last_p, self._last_i,
                    self._last_ff, "SOC full",
                )

        return None

    def _clamp(self, raw: float, surplus: float) -> float:
        """Enforce hard power limits and surplus cap.

        * Discharge is capped at ``max_discharge_w``.
        * Charge is capped at both ``max_charge_w`` and the available
          solar surplus (whichever is smaller), preventing the battery
          from pulling power from the grid.

        Returns:
            Clamped power value (W).
        """
        if raw > 0:
            return min(raw, self.cfg.max_discharge_w)
        elif raw < 0:
            max_safe_charge = max(0.0, surplus)
            return max(raw, -self.cfg.max_charge_w, -max_safe_charge)
        return 0.0


# ═══════════════════════════════════════════════════════════
#  Main AppDaemon App (thin HA adapter)
# ═══════════════════════════════════════════════════════════

try:
    import appdaemon.plugins.hass.hassapi as hass

    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


class ZeroFeedInController(_HASS_BASE):
    """Thin Home Assistant / AppDaemon adapter.

    Responsibilities:

    * Read HA sensor entities and assemble a ``Measurement``.
    * Delegate computation to ``ControlLogic``.
    * Publish results back to HA as ``sensor.zfi_*`` entities.

    All domain logic lives in ``ControlLogic`` — this class only
    bridges the gap between AppDaemon callbacks and pure Python.
    """

    cfg: Config
    logic: ControlLogic

    def initialize(self) -> None:
        self.cfg = Config.from_args(self.args)
        self.logic = ControlLogic(self.cfg, log=self.log)

        # CSV file logger (disabled when log_dir is empty)
        self._csv: CsvLogger | None = None
        if self.cfg.log_dir:
            self._csv = CsvLogger(
                self.cfg.log_dir,
                "zfi_controller",
                CONTROLLER_CSV_COLUMNS,
            )
            self.log(f"CSV logging to {self.cfg.log_dir}/zfi_controller_*.csv")

        bp = self._read_float(self.cfg.battery_power_sensor)
        self.logic.seed(bp)

        # State persistence: restore from file, then seed overrides if needed
        _run_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "run")
        os.makedirs(_run_dir, exist_ok=True)
        self._state_file: str = self.args.get(
            "state_file",
            os.path.join(_run_dir, "zfi_controller_state.json"),
        )
        self._state_save_interval: int = 12  # save every 12 ticks (~60s at 5s interval)
        self._tick_count: int = 0
        if self._restore_state():
            self.log(
                f"Restored state from {self._state_file}: "
                f"mode={self.logic.state.mode.name} "
                f"integral={self.logic.state.integral:.1f}W "
                f"last_computed={self.logic.state.last_computed_w:.1f}W"
            )
        else:
            self.log("No saved state found, using battery_power seed")

        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | mode={self.logic.state.mode.name} "
            f"battery_sensor_mode={self.cfg.battery_sensor_mode} "
            f"discharge_target={self.cfg.discharge_target_w}W "
            f"charge_target={self.cfg.charge_target_w}W "
            f"hysteresis={self.cfg.mode_hysteresis_w}W "
            f"Gains | dis_up=({self.cfg.kp_discharge_up:.2f},{self.cfg.ki_discharge_up:.3f}) "
            f"dis_dn=({self.cfg.kp_discharge_down:.2f},{self.cfg.ki_discharge_down:.3f}) "
            f"chg_up=({self.cfg.kp_charge_up:.2f},{self.cfg.ki_charge_up:.3f}) "
            f"chg_dn=({self.cfg.kp_charge_down:.2f},{self.cfg.ki_charge_down:.3f}) "
            f"ff_sources={len(self.cfg.ff_sources)} "
            f"ff_enabled={self.cfg.ff_enabled} "
            f"dry_run={self.cfg.dry_run}"
        )

        if self.cfg.battery_sensor_mode == "unsigned" and not self.cfg.ac_mode_entity:
            self.log(
                "battery_sensor_mode='unsigned' requires ac_mode_entity",
                level="ERROR",
            )

        self._check_entities()

    def terminate(self) -> None:
        """Called by AppDaemon on shutdown — save state for next startup."""
        self._save_state()

    # ─── State persistence ────────────────────────────────

    def _save_state(self) -> None:
        """Atomically write controller state to JSON file."""
        try:
            tmp = self._state_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.logic.state_snapshot(), f)
            os.replace(tmp, self._state_file)
        except OSError as exc:
            self.log(f"Failed to save state: {exc}", level="WARNING")

    def _restore_state(self) -> bool:
        """Try to restore controller state from JSON file.

        Returns True if state was successfully restored.
        """
        try:
            with open(self._state_file) as f:
                data = json.load(f)
            return self.logic.restore_from_snapshot(data)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return False

    def _check_entities(self) -> None:
        """Log availability of every configured HA entity at startup."""
        entities = {
            "grid_sensor": self.cfg.grid_sensor,
            "soc_sensor": self.cfg.soc_sensor,
            "battery_power": self.cfg.battery_power_sensor,
        }
        if self.cfg.ac_mode_entity:
            entities["ac_mode"] = self.cfg.ac_mode_entity
        if self.cfg.charge_switch:
            entities["charge_switch"] = self.cfg.charge_switch
        if self.cfg.discharge_switch:
            entities["discharge_switch"] = self.cfg.discharge_switch
        if self.cfg.relay_locked_sensor:
            entities["relay_locked"] = self.cfg.relay_locked_sensor
        for entity in self.logic.ff.entities:
            entities[f"ff:{entity}"] = entity
        for label, entity_id in entities.items():
            raw = self.get_state(entity_id)
            if raw in UNAVAILABLE_STATES:
                self.log(f"  ✗ {label}: {entity_id} → {raw!r}", level="WARNING")
            else:
                self.log(f"  ✓ {label}: {entity_id} → {raw}")

    # ─── Tick ─────────────────────────────────────────────

    def _on_tick(self, _kwargs: dict) -> None:
        """Called every ``interval_s`` seconds by AppDaemon's scheduler."""
        m = self._read_measurement()
        if m is None:
            return

        output = self.logic.compute(m)
        surplus = ControlLogic.estimate_surplus(m)
        self._log_output(output, m, surplus)
        self._publish_ha_sensors(output, m, surplus)
        self._log_csv(output, m, surplus)

        self._tick_count += 1
        if self._tick_count % self._state_save_interval == 0:
            self._save_state()

    # ─── Sensor reading ──────────────────────────────────

    def _read_float(self, entity: str) -> float | None:
        """Read an HA entity's state as a float, returning None on failure."""
        raw: str | None = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            return None
        try:
            return float(raw)
        except (ValueError, TypeError):
            self.log(f"  {entity} → {raw!r} (not a number)", level="WARNING")
            return None

    def _read_measurement(self) -> Measurement | None:
        """Assemble a ``Measurement`` from HA sensor states.

        Returns ``None`` (and logs a warning) when any required sensor
        is unavailable, causing the tick to be skipped.
        """
        grid = self._read_float(self.cfg.grid_sensor)
        soc = self._read_float(self.cfg.soc_sensor)
        bp = self._read_float(self.cfg.battery_power_sensor)

        unavailable = []
        if grid is None:
            unavailable.append(f"grid ({self.cfg.grid_sensor})")
        if soc is None:
            unavailable.append(f"soc ({self.cfg.soc_sensor})")
        if bp is None:
            unavailable.append(f"battery_power ({self.cfg.battery_power_sensor})")

        if unavailable:
            self.log(
                f"Sensor unavailable: {', '.join(unavailable)}, skipping cycle",
                level="WARNING",
            )
            return None

        # Derive signed battery power from sensor mode
        if self.cfg.battery_sensor_mode == "unsigned":
            # Unsigned sensor: always positive, sign from AC mode entity
            ac_mode = self.get_state(self.cfg.ac_mode_entity)
            if ac_mode == "Input mode":
                battery_power = -abs(bp)   # charging → negative
            else:
                battery_power = abs(bp)    # discharging → positive
        else:
            # Signed sensor: +discharge/-charge (no transformation)
            battery_power = bp

        # Read feed-forward sensor readings
        ff_readings: dict[str, float | None] = {}
        for entity in self.logic.ff.entities:
            ff_readings[entity] = self._read_float(entity)

        return Measurement(
            grid_power_w=grid,
            soc_pct=soc,
            battery_power_w=battery_power,
            ff_readings=ff_readings,
            discharge_enabled=self._is_switch_on(self.cfg.discharge_switch),
            charge_enabled=self._is_switch_on(self.cfg.charge_switch),
            relay_locked=self._is_relay_locked(),
            dynamic_min_soc_pct=self._read_float(self.cfg.dynamic_min_soc_entity)
            if self.cfg.dynamic_min_soc_entity
            else None,
        )

    def _is_relay_locked(self) -> bool:
        """Return True if the driver's relay SM is clamping output."""
        entity = self.cfg.relay_locked_sensor
        if entity is None:
            return False
        return self.get_state(entity) == "true"

    def _is_switch_on(self, entity: str | None) -> bool:
        """Return True if the switch entity is on, or if no entity is configured."""
        if entity is None:
            return True
        return self.get_state(entity) == "on"

    # ─── Output ──────────────────────────────────────────

    def _log_output(
        self, output: ControlOutput, m: Measurement, surplus: float
    ) -> None:
        """Emit a single-line summary of the control cycle to the AppDaemon log."""
        mode = "DRY" if self.cfg.dry_run else "LIVE"
        power = output.desired_power_w
        if power >= 0:
            power_str = f"discharge={power:.0f}W"
        else:
            power_str = f"charge={-power:.0f}W"

        self.log(
            f"{mode} | grid={m.grid_power_w:.0f}W "
            f"surplus={surplus:.0f}W "
            f"soc={m.soc_pct:.0f}% "
            f"mode={self.logic.state.mode.name} "
            f"{power_str} | "
            f"P={output.p_term:.0f} I={output.i_term:.0f} "
            f"FF={output.ff_term:.0f} | "
            f"{output.reason}"
        )

    def _log_csv(
        self, output: ControlOutput, m: Measurement, surplus: float,
    ) -> None:
        """Append one row to the CSV log file (if file logging is enabled)."""
        if self._csv is None:
            return
        target = self.logic.target_for_mode()
        self._csv.log_row({
            "grid_w": round(m.grid_power_w),
            "soc_pct": round(m.soc_pct, 1),
            "battery_power_w": round(m.battery_power_w),
            "surplus_w": round(surplus),
            "mode": self.logic.state.mode.name,
            "desired_power_w": round(output.desired_power_w),
            "p_term": round(output.p_term),
            "i_term": round(output.i_term),
            "ff_term": round(output.ff_term),
            "integral": round(self.logic.state.integral),
            "target_w": round(target),
            "error_w": round(m.grid_power_w - target),
            "in_deadband": int(self.logic._last_in_deadband),
            "relay_locked": int(m.relay_locked),
            "charge_pending": int(self.logic.state.charge_pending_since is not None),
            "kp_used": self.logic._last_kp,
            "ki_used": self.logic._last_ki,
            "reason": output.reason,
        })

    # ─── HA sensor publishing ────────────────────────────

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
        icon: str | None = None,
    ) -> None:
        """Create or update a ``sensor.zfi_<name>`` entity in HA."""
        entity_id = f"{self.cfg.sensor_prefix}_{name}"
        attrs = {"friendly_name": f"ZFI {name.replace('_', ' ').title()}"}
        if unit:
            attrs["unit_of_measurement"] = unit
            attrs["state_class"] = "measurement"
        if icon:
            attrs["icon"] = icon
        try:
            self.set_state(entity_id, state=str(value), attributes=attrs, replace=True)
        except Exception as exc:
            self.log(
                f"set_state failed for {entity_id}: {exc}",
                level="WARNING",
            )

    def _publish_ha_sensors(
        self,
        output: ControlOutput,
        m: Measurement,
        surplus: float,
    ) -> None:
        """Push all ``sensor.zfi_*`` entities to HA for dashboards and the driver."""
        # Desired power (the main output for the driver)
        self._set_sensor(
            "desired_power",
            round(output.desired_power_w),
            "W",
            "mdi:transmission-tower",
        )

        # Operating regime
        self._set_sensor(
            "mode",
            self.logic.state.mode.name.lower(),
            icon="mdi:swap-vertical",
        )

        if not self.cfg.debug:
            return

        # ── Debug-only sensors below ─────────────────────

        # Physical estimates
        self._set_sensor("surplus", round(surplus), "W", "mdi:solar-power")
        self._set_sensor(
            "battery_power",
            round(m.battery_power_w),
            "W",
            "mdi:battery-charging",
        )

        # PI internals
        self._set_sensor("p_term", round(output.p_term), "W", "mdi:alpha-p-box")
        self._set_sensor("i_term", round(output.i_term), "W", "mdi:alpha-i-box")
        self._set_sensor(
            "ff", round(output.ff_term), "W", "mdi:flash",
        )
        pv = next((d for d in self.logic.ff.last_debug if d.name == "pv"), None)
        if pv is not None:
            self._set_sensor(
                "ff_pv_raw",
                round(pv.raw_w) if pv.raw_w is not None else "unavailable",
                "W", "mdi:solar-power-variant",
            )
            self._set_sensor(
                "ff_pv_ema",
                round(pv.ema_w) if pv.ema_w is not None else "unavailable",
                "W", "mdi:chart-bell-curve",
            )
            self._set_sensor("ff_pv_contrib", round(pv.contrib_w), "W", "mdi:flash-outline")
        others = sum(d.contrib_w for d in self.logic.ff.last_debug if d.name != "pv")
        self._set_sensor("ff_others_contrib", round(others), "W", "mdi:flash-outline")
        self._set_sensor(
            "integral", round(self.logic.state.integral), "W", "mdi:sigma"
        )
        target_w = self.logic.target_for_mode()
        self._set_sensor("target", round(target_w), "W", "mdi:target")
        self._set_sensor(
            "error", round(m.grid_power_w - target_w), "W", "mdi:delta"
        )
        self._set_sensor("reason", output.reason, icon="mdi:information-outline")

        # Dynamic min SOC (shows effective value after clamping)
        if m.dynamic_min_soc_pct is not None:
            effective = max(
                self.cfg.min_soc_pct,
                min(m.dynamic_min_soc_pct, self.cfg.max_soc_pct),
            )
        else:
            effective = self.cfg.min_soc_pct
        self._set_sensor(
            "effective_min_soc", round(effective), "%", "mdi:battery-low",
        )
