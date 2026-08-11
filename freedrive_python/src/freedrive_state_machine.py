"""Enabling-button / foot-pedal state machine for persistent freedrive sessions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable


class FreedriveState(Enum):
    WAITING_FOR_ENABLE = auto()
    STARTING_FREEDRIVE = auto()
    FREEDRIVE_ACTIVE = auto()
    STOPPING_FREEDRIVE = auto()
    WAITING_FOR_RELEASE = auto()
    TERMINATING = auto()
    ERROR = auto()


class Action(Enum):
    NONE = auto()
    START_PRIMITIVE = auto()
    STOP_SESSION = auto()
    TERMINATE = auto()


@dataclass
class DebouncedButton:
    """Require a stable button reading for debounce_s before accepting edges."""

    debounce_s: float
    monotonic: Callable[[], float]
    raw: bool = False
    stable: bool = False
    candidate: bool | None = None
    candidate_since: float | None = None

    def update(self, raw_pressed: bool) -> tuple[bool, bool, bool]:
        """Return (stable_pressed, rising_edge, falling_edge)."""
        now = self.monotonic()
        if self.candidate is None or raw_pressed != self.candidate:
            self.candidate = raw_pressed
            self.candidate_since = now
            if self.debounce_s > 0.0:
                return self.stable, False, False
        elif self.debounce_s > 0.0:
            since = self.candidate_since
            if since is None or (now - since) < self.debounce_s:
                return self.stable, False, False
        if raw_pressed == self.stable:
            return self.stable, False, False
        previous = self.stable
        self.stable = raw_pressed
        rising = (not previous) and self.stable
        falling = previous and (not self.stable)
        return self.stable, rising, falling


@dataclass
class StateMachineInputs:
    button_pressed: bool
    rising_edge: bool
    falling_edge: bool
    fault: bool
    connected: bool
    quit_requested: bool
    primitive_terminated: bool
    start_failed: bool = False
    stop_complete: bool = False
    # Foot-pedal Pedal 2: key-down edge toggles enter/exit (not hold-to-run).
    toggle_edge: bool = False
    use_toggle_control: bool = False


class FreedriveStateMachine:
    def __init__(self) -> None:
        self.state = FreedriveState.WAITING_FOR_ENABLE
        self.session_count = 0

    def on_inputs(self, inputs: StateMachineInputs) -> Action:
        if self.state in {FreedriveState.TERMINATING, FreedriveState.ERROR}:
            return Action.NONE

        if (
            inputs.quit_requested
            or inputs.fault
            or not inputs.connected
        ):
            if self.state in {
                FreedriveState.STARTING_FREEDRIVE,
                FreedriveState.FREEDRIVE_ACTIVE,
            }:
                self.state = FreedriveState.TERMINATING
                return Action.TERMINATE
            self.state = FreedriveState.TERMINATING
            return Action.TERMINATE

        if self.state == FreedriveState.WAITING_FOR_ENABLE:
            if inputs.use_toggle_control:
                if inputs.toggle_edge:
                    self.state = FreedriveState.STARTING_FREEDRIVE
                    return Action.START_PRIMITIVE
                return Action.NONE
            if inputs.rising_edge:
                self.state = FreedriveState.STARTING_FREEDRIVE
                return Action.START_PRIMITIVE
            return Action.NONE

        if self.state == FreedriveState.STARTING_FREEDRIVE:
            if inputs.start_failed:
                self.state = FreedriveState.ERROR
                return Action.TERMINATE
            # Successful start moves to ACTIVE via mark_started().
            return Action.NONE

        if self.state == FreedriveState.FREEDRIVE_ACTIVE:
            if inputs.use_toggle_control:
                if inputs.toggle_edge:
                    self.state = FreedriveState.STOPPING_FREEDRIVE
                    return Action.STOP_SESSION
                if inputs.primitive_terminated:
                    # Toggle mode: after controller stop, return to waiting.
                    self.state = FreedriveState.STOPPING_FREEDRIVE
                    return Action.STOP_SESSION
                return Action.NONE
            if inputs.falling_edge:
                self.state = FreedriveState.STOPPING_FREEDRIVE
                return Action.STOP_SESSION
            if inputs.primitive_terminated:
                # Do not auto-restart; require a full release then new rising edge.
                self.state = FreedriveState.WAITING_FOR_RELEASE
                return Action.STOP_SESSION
            return Action.NONE

        if self.state == FreedriveState.STOPPING_FREEDRIVE:
            if inputs.stop_complete:
                if inputs.use_toggle_control:
                    self.state = FreedriveState.WAITING_FOR_ENABLE
                elif inputs.button_pressed:
                    self.state = FreedriveState.WAITING_FOR_RELEASE
                else:
                    self.state = FreedriveState.WAITING_FOR_ENABLE
            return Action.NONE

        if self.state == FreedriveState.WAITING_FOR_RELEASE:
            # Hold-to-run only: require a full release before the next rising edge.
            if not inputs.button_pressed:
                self.state = FreedriveState.WAITING_FOR_ENABLE
            return Action.NONE

        return Action.NONE

    def mark_started(self) -> None:
        if self.state == FreedriveState.STARTING_FREEDRIVE:
            self.state = FreedriveState.FREEDRIVE_ACTIVE
            self.session_count += 1

    def mark_start_failed(self) -> None:
        self.state = FreedriveState.ERROR
