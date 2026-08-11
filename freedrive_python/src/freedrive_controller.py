"""RDK operations and freedrive primitive lifecycle for Flexiv RDK v1.9."""

from __future__ import annotations

import select
import sys
import time
from typing import Any, Callable, TextIO

from src.foot_pedal_configuration import FootPedalConfiguration, PedalBinding
from src.foot_pedal_input import (
    FootPedalReader,
    PedalEvent,
    StubPedalActionHandler,
    XInputFootPedalReader,
)
from src.freedrive_configuration import (
    EXPECTED_ROBOT_MODEL,
    EXPECTED_ROBOT_SOFTWARE_SERIES,
    FreedriveConfiguration,
    normalize_robot_model,
    software_version_is_supported,
)
from src.freedrive_csv_logger import FreedriveCSVLogger
from src.freedrive_state_machine import (
    Action,
    DebouncedButton,
    FreedriveState,
    FreedriveStateMachine,
    StateMachineInputs,
)


class FreedriveError(RuntimeError):
    """Operator-actionable freedrive failure."""


class SafetyError(FreedriveError):
    """Safety gate failure."""


class OperationTimeout(FreedriveError):
    """A finite wait timed out."""


SCHEMA_NOTE = (
    "Robot Software 3.11 documents FloatingCartesian for Rizon 4(s) with "
    "floatingAxis and enableElbowMotion. RDK 1.9 accepts primitive boolean "
    "values with integer 1/0."
)
class FreedriveController:
    def __init__(
        self,
        robot: Any,
        primitive_mode: Any,
        config: FreedriveConfiguration,
        *,
        sink: FreedriveCSVLogger | None = None,
        coord_factory: Any | None = None,
        tool_api: Any | None = None,
        mode_names: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        output: Callable[[str], None] = print,
        stdin: TextIO | None = None,
        foot_pedal_config: FootPedalConfiguration | None = None,
        foot_pedal_reader: FootPedalReader | XInputFootPedalReader | None = None,
    ) -> None:
        self.robot = robot
        self.primitive_mode = primitive_mode
        self.config = config
        self.sink = sink
        self.coord_factory = coord_factory
        self.tool_api = tool_api
        self.mode_names = mode_names
        self.monotonic = monotonic
        self.sleep = sleep
        self.output = output
        self.stdin = stdin if stdin is not None else sys.stdin
        self.foot_pedal_config = foot_pedal_config
        self.foot_pedal_reader = foot_pedal_reader
        self.machine = FreedriveStateMachine()
        self.button = DebouncedButton(
            debounce_s=config.debounce_s, monotonic=monotonic
        )
        self._started = monotonic()
        self._active_tool = "unknown"
        self._tool_mass: float | str = "unknown"
        self._quit = False
        self._extension_handler = StubPedalActionHandler(output=output)

    @property
    def use_foot_pedal_toggle(self) -> bool:
        cfg = self.foot_pedal_config
        return (
            cfg is not None
            and cfg.freedrive_toggle_ready()
            and self.foot_pedal_reader is not None
        )

    def _on_pedal_extension(self, event: PedalEvent, binding: PedalBinding) -> None:
        """Extension hook for Pedal 1 / Pedal 3 actions (not implemented yet).

        Later: replace StubPedalActionHandler or override this method to run
        portable-device / pre-programmed behaviors when those actions are armed.
        """
        self._extension_handler.on_key_down(event, binding)

    def _poll_foot_pedal_toggle(self) -> bool:
        """Return True once when Pedal 2 key-down requests a Freedrive toggle."""
        reader = self.foot_pedal_reader
        if reader is None or not self.use_foot_pedal_toggle:
            return False
        toggle = False
        for event in reader.poll():
            if event.action == "freedrive_toggle" and event.pedal_id == "pedal_2":
                toggle = True
                self.output(
                    f"foot pedal Pedal 2 KEY_DOWN (key_code={event.key_code}): "
                    "freedrive toggle"
                )
            else:
                try:
                    binding = self.foot_pedal_config.pedal(event.pedal_id)  # type: ignore[union-attr]
                except Exception:
                    binding = PedalBinding(
                        event.pedal_id,
                        event.key_code,
                        event.action,
                        event.intended_function,
                    )
                self._on_pedal_extension(event, binding)
        return toggle

    def request_quit(self) -> None:
        self._quit = True

    def _check_identity(self) -> None:
        info = self.robot.info()
        model = normalize_robot_model(info.model_name)
        if model != normalize_robot_model(EXPECTED_ROBOT_MODEL):
            raise SafetyError(
                f"this project only allows {EXPECTED_ROBOT_MODEL}; "
                f"robot reported {info.model_name!r}"
            )
        if not software_version_is_supported(info.software_ver):
            raise SafetyError(
                f"Robot Software {EXPECTED_ROBOT_SOFTWARE_SERIES}.x is required "
                "by Flexiv RDK 1.9.0; "
                f"robot reported {info.software_ver!r}"
            )
        manipulator_dof = getattr(info, "DoF_m", 7)
        if int(manipulator_dof) != 7:
            raise SafetyError(
                "Rizon 4S must report 7 manipulator joints; "
                f"robot reported DoF_m={manipulator_dof!r}"
            )
        self.output(
            f"robot={info.model_name}, software={info.software_ver}, "
            f"license={info.license_type}"
        )

    def _read_tool(self) -> None:
        if self.tool_api is None:
            return
        try:
            self._active_tool = str(self.tool_api.name())
            params = self.tool_api.params()
            self._tool_mass = float(params.mass)
            self.output(
                f"active_tool={self._active_tool}, mass_kg={self._tool_mass}, "
                f"CoM={list(params.CoM)}, inertia={list(params.inertia)}"
            )
            if self._active_tool != "Flange" and float(self._tool_mass) <= 0.0:
                self.output(
                    "WARNING: active tool is not Flange but mass is zero/unknown. "
                    "If a physical tool is mounted, stop freedrive until payload is calibrated."
                )
        except Exception as exc:
            self.output(f"WARNING: unable to read tool parameters: {exc}")

    def _wait_operational(self) -> None:
        def status() -> str:
            try:
                return str(self.robot.operational_status())
            except Exception:
                return "unknown"

        self.output(
            "Waiting for robot operational status READY "
            f"(current={status()}). Check E-stop release and Auto mode "
            "(some Elements versions label this Auto/Remote)."
        )
        deadline = self.monotonic() + self.config.startup_timeout_s
        while not self.robot.operational():
            if not self.robot.connected() or self.robot.fault():
                raise SafetyError(
                    "robot not ready while waiting for operational "
                    f"(status={status()})"
                )
            if self.monotonic() >= deadline:
                raise OperationTimeout(
                    "timed out waiting for operational "
                    f"(status={status()}); verify E-stop is released, robot is "
                    "enabled, brakes are released, and mode is Auto"
                )
            self.sleep(0.1)

    def _mode_name(self) -> str:
        try:
            mode = self.robot.mode()
            if self.mode_names is not None:
                return str(self.mode_names[int(mode)])
            return str(mode)
        except Exception:
            return "unknown"

    def _primitive_terminated(self) -> bool:
        try:
            states = self.robot.primitive_states()
        except Exception:
            return False
        value = states.get("terminated", 0)
        try:
            return int(value) != 0
        except Exception:
            return bool(value)

    def _poll_quit_char(self) -> bool:
        if self._quit:
            return True
        reader = self.foot_pedal_reader
        if reader is not None and getattr(reader, "owns_stdin", False):
            return bool(getattr(reader, "quit_requested", False))
        stream = self.stdin
        if stream is None or not hasattr(stream, "fileno"):
            return False
        try:
            if not stream.isatty():
                return False
            ready, _, _ = select.select([stream], [], [], 0)
            if not ready:
                return False
            line = stream.readline()
        except Exception:
            return False
        return line.strip().lower() in {"q", "quit"}

    def _log_sample(self, button: bool) -> None:
        states = self.robot.states()
        tcp_pose = list(states.tcp_pose)
        joint_positions = list(states.q)
        if self.sink is None:
            return
        try:
            primitive_state = dict(self.robot.primitive_states())
        except Exception:
            primitive_state = {}
        self.sink.write_states(
            elapsed_s=self.monotonic() - self._started,
            state_machine=self.machine.state.name,
            enabling_button=button,
            primitive_state=primitive_state,
            tcp_pose=tcp_pose,
            q=joint_positions,
            dq=list(getattr(states, "dq", [])),
            ext_wrench_tcp=list(getattr(states, "ext_wrench_in_tcp", [])),
            ext_wrench_tcp_raw=list(getattr(states, "ext_wrench_in_tcp_raw", [])),
            tau_ext=list(getattr(states, "tau_ext", [])),
            fault=bool(self.robot.fault()),
            operational=bool(self.robot.operational()),
            connected=bool(self.robot.connected()),
            mode=self._mode_name(),
            active_tool=self._active_tool,
            tool_mass_kg=self._tool_mass,
        )

    def _start_primitive(self) -> None:
        if not self.robot.operational():
            raise SafetyError("refusing to start freedrive while not operational")
        if self.robot.fault():
            raise SafetyError("refusing to start freedrive while faulted")
        params = self.config.primitive_params(self.coord_factory)
        self.output(
            "Starting freedrive session "
            f"#{self.machine.session_count + 1}: "
            f"primitive={self.config.primitive_name()} params={params} "
            f"active_tool={self._active_tool}"
        )
        self.robot.SwitchMode(self.primitive_mode)
        if not self.use_foot_pedal_toggle and not self.robot.enabling_button_pressed():
            raise SafetyError(
                "enabling button released during mode switch; not starting primitive"
            )
        try:
            self.robot.ExecutePrimitive(self.config.primitive_name(), params)
        except Exception as exc:
            raise self._translate_primitive_error(exc) from exc
        self.machine.mark_started()
        if self.use_foot_pedal_toggle:
            self.output(
                "Freedrive active (Pedal 2 toggle). Press Pedal 2 again to stop this "
                "session without exiting. Ctrl+C or 'q'+Enter terminates the program."
            )
            self.output(
                "SAFETY: if the robot moves by itself with no contact, press Pedal 2 "
                "to exit Freedrive, press E-stop if needed, and stop testing until "
                "tool/payload/mounting/dynamics are verified in Elements."
            )
        else:
            self.output(
                "Freedrive active. Hold enabling button. Release to stop this session "
                "without exiting. Ctrl+C or 'q'+Enter terminates the program."
            )
            self.output(
                "SAFETY: if the robot moves by itself with no contact, release the "
                "enabling button immediately, press E-stop if needed, and stop testing "
                "until tool/payload/mounting/dynamics are verified in Elements."
            )

    @staticmethod
    def _translate_primitive_error(exc: Exception) -> FreedriveError:
        text = str(exc)
        lower = text.lower()
        if any(token in lower for token in ("license", "unlicensed")):
            return FreedriveError(
                f"primitive or NRT_PRIMITIVE_EXECUTION license unavailable: {text}"
            )
        if any(
            token in lower
            for token in ("parameter", "unknown", "unsupported", "invalid argument")
        ):
            return FreedriveError(
                f"robot rejected primitive parameters: {text}. {SCHEMA_NOTE}"
            )
        return FreedriveError(f"primitive start failed: {text}")

    def _stop_session(self) -> None:
        self.output("Stopping current freedrive session (program stays alive)...")
        self.robot.Stop()
        deadline = self.monotonic() + self.config.stop_timeout_s
        while not self.robot.stopped():
            if self.monotonic() >= deadline:
                raise OperationTimeout("timed out waiting for robot.stopped()")
            if self.robot.fault() or not self.robot.connected():
                break
            self.sleep(0.05)
        if self.use_foot_pedal_toggle:
            assert self.foot_pedal_reader is not None
            self.foot_pedal_reader.rearm_after_stop(sleep=self.sleep)
            self.output(
                "Session stopped; queued presses were discarded and Pedal 2 was "
                "released/rearmed. Press it again to start the next session."
            )
        else:
            self.output(
                "Session stopped; waiting for enabling button for the next session."
            )

    def _ensure_locked(self) -> None:
        """Enter a known stopped state without commanding a Home trajectory."""
        self.output(
            "Startup safety state: calling Stop(); no Home or other trajectory "
            "will run. Freedrive remains locked until the configured control is pressed."
        )
        self.robot.Stop()
        deadline = self.monotonic() + self.config.stop_timeout_s
        while not self.robot.stopped():
            if self.monotonic() >= deadline:
                raise OperationTimeout("timed out waiting for locked/stopped startup state")
            if not self.robot.connected() or self.robot.fault():
                raise SafetyError("disconnect/fault while entering locked startup state")
            self.sleep(0.05)
        if self.use_foot_pedal_toggle:
            self.output(
                "Robot is locked/stopped and ready for foot-pedal arming."
            )
        else:
            self.output(
                "Robot is locked/stopped. Waiting for the enabling button."
            )

    def _zero_ft_sensor(self) -> None:
        """Reset the session F/T offset before Cartesian freedrive."""
        self.output(
            "ZeroFTSensor: keep the last joint, flange, and tool completely "
            "untouched until zeroing finishes. No Home trajectory will run."
        )
        self.robot.SwitchMode(self.primitive_mode)
        self.robot.ExecutePrimitive("ZeroFTSensor", {})
        deadline = self.monotonic() + self.config.startup_timeout_s
        while not self._primitive_terminated():
            if not self.robot.connected() or self.robot.fault():
                raise SafetyError("disconnect/fault while running ZeroFTSensor")
            if self.monotonic() >= deadline:
                raise OperationTimeout("ZeroFTSensor did not finish before timeout")
            self.sleep(0.05)
        self.output(
            "ZeroFTSensor finished; TCP wrench="
            f"{list(self.robot.states().ext_wrench_in_world)}"
        )
        self._ensure_locked()

    def run_diagnose_only(self) -> None:
        """Record diagnostics without starting a floating primitive."""
        self._check_identity()
        if not self.robot.operational():
            self.robot.Enable()
            self._wait_operational()
        self._read_tool()
        self.output(
            "Diagnose-only mode: recording state without ExecutePrimitive. "
            "Ctrl+C or 'q'+Enter to stop."
        )
        try:
            while not self._poll_quit_char():
                if not self.robot.connected() or self.robot.fault():
                    raise SafetyError("diagnose aborted due to disconnect/fault")
                button = bool(self.robot.enabling_button_pressed())
                self._log_sample(button)
                self.output(
                    f"diagnose t={self.monotonic() - self._started:7.2f}s "
                    f"button={int(button)} tool={self._active_tool} "
                    f"fault={int(self.robot.fault())}"
                )
                self.sleep(self.config.sample_period_s)
        finally:
            try:
                self.robot.Stop()
            except Exception as exc:
                self.output(f"WARNING: Stop() during diagnose cleanup failed: {exc}")

    def run(self) -> None:
        """Persistent enabling-button loop; only whole-program exits terminate."""
        self.config.validate()

        try:
            if not self.robot.connected():
                raise SafetyError("robot is not connected")
            self._check_identity()
            if self.robot.fault():
                raise SafetyError(
                    "robot is in fault; clear it in Elements before freedrive"
                )
            if not self.robot.operational():
                self.robot.Enable()
                self._wait_operational()
            self._read_tool()

            if self.config.diagnose_only:
                self.run_diagnose_only()
                return

            self._ensure_locked()
            self._zero_ft_sensor()
            if self.use_foot_pedal_toggle:
                assert self.foot_pedal_reader is not None
                self.foot_pedal_reader.arm()
                self.output(
                    "Pedal 2 armed after clearing startup events and verifying it "
                    "is released. Waiting for a new KEY_DOWN."
                )
            report = self.config.command_report(self.coord_factory)
            self.output(f"Resolved freedrive command: {report}")
            if self.use_foot_pedal_toggle:
                info = (
                    self.foot_pedal_reader.device_info
                    if self.foot_pedal_reader is not None
                    else None
                )
                device_note = (
                    f" device={info.path} ({info.name!r})"
                    if info is not None
                    else ""
                )
                self.output(
                    "Pedal 2 toggle control armed"
                    f"{device_note}. First KEY_DOWN enters Freedrive; each later "
                    "KEY_DOWN alternates exit/enter. KEY_UP / KEY_REPEAT / bounce "
                    "are ignored. Program exits only on Ctrl+C, 'q'+Enter, fault, "
                    "or disconnect."
                )
            else:
                self.output(
                    "Waiting for enabling button. Press-release-press-release starts "
                    "and stops sessions. Program exits only on Ctrl+C, 'q'+Enter, "
                    "fault, disconnect, or program timeout."
                )

            try:
                while self.machine.state not in {
                    FreedriveState.TERMINATING,
                    FreedriveState.ERROR,
                }:
                    connected = bool(self.robot.connected())
                    fault = bool(self.robot.fault()) if connected else True
                    toggle_edge = self._poll_foot_pedal_toggle()
                    # XInput reader drains the HID keyboard character and records
                    # q itself, so poll it before checking quit.
                    quit_requested = self._poll_quit_char()
                    use_toggle = self.use_foot_pedal_toggle

                    if use_toggle:
                        # Pedal toggle replaces enabling-button hold-to-run.
                        stable_button = False
                        rising = False
                        falling = False
                    else:
                        raw_button = (
                            bool(self.robot.enabling_button_pressed())
                            if connected
                            else False
                        )
                        stable_button, rising, falling = self.button.update(raw_button)

                    terminated = False
                    if self.machine.state == FreedriveState.FREEDRIVE_ACTIVE:
                        terminated = self._primitive_terminated()

                    inputs = StateMachineInputs(
                        button_pressed=stable_button,
                        rising_edge=rising,
                        falling_edge=falling,
                        fault=fault,
                        connected=connected,
                        quit_requested=quit_requested,
                        primitive_terminated=terminated,
                        toggle_edge=toggle_edge,
                        use_toggle_control=use_toggle,
                    )
                    action = self.machine.on_inputs(inputs)

                    if action == Action.START_PRIMITIVE:
                        try:
                            self._start_primitive()
                        except Exception:
                            self.machine.mark_start_failed()
                            raise
                    elif action == Action.STOP_SESSION:
                        self._stop_session()
                        inputs.stop_complete = True
                        inputs.toggle_edge = False
                        self.machine.on_inputs(inputs)
                    elif action == Action.TERMINATE:
                        break

                    self._log_sample(stable_button)
                    if self.machine.state == FreedriveState.WAITING_FOR_ENABLE:
                        waiting_for = (
                            "Pedal 2 KEY_DOWN"
                            if use_toggle
                            else "enabling button"
                        )
                        self.output(
                            f"[{self.machine.state.name}] waiting for {waiting_for} "
                            f"(sessions_completed={self.machine.session_count})"
                        )
                    elif self.machine.state == FreedriveState.FREEDRIVE_ACTIVE:
                        states = self.robot.states()
                        self.output(
                            f"[{self.machine.state.name}] "
                            f"tcp={list(states.tcp_pose)} q={list(states.q)}"
                        )
                    self.sleep(self.config.sample_period_s)
            finally:
                if self.foot_pedal_reader is not None:
                    self.foot_pedal_reader.close()
        finally:
            try:
                if self.robot.operational() or not self.robot.stopped():
                    self.robot.Stop()
            except Exception as exc:
                self.output(
                    f"WARNING: Stop() failed during program cleanup; "
                    f"use physical E-stop: {exc}"
                )
