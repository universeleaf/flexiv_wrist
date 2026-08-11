from __future__ import annotations

from types import SimpleNamespace

from src.freedrive_configuration import FreedriveConfiguration
from src.freedrive_controller import FreedriveController
from src.freedrive_state_machine import (
    Action,
    DebouncedButton,
    FreedriveState,
    FreedriveStateMachine,
    StateMachineInputs,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeRobot:
    def __init__(self) -> None:
        self.button = False
        self.is_connected = True
        self.is_fault = False
        self.is_operational = True
        self.is_stopped = True
        self.calls: list[object] = []
        self._terminated = 0
        self._reached_target = 0
        self._mode = "IDLE"

    def connected(self) -> bool:
        return self.is_connected

    def fault(self) -> bool:
        return self.is_fault

    def operational(self) -> bool:
        return self.is_operational

    def stopped(self) -> bool:
        return self.is_stopped

    def info(self) -> SimpleNamespace:
        return SimpleNamespace(
            model_name="Rizon4s",
            software_ver="v3.11.0",
            license_type="RDK-Professional",
            DoF_m=7,
        )

    def enabling_button_pressed(self) -> bool:
        return self.button

    def Enable(self) -> None:
        self.calls.append("Enable")
        self.is_operational = True

    def SwitchMode(self, mode: object) -> None:
        self.calls.append(("SwitchMode", mode))
        self._mode = str(mode)

    def ExecutePrimitive(self, name: str, params: dict) -> None:
        self.calls.append(("ExecutePrimitive", name, params))
        self.is_stopped = False
        self._terminated = 1 if name == "ZeroFTSensor" else 0
        self._reached_target = 1 if name == "Home" else 0

    def primitive_states(self) -> dict:
        return {
            "terminated": self._terminated,
            "reachedTarget": self._reached_target,
            "primitiveName": "FloatingCartesian",
        }

    def states(self) -> SimpleNamespace:
        return SimpleNamespace(
            tcp_pose=[0.0] * 7,
            q=[0.0] * 7,
            dq=[0.0] * 7,
            ext_wrench_in_tcp=[0.0] * 6,
            ext_wrench_in_tcp_raw=[0.0] * 6,
            ext_wrench_in_world=[0.0] * 6,
            tau_ext=[0.0] * 7,
        )

    def mode(self) -> str:
        return self._mode

    def Stop(self) -> None:
        self.calls.append("Stop")
        self.is_stopped = True
        self._terminated = 1
        self._mode = "IDLE"


def _inputs(**kwargs) -> StateMachineInputs:
    base = dict(
        button_pressed=False,
        rising_edge=False,
        falling_edge=False,
        fault=False,
        connected=True,
        quit_requested=False,
        primitive_terminated=False,
    )
    base.update(kwargs)
    return StateMachineInputs(**base)


def test_rising_edge_starts_one_primitive_action() -> None:
    sm = FreedriveStateMachine()
    action = sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    assert action == Action.START_PRIMITIVE
    assert sm.state == FreedriveState.STARTING_FREEDRIVE
    sm.mark_started()
    assert sm.state == FreedriveState.FREEDRIVE_ACTIVE
    # Holding does not request another start.
    action = sm.on_inputs(_inputs(button_pressed=True))
    assert action == Action.NONE
    assert sm.session_count == 1


def test_falling_edge_stops_session_but_not_program() -> None:
    sm = FreedriveStateMachine()
    sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    sm.mark_started()
    action = sm.on_inputs(_inputs(falling_edge=True, button_pressed=False))
    assert action == Action.STOP_SESSION
    assert sm.state == FreedriveState.STOPPING_FREEDRIVE
    sm.on_inputs(_inputs(button_pressed=False, stop_complete=True))
    assert sm.state == FreedriveState.WAITING_FOR_ENABLE


def test_second_rising_edge_starts_second_session() -> None:
    sm = FreedriveStateMachine()
    sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    sm.mark_started()
    sm.on_inputs(_inputs(falling_edge=True, button_pressed=False))
    sm.on_inputs(_inputs(button_pressed=False, stop_complete=True))
    action = sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    assert action == Action.START_PRIMITIVE
    sm.mark_started()
    assert sm.session_count == 2


def test_quit_or_fault_terminates_program() -> None:
    sm = FreedriveStateMachine()
    action = sm.on_inputs(_inputs(quit_requested=True))
    assert action == Action.TERMINATE
    assert sm.state == FreedriveState.TERMINATING

    sm = FreedriveStateMachine()
    sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    sm.mark_started()
    action = sm.on_inputs(_inputs(button_pressed=True, fault=True))
    assert action == Action.TERMINATE


def test_internal_primitive_termination_requires_release() -> None:
    sm = FreedriveStateMachine()
    sm.on_inputs(_inputs(rising_edge=True, button_pressed=True))
    sm.mark_started()
    action = sm.on_inputs(
        _inputs(button_pressed=True, primitive_terminated=True)
    )
    assert action == Action.STOP_SESSION
    assert sm.state == FreedriveState.WAITING_FOR_RELEASE
    # Rising while still waiting for release is ignored until release.
    action = sm.on_inputs(_inputs(button_pressed=True, rising_edge=True))
    assert action == Action.NONE
    sm.on_inputs(_inputs(button_pressed=False))
    assert sm.state == FreedriveState.WAITING_FOR_ENABLE


def test_button_debounce_requires_stable_interval() -> None:
    clock = FakeClock()
    button = DebouncedButton(debounce_s=0.1, monotonic=clock.monotonic)
    stable, rising, falling = button.update(True)
    assert (stable, rising, falling) == (False, False, False)
    clock.now = 0.05
    stable, rising, falling = button.update(True)
    assert rising is False
    clock.now = 0.11
    stable, rising, falling = button.update(True)
    assert (stable, rising, falling) == (True, True, False)


def test_controller_accepts_only_rizon4s_identity() -> None:
    robot = FakeRobot()
    controller = FreedriveController(
        robot,
        primitive_mode="NRT_PRIMITIVE_EXECUTION",
        config=FreedriveConfiguration(),
        output=lambda *_: None,
        stdin=None,
    )
    controller._check_identity()

    original_info = robot.info
    robot.info = lambda: SimpleNamespace(
        model_name="Rizon4",
        software_ver="v3.11.0",
        license_type="RDK-Professional",
        DoF_m=7,
    )
    try:
        controller._check_identity()
        raise AssertionError("expected Rizon4 identity rejection")
    except Exception as exc:
        assert "Rizon4s" in str(exc)
    finally:
        robot.info = original_info


def test_controller_press_release_press_without_process_exit() -> None:
    clock = FakeClock()
    robot = FakeRobot()
    cfg = FreedriveConfiguration(
        sample_period_s=0.1,
        debounce_s=0.0,
    )
    controller = FreedriveController(
        robot,
        primitive_mode="NRT_PRIMITIVE_EXECUTION",
        config=cfg,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        output=lambda *_: None,
        stdin=None,
    )

    # Manually drive two sessions through the machine/controller helpers.
    robot.button = True
    controller.machine.on_inputs(
        StateMachineInputs(
            button_pressed=True,
            rising_edge=True,
            falling_edge=False,
            fault=False,
            connected=True,
            quit_requested=False,
            primitive_terminated=False,
        )
    )
    controller._start_primitive()
    assert sum(1 for c in robot.calls if isinstance(c, tuple) and c[0] == "ExecutePrimitive") == 1
    assert robot.calls[-1] == (
        "ExecutePrimitive",
        "FloatingCartesian",
        {"floatingAxis": [1, 1, 1, 1, 1, 1], "enableElbowMotion": 0},
    )

    controller.machine.on_inputs(
        StateMachineInputs(
            button_pressed=False,
            rising_edge=False,
            falling_edge=True,
            fault=False,
            connected=True,
            quit_requested=False,
            primitive_terminated=False,
        )
    )
    assert controller.machine.state == FreedriveState.STOPPING_FREEDRIVE
    controller._stop_session()
    controller.machine.on_inputs(
        StateMachineInputs(
            button_pressed=False,
            rising_edge=False,
            falling_edge=False,
            fault=False,
            connected=True,
            quit_requested=False,
            primitive_terminated=False,
            stop_complete=True,
        )
    )
    assert controller.machine.state == FreedriveState.WAITING_FOR_ENABLE

    controller.machine.on_inputs(
        StateMachineInputs(
            button_pressed=True,
            rising_edge=True,
            falling_edge=False,
            fault=False,
            connected=True,
            quit_requested=False,
            primitive_terminated=False,
        )
    )
    controller._start_primitive()
    assert sum(1 for c in robot.calls if isinstance(c, tuple) and c[0] == "ExecutePrimitive") == 2
    assert robot.calls.count("Stop") >= 1


def test_startup_enters_locked_state_without_home_motion() -> None:
    clock = FakeClock()
    robot = FakeRobot()
    cfg = FreedriveConfiguration()
    controller = FreedriveController(
        robot,
        primitive_mode="NRT_PRIMITIVE_EXECUTION",
        config=cfg,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        output=lambda *_: None,
        stdin=None,
    )

    controller._ensure_locked()

    primitive_calls = [
        call
        for call in robot.calls
        if isinstance(call, tuple)
        and call[0] == "ExecutePrimitive"
    ]
    assert primitive_calls == []
    assert robot.calls[-1] == "Stop"
    assert robot.stopped()


def test_zero_ft_sensor_runs_without_home_and_returns_to_locked_state() -> None:
    clock = FakeClock()
    robot = FakeRobot()
    controller = FreedriveController(
        robot,
        primitive_mode="NRT_PRIMITIVE_EXECUTION",
        config=FreedriveConfiguration(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        output=lambda *_: None,
        stdin=None,
    )

    controller._zero_ft_sensor()

    assert (
        "ExecutePrimitive",
        "ZeroFTSensor",
        {},
    ) in robot.calls
    assert not any(
        isinstance(call, tuple)
        and call[0] == "ExecutePrimitive"
        and call[1] == "Home"
        for call in robot.calls
    )
    assert robot.calls[-1] == "Stop"
    assert robot.stopped()
