from pathlib import Path
import subprocess

from scripts.diagnose_wrist_moteus_config import (
    analyze_position_config,
    parse_config_dump,
)


PROJECT_ROOT = Path(__file__).parents[1]


def test_targeted_reader_declares_controller_can_prefix() -> None:
    completed = subprocess.run(
        [
            str(PROJECT_ROOT / ".venv_moteus" / "bin" / "python"),
            str(PROJECT_ROOT / "scripts" / "read_moteus_position_config.py"),
            "--help",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0
    assert "--can-prefix" in completed.stdout


def test_parse_and_accept_expected_reducer_configuration() -> None:
    parsed = parse_config_dump(
        """
Target: DeviceAddress(can_id=2)
motor_position.output.source 0
motor_position.output.sign 1
motor_position.rotor_to_output_ratio 0.0277777778
motor_position.sources.0.reference 0
motor_position.sources.0.sign 1
servo.pid_position.kp 2.0
servo.pid_position.kd 0.02
servo.fixed_voltage_mode 0
"""
    )
    findings = analyze_position_config(parsed, expected_reduction=36.0)
    assert not any(item.startswith("ERROR:") for item in findings)


def test_report_warns_ratio_and_zero_damping_but_flags_wrong_sign() -> None:
    parsed = parse_config_dump(
        """
motor_position.output.source 1
motor_position.output.sign -1
motor_position.rotor_to_output_ratio 1
motor_position.sources.1.reference 1
motor_position.sources.1.sign 1
servo.pid_position.kp 1
servo.pid_position.kd 0
"""
    )
    findings = analyze_position_config(parsed, expected_reduction=30.0)
    errors = "\n".join(item for item in findings if item.startswith("ERROR:"))
    assert "output.sign" in errors
    assert any("rotor_to_output_ratio" in item for item in findings if item.startswith("WARN:"))
    assert any("kd is zero" in item for item in findings if item.startswith("WARN:"))
