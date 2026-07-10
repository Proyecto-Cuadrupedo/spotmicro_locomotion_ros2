import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from spotmicro_locomotion_ros2.spotmicro_champ_bridge import (
    clamp_force,
    clamp_value,
    parameter_list,
)


def test_clamp_value_limits_range():
    assert clamp_value(3.2, -1.0, 1.0) == 1.0
    assert clamp_value(-3.2, -1.0, 1.0) == -1.0
    assert clamp_value(0.2, -1.0, 1.0) == 0.2


def test_clamp_force_is_symmetric():
    assert clamp_force(0.8, 0.3) == 0.3
    assert clamp_force(-0.8, 0.3) == -0.3


def test_parameter_list_expands_single_value():
    assert parameter_list([2.0], 3, "gain") == [2.0, 2.0, 2.0]