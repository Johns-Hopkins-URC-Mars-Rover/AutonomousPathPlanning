import math

import pytest

from rover_simulation.magnetic_field_simulator import world_to_body


def test_identity_keeps_world_field():
    assert world_to_body((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)) == \
        pytest.approx((1.0, 2.0, 3.0))


def test_positive_ninety_degree_yaw_rotates_world_north_to_body_x():
    half = math.sqrt(0.5)
    assert world_to_body((0.0, 0.0, half, half), (0.0, 1.0, 0.0)) == \
        pytest.approx((1.0, 0.0, 0.0), abs=1.0e-12)


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        world_to_body((0.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
