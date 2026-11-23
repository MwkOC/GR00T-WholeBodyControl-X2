"""Check the X2 adapter against HandUMI's residual-Jacobian DLS equation."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.incremental_chain import ARM_NEUTRAL, make_arm_solver
from gear_sonic.utils.teleop.incremental_chain_v3 import ClutchArmV3
from gear_sonic.utils.teleop.solver.arm.x2_arm_ik import HandumiArmIK, matrix_to_quat_wxyz


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("angle", [0.4, 2.5])
def test_step_matches_handumi_finite_difference_residual(side, angle):
    q = ARM_NEUTRAL[side].copy()
    solver = HandumiArmIK(side, q_preferred=q)
    pos, rot = solver.fk(q)
    pos = pos + [0.04, -0.01, 0.03]
    target_rot = Rotation.from_rotvec([angle, 0.0, 0.0]).as_matrix() @ rot

    def residual(joints):
        xyz, matrix = solver.fk(joints)
        return np.r_[pos - xyz, Rotation.from_matrix(target_rot @ matrix.T).as_rotvec()]

    error = residual(q)
    epsilon = 1e-6
    J = np.column_stack(
        [
            -(residual(q + np.eye(7)[i] * epsilon) - residual(q - np.eye(7)[i] * epsilon)) / (2 * epsilon)
            for i in range(7)
        ]
    )
    weight = np.r_[np.ones(3), np.full(3, 0.1 if np.linalg.norm(error[3:]) <= 2.2 else 0.0)]
    J = weight[:, None] * J
    error = weight * error
    ramp = max(0.0, 1.0 - np.min(np.linalg.svd(J, compute_uv=False)) / 0.05)
    normal = J.T @ J + (0.05**2 + 0.15**2 * ramp**2 + 0.02**2) * np.eye(7)
    delta = np.linalg.solve(normal, J.T @ error)
    candidate = np.clip(q + delta, solver._lo, solver._hi)
    expected = q + np.clip(candidate - q, -0.02, 0.02)
    actual, info = solver.solve(q, pos, matrix_to_quat_wxyz(target_rot), max_iters=20)
    np.testing.assert_allclose(actual, expected, atol=1e-9)
    assert info.iterations == 1
    assert np.max(np.abs(actual - q)) <= 0.02 + 1e-12


@pytest.mark.parametrize("side", ["left", "right"])
def test_repeated_steps_close_error_and_stay_in_joint_limits(side):
    q = ARM_NEUTRAL[side].copy()
    solver, backend = make_arm_solver(side, backend="handumi", q_preferred=q)
    assert backend == "handumi"
    pos, rot = solver.fk(q)
    target = pos + [0.02, 0.0, 0.0]
    for _ in range(100):
        previous = q.copy()
        q, _ = solver.solve(q, target, matrix_to_quat_wxyz(rot))
        assert np.max(np.abs(q - previous)) <= 0.02 + 1e-12
        assert np.all(q >= solver._lo) and np.all(q <= solver._hi)
    assert np.linalg.norm(solver.fk(q)[0] - target) < 0.005


def test_clutch_release_freezes_and_other_arm_does_not_move():
    left = ClutchArmV3("left", backend="handumi", full_pose_ik=True)
    right = ClutchArmV3("right", backend="handumi", full_pose_ik=True)
    pose = np.array([0.2, 0.3, 1.0, 1.0, 0.0, 0.0, 0.0])
    assert left.engage(pose)
    moved = pose.copy()
    moved[0] += 0.04
    original_right = right.q.copy()
    for _ in range(15):
        left.step(True, moved)
    frozen = left.q.copy()
    q, _ = left.step(False, moved + [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_array_equal(q, frozen)
    np.testing.assert_array_equal(right.q, original_right)


def test_invalid_target_rejected():
    solver = HandumiArmIK("left")
    with pytest.raises(ValueError, match="finite"):
        solver.solve(ARM_NEUTRAL["left"], [np.nan, 0, 0])
