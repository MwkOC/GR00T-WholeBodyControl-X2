"""Training-observation parity, trackerless input, and launch routing."""

from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "gear_sonic/scripts"))
from eval_x2_mujoco import DEFAULT_DOF, MJCF_PATH, TeleopObsBuilder  # noqa: E402
from three_point_vr import TeleopTargets, build_teleop_obs, valid_pose  # noqa: E402


def reference_plan():
    frames = np.tile(DEFAULT_DOF, (10, 1))
    frames[:, 0] += np.arange(10) * 0.001
    velocities = np.diff(frames, axis=0) / 0.1
    return {
        "joint_pos_mj": frames[0],
        "joint_pos_mj_future": frames[1:],
        "joint_vel_mj_future": velocities,
        "future_dt_s": [0.1],
        "root_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }, frames


def test_live_observation_matches_training_reference_builder():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    targets = TeleopTargets(model)
    plan, frames = reference_plan()
    pos, orn = targets.local_targets(frames[0])
    actual = build_teleop_obs(plan, pos, orn, [1.0, 0.0, 0.0, 0.0])
    motion = {
        "dof": frames,
        "root_trans_offset": np.zeros((10, 3)),
        "root_rot": np.tile([0.0, 0.0, 0.0, 1.0], (10, 1)),
    }
    expected = TeleopObsBuilder(model).build({"clip": motion}, 0.0, np.array([1.0, 0.0, 0.0, 0.0]), 10.0)
    np.testing.assert_allclose(actual, expected, atol=1e-6)


def test_calibration_is_noop_then_all_three_points_can_drive_targets():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    targets = TeleopTargets(model)
    pose = np.array(
        [
            [0.3, 0.2, 1.0, 1.0, 0.0, 0.0, 0.0],
            [0.3, -0.2, 1.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.7, 1.0, 0.0, 0.0, 0.0],
        ]
    )
    targets.calibrate(pose, DEFAULT_DOF)
    pos, orn = targets.targets(pose)
    expected_pos, expected_orn = targets.local_targets(DEFAULT_DOF)
    np.testing.assert_allclose(pos, expected_pos)
    np.testing.assert_allclose(orn, expected_orn, atol=1e-12)
    for point in range(3):
        moved = pose.copy()
        moved[point, 2] += 0.05
        actual, _ = targets.targets(moved)
        np.testing.assert_allclose(actual[point, 2] - pos[point, 2], 0.05)
    assert not valid_pose(np.zeros((3, 7)))


def test_trackerless_pico_never_requests_body_tracking(monkeypatch):
    import gear_sonic.utils.teleop.vr.pico_x2_reader as module

    def forbidden():
        raise AssertionError("trackerless path requested body tracking")

    sdk = SimpleNamespace(
        init=lambda: None,
        get_time_stamp_ns=lambda: 1,
        is_body_data_available=forbidden,
        get_headset_pose=lambda: [0.0, 1.7, 0.0, 0.0, 0.0, 0.0, 1.0],
        get_left_controller_pose=lambda: [-0.3, 1.0, -0.3, 0.0, 0.0, 0.0, 1.0],
        get_right_controller_pose=lambda: [0.3, 1.0, -0.3, 0.0, 0.0, 0.0, 1.0],
    )
    for name in (
        "get_A_button",
        "get_B_button",
        "get_X_button",
        "get_Y_button",
        "get_left_trigger",
        "get_right_trigger",
        "get_left_grip",
        "get_right_grip",
        "get_left_axis_click",
        "get_right_axis_click",
    ):
        setattr(sdk, name, lambda: 0.0)
    sdk.get_left_axis = sdk.get_right_axis = lambda: (0.0, 0.0)
    monkeypatch.setattr(module, "xrt", sdk)
    reader = module.PicoX2Reader(tracking="3point")
    reader._latest = reader._poll_once()
    assert valid_pose(reader.get_3pt_pose())
    assert reader.get_body_pose_24x7() is None
    assert reader._poll_once() is None  # unchanged timestamp is not fresh tracking


def test_pico_and_quest_three_points_have_identical_axes_and_keep_head_yaw():
    from scipy.spatial.transform import Rotation

    from gear_sonic.utils.teleop.pico_3pt import _process_3pt_from_devices
    from gear_sonic.utils.teleop.vr.quest3_reader import compute_3pt_pose_from_quest3

    poses = [
        [0.1, 1.7, 0.2, *Rotation.from_euler("y", 0.6).as_quat()],
        [-0.3, 1.0, -0.3, *Rotation.from_euler("x", 0.3).as_quat()],
        [0.3, 1.0, -0.3, 0.0, 0.0, 0.0, 1.0],
    ]
    data = {
        key: {"position": pose[:3], "orientation": pose[3:]} for key, pose in zip(("head", "left", "right"), poses)
    }
    pico = _process_3pt_from_devices(*poses)
    quest = compute_3pt_pose_from_quest3(data)
    np.testing.assert_allclose(pico, quest, atol=1e-7)
    assert pico[0, 0] > 0 and pico[0, 1] > 0  # left hand forward and left
    assert abs(pico[2, 6]) > 0.1  # head yaw survives the device conversion


@pytest.mark.parametrize("mode,marker", [("--three-point-vr", "trained TELEOP"), ("--ik", "--ik")])
def test_modes_dry_run_without_services(mode, marker):
    result = subprocess.run(
        ["bash", "gear_sonic/scripts/sim_onnx_planner.sh", mode, "--dry-run"],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=True,
    )
    assert marker in result.stdout


def test_neural_mode_refuses_a_robot_target():
    result = subprocess.run(
        [
            "bash",
            "gear_sonic/scripts/sim_onnx_planner.sh",
            "--three-point-vr",
            "--pc2-host",
            "192.0.2.1",
            "--dry-run",
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0


def test_neuronal_and_ik_modes_cannot_be_combined():
    result = subprocess.run(
        ["bash", "gear_sonic/scripts/sim_onnx_planner.sh", "--ik", "--three-point-vr", "--dry-run"],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0


def test_cached_neural_set_uses_matching_tokens_and_decoder():
    root = Path.home() / ".cache/sonic/x2/three_point_vr"
    if not (root / "teleop.onnx").exists():
        pytest.skip("run --three-point-vr --prepare-models for neural inference parity")
    import onnxruntime as ort

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 2
    sessions = [
        ort.InferenceSession(str(root / name), sess_options=opts, providers=["CPUExecutionProvider"])
        for name in ("pose.onnx", "token.onnx", "teleop.onnx")
    ]
    fingerprints = {s.get_modelmeta().custom_metadata_map["codec_fingerprint"] for s in sessions}
    assert len(fingerprints) == 1
    obs = np.random.default_rng(3).normal(0.0, 0.2, (1, 267)).astype(np.float32)
    tokens = sessions[2].run(None, {"obs": obs})[0]
    decoder_obs = np.zeros((1, 1670), np.float32)
    decoder_obs[:, :64] = tokens
    action = sessions[1].run(None, {"obs": decoder_obs})[0]
    assert tokens.shape == (1, 64) and action.shape == (1, 31)
    assert np.isfinite(action).all()
    changed_head = obs.copy()
    changed_head[:, 246:249] += 0.25
    changed_tokens = sessions[2].run(None, {"obs": changed_head})[0]
    assert not np.array_equal(tokens, changed_tokens)


def test_live_neural_stream_releases_on_dropout_and_requires_reengage(monkeypatch):
    """Real ZMQ + real ONNX; inject only the headset and robot telemetry."""
    import threading
    import time

    import three_point_vr as service
    import zmq

    import gear_sonic.utils.teleop.vr.quest3_reader as quest_module

    root = Path.home() / ".cache/sonic/x2/three_point_vr"
    if not (root / "teleop.onnx").exists():
        pytest.skip("prepare TELEOP models first")
    real_context = zmq.Context
    bus = real_context()
    pubs = [bus.socket(zmq.PUB), bus.socket(zmq.PUB)]
    # Bind the token SUB here; the service's PUB connects to it. All ports
    # are ephemeral, so this check cannot interfere with a live sim stack.
    received = bus.socket(zmq.SUB)
    received.setsockopt_string(zmq.SUBSCRIBE, "pose")
    command_sink = bus.socket(zmq.SUB)
    sockets = [*pubs, received, command_sink]
    endpoints = []
    for sock in sockets:
        sock.setsockopt(zmq.LINGER, 0)
        sock.bind("tcp://127.0.0.1:*")
        endpoints.append(sock.getsockopt_string(zmq.LAST_ENDPOINT))
    endpoint_map = dict(zip((5656, 5659, 5574, 5663), endpoints))

    class Socket:
        def __init__(self, sock):
            self.sock = sock

        def __getattr__(self, name):
            return getattr(self.sock, name)

        def connect(self, endpoint):
            self.sock.connect(endpoint_map[int(endpoint.rsplit(":", 1)[1])])

        def bind(self, endpoint):
            self.connect(endpoint)

    class Context:
        def __init__(self):
            self.ctx = real_context()

        def socket(self, kind):
            return Socket(self.ctx.socket(kind))

        def term(self):
            self.ctx.term()

    started = [None]

    class Headset:
        def start(self):
            started[0] = time.monotonic()

        def stop(self):
            pass

        def elapsed(self):
            return time.monotonic() - started[0]

        def get_3pt_pose(self):
            return np.array(
                [
                    [0.3, 0.2, 1.0, 1.0, 0.0, 0.0, 0.0],
                    [0.3, -0.2, 1.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.7, 1.0, 0.0, 0.0, 0.0],
                ]
            )

        def get_last_message_age_s(self):
            return 1.0 if 0.9 < self.elapsed() < 1.2 else 0.0

        def get_buttons(self):
            return (0.5 < self.elapsed() < 0.7,) * 4

        def get_stick_clicks(self):
            return False, False

        def get_controller_inputs(self):
            return 0.0, 0.0, 0.0, 0.0

        def get_controller_axes(self):
            return 0.0, 0.0, 0.0, 0.0

    stop = threading.Event()
    flags = []

    def robot():
        frame = 0
        while not stop.is_set():
            plan, _ = reference_plan()
            plan["frame_index"] = np.array([frame], np.int64)
            pubs[0].send(
                service.pack_pose_message({k: np.asarray(v, dtype=np.float32) for k, v in plan.items()}, version=4)
            )
            pubs[1].send(
                service.pack_pose_message(
                    {
                        "body_q": DEFAULT_DOF.astype(np.float32),
                        "base_quat": np.array([1.0, 0.0, 0.0, 0.0], np.float32),
                    },
                    topic="x2_debug",
                    version=4,
                )
            )
            try:
                fields = service.unpack_message(received.recv(zmq.NOBLOCK), expected_topic="pose").fields
                flags.append((time.monotonic() - started[0], float(fields["engaged"][0])))
            except zmq.Again:
                pass
            frame += 1
            time.sleep(0.01)

    monkeypatch.setattr(service.zmq, "Context", Context)
    monkeypatch.setattr(quest_module, "Quest3Reader", Headset)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "three_point_vr.py",
            "--input-source",
            "quest3",
            "--tokenizer",
            str(root / "teleop.onnx"),
            "--seconds",
            "1.6",
        ],
    )
    worker = threading.Thread(target=robot)
    worker.start()
    try:
        assert service.main() == 0
    finally:
        stop.set()
        worker.join()
        for sock in sockets:
            sock.close()
        bus.term()
    assert any(flag == 1.0 for _, flag in flags)
    assert all(flag == 0.0 for elapsed, flag in flags if elapsed > 1.3)
