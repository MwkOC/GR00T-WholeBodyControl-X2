#!/usr/bin/env python3
"""Trackerless VR -> trained SONIC TELEOP encoder -> X2 token overlay (SIM only)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import zmq

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_x2_mujoco import MJCF_PATH, TeleopObsBuilder  # noqa: E402

from gear_sonic.utils.teleop.zmq.zmq_packed_message_decoder import unpack_message  # noqa: E402
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import pack_pose_message  # noqa: E402


def rotation(quat):
    return Rotation.from_quat(np.asarray(quat)[..., [1, 2, 3, 0]])


def valid_pose(pose):
    return (
        pose is not None
        and np.shape(pose) == (3, 7)
        and np.isfinite(pose).all()
        and np.all(np.linalg.norm(pose[:, 3:], axis=1) > 1e-6)
    )


class TeleopTargets:
    """Engage-time calibration to the measured X2 pose, without arm IK."""

    def __init__(self, model):
        self.fk = TeleopObsBuilder(model)
        self.operator = None

    def local_targets(self, q_mj):
        motion = {
            "root_trans_offset": np.zeros((1, 3)),
            "root_rot": np.array([[0, 0, 0, 1.0]]),
            "dof": np.asarray(q_mj)[None, :],
        }
        anchor, quat, pos, orn = self.fk._fk(motion, 0)
        basis = rotation(quat).inv()
        return basis.apply(pos - anchor), (basis * rotation(orn)).as_quat()[:, [3, 0, 1, 2]]

    def calibrate(self, pose, q_mj):
        if not valid_pose(pose):
            raise ValueError("calibration needs a valid head and both controllers")
        self.operator = np.asarray(pose).copy()
        self.positions, self.orientations = self.local_targets(q_mj)

    def targets(self, pose):
        if self.operator is None or not valid_pose(pose):
            raise ValueError("tracking missing or not calibrated")
        pos = self.positions + pose[:, :3] - self.operator[:, :3]
        delta = rotation(self.operator[:, 3:]).inv() * rotation(pose[:, 3:])
        orn = (rotation(self.orientations) * delta).as_quat()[:, [3, 0, 1, 2]]
        return pos, orn


def build_teleop_obs(plan, vr_position, vr_orientation, base_quat):
    """Training layout: lower-body pos120/vel120, VR xyz9/wxyz12, anchor6."""
    current = np.asarray(plan["joint_pos_mj"], dtype=np.float32).reshape(31)
    future = np.asarray(plan["joint_pos_mj_future"], dtype=np.float32)
    velocities = np.asarray(plan["joint_vel_mj_future"], dtype=np.float32)
    if future.ndim != 2 or future.shape[0] < 9 or future.shape[1] != 31 or velocities.shape != future.shape:
        raise ValueError("planner must provide at least nine future 31-joint frames and velocities")
    if not np.isclose(float(np.asarray(plan["future_dt_s"]).ravel()[0]), 0.1):
        raise ValueError("TELEOP encoder requires the trained 0.1s reference stride")
    positions = np.vstack((current, future[:9]))[:, :12]
    current_vel = np.asarray(plan.get("joint_vel_mj", np.zeros(31))).reshape(31)
    velocities = np.vstack((current_vel, velocities[:9]))[:, :12]
    ref = np.asarray(plan["root_quat_xyzw"]).reshape(4)
    anchor = (rotation(base_quat).inv() * Rotation.from_quat(ref)).as_matrix()[:, :2].ravel()
    obs = np.concatenate(
        (
            positions.ravel(),
            velocities.ravel(),
            np.asarray(vr_position).reshape(9),
            np.asarray(vr_orientation).reshape(12),
            anchor,
        )
    ).astype(np.float32)
    if obs.shape != (267,) or not np.isfinite(obs).all():
        raise ValueError("invalid TELEOP observation")
    return obs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tokenizer", required=True)
    ap.add_argument("--input-source", choices=("pico", "quest3"), default="pico")
    ap.add_argument("--seconds", type=float, default=0)
    args = ap.parse_args()
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(args.tokenizer, sess_options=options, providers=["CPUExecutionProvider"])
    if (
        session.get_inputs()[0].shape[-1] != 267
        or session.get_modelmeta().custom_metadata_map.get("graph_kind") != "native_teleop_tokenizer"
    ):
        ap.error("need the matched trained TELEOP tokenizer (267 -> 64), not SMPL")
    if args.input_source == "pico":
        from gear_sonic.utils.teleop.vr.pico_x2_reader import PicoX2Reader

        reader = PicoX2Reader(tracking="3point")
    else:
        from gear_sonic.utils.teleop.vr.quest3_reader import Quest3Reader

        reader = Quest3Reader()
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    targets = TeleopTargets(model)
    ctx = zmq.Context()

    def subscribe(port, topic):
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        sock.connect(f"tcp://127.0.0.1:{port}")
        return sock

    plans = subscribe(5656, "pose")
    debug = subscribe(5659, "x2_debug")
    tokens = ctx.socket(zmq.PUB)
    tokens.setsockopt(zmq.LINGER, 0)
    tokens.bind("tcp://127.0.0.1:5574")
    command = ctx.socket(zmq.PUB)
    command.setsockopt(zmq.LINGER, 0)
    command.connect("tcp://127.0.0.1:5663")

    def locomotion(fwd=0.0, side=0.0, yaw=0.0):
        command.send_multipart(
            [
                b"planner_cmd",
                json.dumps(
                    {
                        "intent": "locomotion",
                        "magnitude": "continuous",
                        "stick_fwd": fwd,
                        "stick_side": side,
                        "stick_yaw": yaw,
                    }
                ).encode(),
            ]
        )

    reader.start()
    print("[three-point] SIM ONLY; head + two controllers, no body trackers or IK.", flush=True)
    print("[three-point] A+B+X+Y: calibrate/engage or release; left stick click: TELEOP/planner.", flush=True)
    print(
        "[three-point] Hold left trigger to walk/turn with sticks. Tracking loss requires re-engage.", flush=True
    )
    plan = state = None
    plan_t = state_t = -float("inf")
    enabled = active = False
    chord_prev = click_prev = False
    started = next_tick = time.monotonic()
    frame = 0
    try:
        while args.seconds <= 0 or time.monotonic() - started < args.seconds:
            now = time.monotonic()
            for sock, topic in ((plans, "pose"), (debug, "x2_debug")):
                try:
                    fields = unpack_message(sock.recv(zmq.NOBLOCK), expected_topic=topic).fields
                except zmq.Again:
                    continue
                if topic == "pose":
                    if plan is not None:
                        ticks = int(np.asarray(fields["frame_index"]).ravel()[0]) - int(
                            np.asarray(plan["frame_index"]).ravel()[0]
                        )
                        if ticks > 0:
                            fields["joint_vel_mj"] = (
                                np.asarray(fields["joint_pos_mj"]) - np.asarray(plan["joint_pos_mj"])
                            ) * (50.0 / ticks)
                    plan, plan_t = fields, now
                else:
                    state, state_t = fields, now
            pose = reader.get_3pt_pose()
            fresh = (
                reader.get_last_message_age_s() < 0.2
                and valid_pose(pose)
                and now - plan_t < 0.2
                and now - state_t < 0.2
            )
            chord = all(reader.get_buttons())
            click = reader.get_stick_clicks()[0]
            if enabled and not fresh:
                enabled = active = False
                print("[three-point] tracking/planner/state lost: released to pose planner", flush=True)
            if chord and not chord_prev:
                enabled = not enabled and fresh
                active = enabled
                if enabled:
                    targets.calibrate(pose, np.asarray(state["body_q"]).reshape(31))
                print(
                    f"[three-point] {'TELEOP' if active else 'PLANNER / waiting for fresh tracking'}", flush=True
                )
            elif enabled and click and not click_prev:
                active = not active
                if active:
                    targets.calibrate(pose, np.asarray(state["body_q"]).reshape(31))
            chord_prev, click_prev = chord, click
            token = np.zeros(64, np.float32)
            if active:
                pos, orn = targets.targets(pose)
                obs = build_teleop_obs(plan, pos, orn, np.asarray(state["base_quat"]).reshape(4))
                token = session.run(None, {session.get_inputs()[0].name: obs[None]})[0].reshape(64)
                if not np.isfinite(token).all():
                    raise ValueError("nonfinite TELEOP token")
            tokens.send(
                pack_pose_message(
                    {
                        "motion_token": token,
                        "engaged": np.array([float(active)], np.float32),
                        "joint_pos_mj": np.zeros(31, np.float32),
                        "root_quat_xyzw": np.array([0, 0, 0, 1], np.float32),
                        "frame_index": np.array([frame], np.int64),
                    },
                    version=4,
                )
            )
            lt = reader.get_controller_inputs()[0]
            axes = reader.get_controller_axes()
            axes = [0.0 if abs(v) < 0.2 else float(np.clip(v, -1, 1)) for v in axes]
            locomotion(axes[1], -axes[0], -axes[2]) if enabled and fresh and lt > 0.6 else locomotion()
            frame += 1
            next_tick += 0.02
            time.sleep(max(0.0, min(0.02, next_tick - time.monotonic())))
            if next_tick < time.monotonic() - 0.1:
                next_tick = time.monotonic()
    finally:
        locomotion()
        reader.stop()
        for sock in (plans, debug, tokens, command):
            sock.close()
        ctx.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
