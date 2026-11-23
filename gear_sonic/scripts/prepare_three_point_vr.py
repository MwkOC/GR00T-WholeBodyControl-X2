#!/usr/bin/env python3
"""Export a matched X2 SONIC pose/token/TELEOP set from a trained checkpoint."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_x2_mujoco import load_actor_from_checkpoint
from native_token_onnx_export import NativeSmplTokenizer, NativeTokenGraph, _export


class PoseGraph(torch.nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs):
        return self.actor(obs[:, 680:], obs[:, :680])


def prepare(out_dir: Path, checkpoint: str | None = None) -> None:
    if checkpoint is None:
        from huggingface_hub import hf_hub_download

        checkpoint = hf_hub_download(
            "tinkerbuggy/sonic-x2",
            "sonic_torch/model_step_014000.pt",
            revision="f576dd02ba447385c2455079a1c88f9d4b050e0d",
        )
    digest = hashlib.md5(Path(checkpoint).read_bytes()).hexdigest()
    metadata = {
        "codec_fingerprint": f"native-3enc:{digest}",
        "source_checkpoint": Path(checkpoint).name,
        "encoder_name": "teleop",
    }
    pose = load_actor_from_checkpoint(checkpoint, encoder="g1")
    teleop = load_actor_from_checkpoint(checkpoint, encoder="teleop")
    for key, value in pose.decoder.state_dict().items():
        torch.testing.assert_close(value, teleop.decoder.state_dict()[key], rtol=0, atol=0)
    out_dir.mkdir(parents=True, exist_ok=True)
    # NativeSmplTokenizer is encoder-agnostic: the same trained FSQ for
    # TELEOP's 267 inputs, not a reconstruction of an SMPL skeleton.
    modules = [
        ("pose.onnx", PoseGraph(pose), 1670, "native_pose"),
        ("token.onnx", NativeTokenGraph(teleop), 1670, "any2any_token_deploy"),
        ("teleop.onnx", NativeSmplTokenizer(teleop), 267, "native_teleop_tokenizer"),
    ]
    import onnxruntime as ort

    rng = np.random.default_rng(7)
    torch.set_num_threads(2)
    for name, module, width, kind in modules:
        module.eval()
        sample = torch.from_numpy(rng.normal(0, 0.2, (1, width)).astype(np.float32))
        target = out_dir / name
        tmp = str(target) + ".tmp"
        _export(module, sample, tmp, "obs", "output", 17, {**metadata, "graph_kind": kind})
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        session = ort.InferenceSession(tmp, sess_options=options, providers=["CPUExecutionProvider"])
        for _ in range(8):
            sample = torch.from_numpy(rng.normal(0, 0.2, (1, width)).astype(np.float32))
            with torch.no_grad():
                expected = module(sample).numpy()
            actual = session.run(None, {"obs": sample.numpy()})[0]
            np.testing.assert_allclose(actual, expected, atol=1e-4, rtol=1e-4)
        os.replace(tmp, target)
        print(f"[three-point] parity OK: {target}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", help="native X2 checkpoint with its trained TELEOP encoder")
    parser.add_argument("--out-dir", type=Path, default=Path.home() / ".cache/sonic/x2/three_point_vr")
    args = parser.parse_args()
    prepare(args.out_dir, args.checkpoint)
