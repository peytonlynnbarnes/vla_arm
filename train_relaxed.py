#!/usr/bin/env python3
"""Relax LeRobot's tolerances / off-by-ones for our ROS-recorded demos:
 - Disable timestamp-sync check.
 - Bump LeRobotDataset.tolerance_s for loose video/parquet timing.
 - Clamp out-of-range timestamps in decode_video_frames_torchcodec so our
   few (n+1)-row parquet / n-frame mp4 episodes don't crash the loader.
Then delegate to lerobot.scripts.train."""
import runpy


def _nop_check(*args, **kwargs):
    return True


# 1. Kill the dataset-level sync check.
for modpath in (
    "lerobot.datasets.utils",
    "lerobot.datasets.lerobot_dataset",
    "lerobot.common.datasets.utils",
    "lerobot.common.datasets.lerobot_dataset",
):
    try:
        mod = __import__(modpath, fromlist=["check_timestamps_sync"])
    except ImportError:
        continue
    if hasattr(mod, "check_timestamps_sync"):
        mod.check_timestamps_sync = _nop_check

# 2. Bump tolerance_s on the dataset after init.
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

_orig_init = LeRobotDataset.__init__


def _patched_init(self, *args, **kwargs):
    _orig_init(self, *args, **kwargs)
    self.tolerance_s = 0.1


LeRobotDataset.__init__ = _patched_init

# 3. Clamp timestamps at the torchcodec layer (called via module-global lookup
#    from decode_video_frames, so patching the module attribute works).
try:
    from lerobot.datasets import video_utils as _vu
except ImportError:
    from lerobot.common.datasets import video_utils as _vu

_orig_tc = _vu.decode_video_frames_torchcodec


def _safe_tc(video_path, timestamps, tolerance_s, *args, **kwargs):
    try:
        from torchcodec.decoders import VideoDecoder
        import torch
        decoder = VideoDecoder(str(video_path))
        n_frames = decoder.metadata.num_frames
        fps = decoder.metadata.average_fps or 10.0
        max_t = max(0.0, (n_frames - 1) / fps)
        if isinstance(timestamps, torch.Tensor):
            timestamps = timestamps.clamp(max=max_t)
        else:
            timestamps = [min(float(t), max_t) for t in timestamps]
    except Exception:
        pass
    return _orig_tc(video_path, timestamps, tolerance_s, *args, **kwargs)


_vu.decode_video_frames_torchcodec = _safe_tc

runpy.run_module("lerobot.scripts.train", run_name="__main__")
