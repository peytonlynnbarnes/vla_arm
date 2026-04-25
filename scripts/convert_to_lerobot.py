#!/usr/bin/env python3
"""Convert raw `demos_raw/episode_NNNN/` directories to LeRobot v2 format.

Reads the per-episode `frames.npz` + `meta.json` produced by `demo_recorder.py`
and writes a LeRobot-compatible parquet + video dataset that SmolVLA can load
via `LeRobotDataset.from_root(...)`.

Output layout (target_dir):
  {target_dir}/
    meta/
      episodes.jsonl             one line per episode
      info.json                  dataset-level metadata (features, counts, fps)
      stats.json                 per-feature min/max/mean/std stubs
      tasks.jsonl                task-index mapping
    data/
      chunk-000/
        episode_000000.parquet
        episode_000001.parquet
        ...
    videos/
      chunk-000/
        observation.images.third_person/
          episode_000000.mp4
          ...

Feature schema (matching SmolVLA's expectations for SO-100/SO-101):
  observation.images.third_person : video, (H, W, 3) uint8
  observation.state               : float32[6]  (joint positions)
  action                          : float32[6]  (commanded joint positions)
  task                            : string      (e.g. "pick the blue ball and place it on the green marker")
  timestamp                       : float32
  frame_index                     : int64
  episode_index                   : int64
  index                           : int64 (global)

Dependencies installed on demand:
  - pyarrow (pip): parquet serialization
  - ffmpeg (system) OR imageio-ffmpeg (pip): video encoding

Usage:
  python3 scripts/convert_to_lerobot.py \
    --input-dir /workspace/data/demos_raw \
    --output-dir /workspace/data/demos_lerobot \
    --fps 10
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


def _ffmpeg_exe() -> str:
    """Return the ffmpeg binary path: prefer system, fall back to imageio_ffmpeg."""
    sys_ff = shutil.which('ffmpeg')
    if sys_ff:
        return sys_ff
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        raise RuntimeError(
            'ffmpeg not found. Install via apt (`sudo apt install ffmpeg`) '
            'or pip (`pip install imageio-ffmpeg`).'
        )


def write_video(frames: np.ndarray, output_path: Path, fps: int):
    """Write (N, H, W, 3) uint8 frames to MP4 via ffmpeg."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    n, h, w, _ = frames.shape
    cmd = [
        _ffmpeg_exe(), '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-s', f'{w}x{h}', '-pix_fmt', 'rgb24',
        '-r', str(fps), '-i', '-',
        '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
        '-pix_fmt', 'yuv420p', str(output_path),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    proc.stdin.write(frames.tobytes())
    proc.stdin.close()
    err = proc.stderr.read().decode('utf-8', errors='ignore')
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f'ffmpeg failed: {err[-500:]}')


def write_parquet(records: List[Dict], output_path: Path):
    import pyarrow as pa
    import pyarrow.parquet as pq
    output_path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.Table.from_pylist(records)
    pq.write_table(table, output_path)


def target_instruction(target: str) -> str:
    color = target.replace('_ball', '')
    return f'pick the {color} ball and place it on the green marker'


def convert(
    input_dir: Path,
    output_dir: Path,
    fps: int,
    repo_id: str = 'so101_pickplace_sim',
    task_name: str = 'pickplace',
):
    ep_dirs = sorted([p for p in input_dir.glob('episode_*') if p.is_dir()])
    if not ep_dirs:
        print(f'No episodes found in {input_dir}', file=sys.stderr)
        return 1
    print(f'Found {len(ep_dirs)} episodes.')
    output_dir.mkdir(parents=True, exist_ok=True)

    meta_dir = output_dir / 'meta'
    meta_dir.mkdir(parents=True, exist_ok=True)
    data_dir = output_dir / 'data' / 'chunk-000'
    video_dir = output_dir / 'videos' / 'chunk-000' / 'observation.images.third_person'
    data_dir.mkdir(parents=True, exist_ok=True)
    video_dir.mkdir(parents=True, exist_ok=True)

    # Per-feature stats accumulators
    state_all: List[np.ndarray] = []
    action_all: List[np.ndarray] = []
    episodes_meta = []
    global_index = 0
    tasks: Dict[str, int] = {}

    for ep_i, ep_dir in enumerate(ep_dirs):
        meta = json.loads((ep_dir / 'meta.json').read_text())
        npz = np.load(ep_dir / 'frames.npz')
        images = npz['images']            # (N, H, W, 3) uint8
        states = npz['states'].astype(np.float32)   # (N, 6)
        actions = npz['actions'].astype(np.float32) # (N, 6)
        timestamps = npz['timestamps'].astype(np.float32) # (N,)
        n = images.shape[0]
        target = meta.get('target', 'blue_ball')
        instr = target_instruction(target)
        if instr not in tasks:
            tasks[instr] = len(tasks)
        task_idx = tasks[instr]

        # Write video
        video_path = video_dir / f'episode_{ep_i:06d}.mp4'
        write_video(images, video_path, fps)

        # Per-frame records
        records = []
        for fi in range(n):
            records.append({
                'observation.state': states[fi].tolist(),
                'action': actions[fi].tolist(),
                'timestamp': float(timestamps[fi]),
                'frame_index': int(fi),
                'episode_index': ep_i,
                'index': global_index,
                'task_index': task_idx,
            })
            global_index += 1
        parquet_path = data_dir / f'episode_{ep_i:06d}.parquet'
        write_parquet(records, parquet_path)

        state_all.append(states)
        action_all.append(actions)

        episodes_meta.append({
            'episode_index': ep_i,
            'tasks': [instr],
            'length': n,
        })
        print(f'  episode {ep_i:4d}: {n:4d} frames, target={target}, task="{instr}"')

    # Stats
    state_cat = np.concatenate(state_all)
    action_cat = np.concatenate(action_all)
    stats = {
        'observation.state': {
            'mean': state_cat.mean(axis=0).tolist(),
            'std':  state_cat.std(axis=0).tolist(),
            'min':  state_cat.min(axis=0).tolist(),
            'max':  state_cat.max(axis=0).tolist(),
            'count': int(state_cat.shape[0]),
        },
        'action': {
            'mean': action_cat.mean(axis=0).tolist(),
            'std':  action_cat.std(axis=0).tolist(),
            'min':  action_cat.min(axis=0).tolist(),
            'max':  action_cat.max(axis=0).tolist(),
            'count': int(action_cat.shape[0]),
        },
    }
    (meta_dir / 'stats.json').write_text(json.dumps(stats, indent=2))

    # info.json
    info = {
        'codebase_version': 'v2.0',
        'robot_type': 'so101',
        'total_episodes': len(ep_dirs),
        'total_frames': global_index,
        'total_tasks': len(tasks),
        'total_videos': len(ep_dirs),
        'total_chunks': 1,
        'chunks_size': 1000,
        'fps': fps,
        'splits': {'train': f'0:{len(ep_dirs)}'},
        'data_path': 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
        'video_path': 'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4',
        'features': {
            'observation.images.third_person': {
                'dtype': 'video',
                'shape': [224, 224, 3],
                'names': ['height', 'width', 'channels'],
                'info': {
                    'video.fps': float(fps),
                    'video.codec': 'h264',
                    'video.pix_fmt': 'yuv420p',
                    'video.is_depth_map': False,
                    'has_audio': False,
                },
            },
            'observation.state': {
                'dtype': 'float32',
                'shape': [6],
                'names': ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                          'wrist_flex', 'wrist_roll', 'gripper'],
            },
            'action': {
                'dtype': 'float32',
                'shape': [6],
                'names': ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                          'wrist_flex', 'wrist_roll', 'gripper'],
            },
            'timestamp': {'dtype': 'float32', 'shape': [1], 'names': None},
            'frame_index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'episode_index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'task_index': {'dtype': 'int64', 'shape': [1], 'names': None},
        },
    }
    (meta_dir / 'info.json').write_text(json.dumps(info, indent=2))

    # episodes.jsonl
    with open(meta_dir / 'episodes.jsonl', 'w') as f:
        for ep in episodes_meta:
            f.write(json.dumps(ep) + '\n')

    # tasks.jsonl
    with open(meta_dir / 'tasks.jsonl', 'w') as f:
        for task, idx in tasks.items():
            f.write(json.dumps({'task_index': idx, 'task': task}) + '\n')

    print(f'\nDataset written to {output_dir}')
    print(f'  Total episodes: {len(ep_dirs)}')
    print(f'  Total frames:   {global_index}')
    print(f'  Tasks:          {list(tasks.keys())}')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', default='/workspace/data/demos_raw')
    parser.add_argument('--output-dir', default='/workspace/data/demos_lerobot')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--repo-id', default='so101_pickplace_sim')
    args = parser.parse_args()
    return convert(
        input_dir=Path(args.input_dir),
        output_dir=Path(args.output_dir),
        fps=args.fps,
        repo_id=args.repo_id,
    )


if __name__ == '__main__':
    sys.exit(main())
