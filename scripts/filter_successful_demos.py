#!/usr/bin/env python3
"""Filter the per-batch demo directories produced by `collect_200_demos.sh`,
keeping only episodes whose expert-policy run reported SUCCESS.

Parses /tmp/collect_progress.log for the ordering of successes/fails. The
recorder writes episodes sequentially inside each batch directory, so the N-th
SUCCESS/FAIL line in the log corresponds to `episode_NNNN` in the N-th batch.

Output: writes a flat `data/demos_successful/episode_NNNNNN/` tree pointing at
successful episodes (via hard links so we don't duplicate GBs of images).
Usage:
    python3 scripts/filter_successful_demos.py \\
        --progress-log /tmp/collect_progress.log \\
        --batches-root /workspace/data/demos_raw_final \\
        --output /workspace/data/demos_successful
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path


def parse_progress(log_path: Path):
    """Parse the progress log into a flat list of (batch_idx, episode_in_batch, success).

    The expert policy starts at episode 1 per batch (via --seed N), and each
    batch is invoked with the same --trials value. Every '=== episode X/Y ===' line
    begins a new attempt, followed eventually by either SUCCESS or FAIL for that
    attempt. Between batches the progress log gets appended to; batch boundary is
    wherever episode_in_batch resets to 1.
    """
    results = []
    batch_idx = -1
    current_ep = None
    last_ep_in_batch = 0
    for line in log_path.read_text().splitlines():
        m = re.search(r'=== episode (\d+)/\d+ ===', line)
        if m:
            ep_in_batch = int(m.group(1))
            if ep_in_batch <= last_ep_in_batch:
                batch_idx += 1
            elif batch_idx < 0:
                batch_idx = 0
            last_ep_in_batch = ep_in_batch
            current_ep = (batch_idx, ep_in_batch - 1)  # 0-indexed in batch
            continue
        if 'SUCCESS' in line:
            if current_ep is not None:
                results.append((*current_ep, True))
                current_ep = None
        elif 'FAIL' in line:
            if current_ep is not None:
                results.append((*current_ep, False))
                current_ep = None
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--progress-log', type=Path, default=Path('/tmp/collect_progress.log'))
    parser.add_argument('--batches-root', type=Path, default=Path('/workspace/data/demos_raw_final'))
    parser.add_argument('--output', type=Path, default=Path('/workspace/data/demos_successful'))
    parser.add_argument('--limit', type=int, default=200)
    args = parser.parse_args()

    if not args.progress_log.exists():
        print(f'no progress log at {args.progress_log}', file=sys.stderr)
        return 1
    if not args.batches_root.exists():
        print(f'no batches at {args.batches_root}', file=sys.stderr)
        return 1

    args.output.mkdir(parents=True, exist_ok=True)
    results = parse_progress(args.progress_log)
    print(f'parsed {len(results)} episodes from progress log; '
          f'{sum(1 for r in results if r[2])} marked SUCCESS')

    kept = 0
    for batch_idx, ep_in_batch, success in results:
        if kept >= args.limit:
            break
        if not success:
            continue
        src = args.batches_root / f'batch_{batch_idx:03d}' / f'episode_{ep_in_batch:04d}'
        if not src.exists():
            print(f'  miss: {src}', file=sys.stderr)
            continue
        dst = args.output / f'episode_{kept:06d}'
        if dst.exists():
            shutil.rmtree(dst)
        dst.mkdir(parents=True, exist_ok=True)
        # Hard-link everything — fast, no duplicate space
        for item in src.iterdir():
            try:
                os.link(item, dst / item.name)
            except OSError:
                shutil.copy2(item, dst / item.name)
        kept += 1

    print(f'kept {kept} successful episodes in {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
