#!/usr/bin/env python3
"""Regenerate scene-labelled videos from measurements; never rerun planning."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

from classroom_voxel_scene import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?', type=Path, default=ROOT/'maps/indoor_async_v2_suite_160s')
    parser.add_argument('--jobs', type=int, default=3)
    args = parser.parse_args()
    output = args.folder.resolve()
    summary = json.loads((output/'summary.json').read_text())
    if summary['finished'] != summary['planned']:
        parser.error('Wait for all planning runs to finish before changing visualization')
    sources = sorted((ROOT/'scripts').glob('*.py'))
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with tarfile.open(output/'render_source_snapshot.tar.gz', 'w:gz') as archive:
        for p in sources:
            archive.add(p, arcname=str(p.relative_to(ROOT)))

    def render(case):
        folder = output/case['id']
        before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in [folder/'events.json', folder/'observed_points.bin', folder/'final_voxels.npz']}
        command = [sys.executable, str(ROOT/'scripts/demo_voxel360_async_v2.py'),
                   '--render-only', '--output-dir', str(folder)]
        with (folder/'render.log').open('w') as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        for name, digest in before.items():
            assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == digest
        with (folder/'render_validation.log').open('w') as log:
            subprocess.run([sys.executable, str(ROOT/'scripts/verify_voxel360_async.py'), str(folder)],
                           cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        (folder/'render_manifest.json').write_text(json.dumps(dict(
            command=command, source_sha256=hashes, source_archive='../render_source_snapshot.tar.gz',
            scene_name=case['layout'], original_data_sha256=before, original_data_unchanged=True,
            note='Display-only regeneration from archived frames and point timeline; no new plans or scans.'), indent=2)+'\n')
        print('RENDERED '+case['id'], flush=True)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        list(pool.map(render, [c for c in summary['cases'] if c['status'] == 'verified']))


if __name__ == '__main__':
    main()
