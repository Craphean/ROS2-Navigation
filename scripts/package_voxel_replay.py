#!/usr/bin/env python3
"""Package replay inputs alongside published MP4/PNG results without duplicating clouds."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def package(folder):
    folder = Path(folder)
    names = ('observed_points.bin', 'final_voxels.npz', 'source_snapshot.tar.gz',
             'render_source_snapshot.tar.gz', 'run.log', 'source_run.log')
    files = sorted(p for p in folder.rglob('*') if p.is_file()
                   and (p.name in names or (p.suffix == '.npz' and p.parent.name == 'frames')))
    if not files:
        raise ValueError(f'No replay inputs in {folder}')
    target = folder/'replay_data.zip'
    temporary = folder/'replay_data.tmp'
    manifest = {}
    try:
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in files:
                relative = path.relative_to(folder).as_posix()
                archive.write(path, relative)
                manifest[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
        with zipfile.ZipFile(temporary) as archive:
            if archive.testzip() is not None:
                raise ValueError('Replay archive CRC verification failed')
            for name, digest in manifest.items():
                if hashlib.sha256(archive.read(name)).hexdigest() != digest:
                    raise ValueError(f'Replay archive checksum mismatch: {name}')
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    (folder/'replay_data_manifest.json').write_text(json.dumps(dict(
        archive='replay_data.zip', sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        files=manifest), indent=2)+'\n')
    return dict(folder=str(folder), files=len(files), archive_bytes=target.stat().st_size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', type=Path, nargs='+')
    args = parser.parse_args()
    for folder in args.folders:
        print(json.dumps(package(folder)), flush=True)
