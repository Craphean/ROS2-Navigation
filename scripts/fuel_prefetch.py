#!/usr/bin/env python3
"""Prefetch Gazebo Fuel models with parallel range requests.

`gz fuel download` is unusable on this box: single-connection throughput to
fuel.gazebosim.org sits around 14 KB/s, so Depot (82 MB) would take well over
an hour and the CLI usually aborts long before that.  Splitting the same file
into N range requests gives ~4x per extra connection here (14 KB/s single ->
52 KB/s with four), which turns "impossible" into "a few minutes".

The result is written into the standard gz fuel cache so worlds load offline:

    <cache>/fuel.gazebosim.org/<owner>/models/<name>/<version>/

Usage:
    fuel_prefetch.py --cache /root/.gz/fuel <fuel model url> [...]
    fuel_prefetch.py --cache /root/.gz/fuel --list models.txt
"""

from __future__ import annotations

import argparse
import concurrent.futures
import io
import json
import os
import shutil
import sys
import tempfile
import urllib.parse
import urllib.request
import zipfile

CHUNK = 2 * 1024 * 1024
WORKERS = 12
RETRIES = 4
TIMEOUT = 120


def _get(url: str, start: int | None = None, end: int | None = None) -> bytes:
    request = urllib.request.Request(url)
    if start is not None:
        request.add_header("Range", f"bytes={start}-{end}")
    request.add_header("User-Agent", "fuel-prefetch/1")
    with urllib.request.urlopen(request, timeout=TIMEOUT) as response:
        return response.read()


def metadata(base: str) -> dict:
    return json.loads(_get(base).decode("utf-8"))


def fetch(url: str, owner: str, name: str, version: str, size: int) -> bytes:
    """Download `url` in parallel ranges and return the whole body."""
    parts: dict[int, bytes] = {}
    ranges = []
    for start in range(0, size, CHUNK):
        ranges.append((start, min(size - 1, start + CHUNK - 1)))

    def one(index: int, start: int, end: int) -> None:
        last = None
        for attempt in range(RETRIES):
            try:
                parts[index] = _get(url, start, end)
                if len(parts[index]) == end - start + 1:
                    return
                last = f"short read {len(parts[index])}/{end - start + 1}"
            except Exception as exc:  # noqa: BLE001 - retried below
                last = repr(exc)
        raise RuntimeError(f"{name} bytes {start}-{end}: {last}")

    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        futures = [
            pool.submit(one, index, start, end) for index, (start, end) in enumerate(ranges)
        ]
        for future in concurrent.futures.as_completed(futures):
            future.result()
            done += 1
            if done % 5 == 0 or done == len(ranges):
                pct = 100.0 * done / len(ranges)
                print(f"  {name}: {pct:5.1f}% ({done}/{len(ranges)} chunks)", flush=True)
    return b"".join(parts[i] for i in range(len(ranges)))


def install(blob: bytes, dest: str, name: str) -> None:
    """Unpack a Fuel model zip into `dest`, dropping its single top folder."""
    os.makedirs(dest, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        names = [n for n in archive.namelist() if n and not n.endswith("/")]
        tops = {n.split("/", 1)[0] for n in names}
        strip = ""
        if len(tops) == 1:
            top = tops.pop()
            if all("/" in n for n in names) and top.lower() == name.lower():
                strip = top + "/"
        archive.extractall(dest)
        if strip:
            moved = os.path.join(dest, strip.rstrip("/"))
            for entry in os.listdir(moved):
                shutil.move(os.path.join(moved, entry), os.path.join(dest, entry))
            shutil.rmtree(moved, ignore_errors=True)


def prefetch(cache: str, url: str) -> bool:
    parsed = urllib.parse.urlparse(url)
    bits = [b for b in parsed.path.split("/") if b]
    # /1.0/<owner>/models/<name>
    if len(bits) < 4 or bits[2] != "models":
        print(f"!! not a model url: {url}", flush=True)
        return False
    owner, name = bits[1], urllib.parse.unquote(bits[3])
    # Model names may contain spaces ("Casual female"): http.client refuses a
    # raw space in the request path, so encode before asking for metadata.
    base = urllib.parse.quote(url, safe=":/")
    meta = metadata(base)
    version = str(meta.get("version") or "1")
    size = int(meta.get("filesize") or 0)
    dest = os.path.join(
        cache, parsed.netloc, owner.lower(), "models", name.lower(), version
    )
    if os.path.isfile(os.path.join(dest, "model.config")):
        print(f"== {owner}/{name} v{version}: already cached", flush=True)
        return True
    if size <= 0:
        print(f"!! {owner}/{name}: server reports no size", flush=True)
        return False
    print(f"== {owner}/{name} v{version}: {size / 1e6:.1f} MB", flush=True)
    file_url = f"{base.rstrip('/')}/{version}/{urllib.parse.quote(name)}.zip"
    blob = fetch(file_url, owner, name, version, size)
    if len(blob) != size:
        print(f"!! {owner}/{name}: got {len(blob)} of {size} bytes", flush=True)
        return False
    install(blob, dest, name)
    ok = os.path.isfile(os.path.join(dest, "model.config"))
    print(f"{'ok' if ok else '!!'} {owner}/{name} -> {dest}", flush=True)
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", default=os.path.expanduser("~/.gz/fuel"))
    parser.add_argument("--list", help="file with one model url per line")
    parser.add_argument("urls", nargs="*")
    args = parser.parse_args()

    urls = list(args.urls)
    if args.list:
        with open(args.list, encoding="utf-8") as handle:
            urls += [line.strip() for line in handle if line.strip() and not line.startswith("#")]
    if not urls:
        parser.error("no urls")

    failed = [u for u in urls if not prefetch(args.cache, u)]
    if failed:
        print(f"\nfailed ({len(failed)}):", flush=True)
        for url in failed:
            print("  " + url, flush=True)
        return 1
    print("\nall models cached", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
