"""Optional C++ acceleration. Same DDA/grid semantics as the NumPy fallback."""
import ctypes
import functools
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import warnings

import numpy as np


@functools.lru_cache(maxsize=1)
def library():
    if os.environ.get("VOXEL_NATIVE") == "0":
        return None
    source = Path(__file__).with_name("voxel_kernels.cpp")
    compiler = shutil.which("g++")
    if not compiler:
        return None
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    cache = source.parent / "__pycache__"
    cache.mkdir(exist_ok=True)
    output = cache / f"voxel_kernels_{digest}.so"
    try:
        if not output.exists():
            with tempfile.NamedTemporaryFile(prefix=f"voxel_{digest}_", suffix=".so",
                                             dir=cache, delete=False) as file:
                temporary = Path(file.name)
            try:
                subprocess.run([compiler, "-std=c++17", "-O3", "-shared", "-fPIC",
                                str(source), "-o", str(temporary)], check=True, capture_output=True)
                temporary.replace(output)
            finally:
                temporary.unlink(missing_ok=True)
        dll = ctypes.CDLL(str(output))
        array = np.ctypeslib.ndpointer
        state = array(dtype=np.int8, flags="C_CONTIGUOUS")
        ids = array(dtype=np.int64, flags="C_CONTIGUOUS")
        real = array(dtype=np.float64, flags="C_CONTIGUOUS")
        byte = array(dtype=np.uint8, flags="C_CONTIGUOUS")
        i, d, n = ctypes.c_int, ctypes.c_double, ctypes.c_int64
        dll.voxel_visible.argtypes = [state, ids, real, d, real, ids, n, d, d, d, d, i, ids]
        dll.voxel_visible.restype = n
        dll.voxel_integrate.argtypes = [state, ids, real, d, real, real, byte, n]
        dll.voxel_integrate.restype = None
        dll.voxel_raycast.argtypes = [state, ids, real, d, real, real, n, ids]
        dll.voxel_raycast.restype = None
        dll.voxel_first_unknown.argtypes = [state, ids, real, d, real, real, n, d, ids]
        dll.voxel_first_unknown.restype = None
        owner = array(dtype=np.int32, flags="C_CONTIGUOUS")
        dll.voxel_unknown_components.argtypes = [state, ids, owner]
        dll.voxel_unknown_components.restype = i
        dll.voxel_update_islands.argtypes = [state, ids, ids, n, n, byte]
        dll.voxel_update_islands.restype = None
        dll.voxel_claim_views.argtypes = [state, ids, real, d, real, n, real, n, d, byte, owner]
        dll.voxel_claim_views.restype = i
        dll.voxel_collect_views.argtypes = [state, ids, real, d, real, n, real, n, d,
                                           byte, owner, ids, n, ids]
        dll.voxel_collect_views.restype = n
        dll.grid_distances.argtypes = [byte, i, i, i, i, d, d, real]
        dll.grid_distances.restype = None
        return dll
    except (OSError, subprocess.CalledProcessError) as exc:
        warnings.warn(f"Voxel native kernels unavailable; using NumPy: {exc}")
        return None
