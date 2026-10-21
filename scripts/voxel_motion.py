"""Distance-based motion along a planned polyline."""
import math
import numpy as np

def advance(path, distance):
    if not path:
        raise ValueError("Expected a reachable path")
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        if length > distance and length > 1e-12:
            return tuple(np.asarray(a) + (np.asarray(b) - a) * distance / length)
        distance -= length
    return tuple(path[-1])
