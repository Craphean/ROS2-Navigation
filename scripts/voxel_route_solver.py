"""Subset dynamic programming shared by the two voxel coverage planners."""
from __future__ import annotations
from typing import List, Tuple
import numpy as np

def _held_karp(
    start_costs: np.ndarray, pairwise: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = len(start_costs)
    size = 1 << n
    dp = np.full((size, n), np.inf, dtype=np.float64)
    parent = np.full((size, n), -1, dtype=np.int16)
    for index in range(n):
        dp[1 << index, index] = float(start_costs[index])

    for mask in range(1, size):
        bits = mask
        while bits:
            lsb = bits & -bits
            last = lsb.bit_length() - 1
            bits -= lsb
            previous = mask ^ lsb
            if previous == 0:
                continue
            values = dp[previous] + pairwise[:, last]
            best_previous = int(np.argmin(values))
            dp[mask, last] = values[best_previous]
            parent[mask, last] = best_previous

    table = np.full(size, np.inf, dtype=np.float64)
    end_choice = np.full(size, -1, dtype=np.int16)
    table[0] = 0.0
    for mask in range(1, size):
        values = dp[mask]
        end = int(np.argmin(values))
        table[mask] = values[end]
        end_choice[mask] = end
    return table, parent, end_choice


def _reconstruct_route(
    mask: int, parent: np.ndarray, end_choice: np.ndarray
) -> List[int]:
    end = int(end_choice[mask])
    route: List[int] = []
    while mask and end >= 0:
        route.append(end)
        previous = mask ^ (1 << end)
        if previous == 0:
            break
        end = int(parent[mask, end])
        mask = previous
    route.reverse()
    return route
