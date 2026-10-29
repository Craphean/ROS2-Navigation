"""Decide whether a 3D scan may be inserted into the world-frame map.

A stalled ros_gz bridge delivers a cloud and a TF sample that do not belong
to the same instant. Inserting that pair paints a second copy of every wall.
The robot cannot exceed its speed limit, so a pose change faster than that
limit is a clock/TF discontinuity and must be dropped.
"""


def pose_is_discontinuous(
    dt: float,
    dist: float,
    max_speed: float = 0.7,
    margin: float = 0.15,
) -> bool:
    """True when the robot could not have moved ``dist`` metres in ``dt`` seconds.

    ``dt`` is the cloud-stamp delta. Negative means sim time went backwards.
    ``max_speed`` sits above the TurtleBot3 limit (0.46 m/s) so a real move is
    kept; ``margin`` absorbs one noisy TF sample.
    """
    if dt < 0.0:
        return dist > margin
    return dist > max_speed * max(dt, 0.05) + margin
