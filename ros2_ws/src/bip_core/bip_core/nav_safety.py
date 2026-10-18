"""ROS-independent conservative command gating for local navigation.

This is a simulation-oriented controller helper, NOT a hardware safety layer.
All obstacle distances and turn clearances must still be validated in Gazebo.
"""


def command_for_heading(heading_error, distance_remaining, max_lin, max_ang,
                        front_blocked=False, allow_turn=True,
                        turn_threshold=0.35):
    """Return (forward_mps, yaw_radps).

    - Never drive forward while the front LiDAR is blocked.
    - Rotate in place for meaningful heading corrections, provided a rotation
      is allowed by the *known-map* clearance check.
    - When blocked and already aligned, return (0, 0) so the caller can
      invoke bounded wait/replanning instead of moving into the obstacle.
    """
    error = float(heading_error)
    angular = max(-max_ang, min(max_ang, 1.8 * error))
    if front_blocked:
        if allow_turn and abs(error) >= turn_threshold:
            return (0.0, angular)
        return (0.0, 0.0)
    if abs(error) >= turn_threshold:
        return (0.0, angular)
    slow = min(1.0, max(0.0, distance_remaining) / 0.6)
    linear = max_lin * slow * (1.0 - 0.6 * abs(error) / 0.9)
    return (max(0.0, linear), angular)
