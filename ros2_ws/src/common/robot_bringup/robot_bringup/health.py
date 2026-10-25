"""Pure freshness calculation shared by the readiness CLI and its checks."""
import math


def freshness(received_at, stamp_s, wall_now, ros_now, max_age):
    if received_at is None:
        return 'no messages received'
    if not all(math.isfinite(v) for v in (stamp_s, wall_now, ros_now, max_age)):
        return 'invalid timestamp or clock'
    if wall_now - received_at > max_age:
        return 'message stream stopped'
    if stamp_s <= 0:
        return 'zero/missing ROS timestamp'
    age = ros_now - stamp_s
    if age < -0.5:
        return 'timestamp is ahead of ROS clock'
    if age > max_age:
        return 'stale message timestamp'
    return ''
