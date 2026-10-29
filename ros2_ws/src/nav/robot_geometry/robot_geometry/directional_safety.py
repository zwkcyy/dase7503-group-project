"""Speed/stop checks; occluded space is never reconstructed as free space.

Central rear visibility is a necessary condition, not full rear coverage.
Narrow shadows remain outside that sector. Course exits require a controlled
field with no new objects entering those shadows; hardware needs rear sensing
to remove that restriction.
"""
import math
from itertools import product


def rear_visibility(scan, sensor_yaw=0., half_angle=math.radians(12)):
    """Require every central rear ray; NaN/missing samples are unknown."""
    if (not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0
            or not math.isfinite(scan.angle_min) or not scan.ranges):
        return False
    for angle in (math.pi-half_angle, math.pi, -math.pi+half_angle):
        relative = math.atan2(math.sin(angle-sensor_yaw), math.cos(angle-sensor_yaw))
        if not any(-.5 <= (relative+turn-scan.angle_min)/scan.angle_increment <= len(scan.ranges)-.5
                   for turn in (-2*math.pi, 0., 2*math.pi)):
            return False
    seen = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min+i*scan.angle_increment+sensor_yaw
        error = abs(math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi)))
        if error <= half_angle:
            seen.append(distance == math.inf or (math.isfinite(distance)
                        and scan.range_min <= distance <= scan.range_max))
    return len(seen) >= 5 and all(seen)


def braking_obstacle(points, bounds, command, measured=(0., 0., 0.),
                     reaction=.35, deceleration=.5, angular_deceleration=1., margin=.012):
    """Check the complete footprint through translation/rotation until stopping.

    Include measured momentum even after a zero request. Reaction time includes
    scan/processing/actuator latency; deceleration must be measured on hardware.
    """
    x0, y0, x1, y1 = bounds
    # Actuator axes need not transition simultaneously. A new forward request
    # can coexist with residual measured yaw; checking just the two endpoints
    # misses that curved swept footprint. Include every axis endpoint pairing.
    velocities = tuple(dict.fromkeys(product(*zip(command, measured))))
    horizons = [reaction+max(math.hypot(vx, vy)/(2*deceleration), abs(wz)/(2*angular_deceleration))
                for vx, vy, wz in velocities]
    # Rotation preserves the circumscribed radius. Points beyond this bound
    # cannot enter any swept rectangle; discard them before the inner loop.
    reach = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))+math.sqrt(2)*margin
    reach += max(math.hypot(v[0], v[1])*h for v, h in zip(velocities, horizons))
    points = [(x, y) for x, y in points if x*x+y*y <= reach*reach]
    if not points:
        return False
    for (vx, vy, wz), horizon in zip(velocities, horizons):
        for step in range(9):
            t = horizon*step/8
            angle = wz*t
            c, s = math.cos(angle), math.sin(angle)
            if abs(wz) > 1e-6:
                dx, dy = (vx*s+vy*(c-1))/wz, (vx*(1-c)+vy*s)/wz
            else:
                dx, dy = vx*t, vy*t
            for px, py in points:
                x, y = px-dx, py-dy
                u, v = c*x+s*y, -s*x+c*y
                if x0-margin <= u <= x1+margin and y0-margin <= v <= y1+margin:
                    return True
    return False


def reverse_policy(phase, vx, distance):
    if vx >= -.005:
        return ''
    if phase in ('EXIT_RACK', 'EXIT_DESTINATION'):
        if vx < -.085:
            return 'reverse_speed_limit'
        if distance >= .50:
            return 'reverse_distance_limit'
    elif phase in ('NAV_TO_RACK', 'NAV_TO_DESTINATION', 'DOCK_RACK', 'DOCK_DESTINATION'):
        if vx < -.035:
            return 'transit_reverse_speed_limit'
        if distance >= .08:
            return 'transit_reverse_distance_limit'
    else:
        return 'reverse_requires_mission_phase'
    return ''
