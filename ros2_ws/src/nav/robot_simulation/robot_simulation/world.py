"""Planar geometry for deterministic kinematics, ray casting and collision."""

import math


def corners(pose, bounds):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    x0, y0, x1, y1 = bounds
    return [(x+c*u-s*v, y+s*u+c*v) for u, v in
            ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def overlap(a, b):
    for poly in (a, b):
        for i in range(4):
            u, v = poly[i], poly[(i+1) % 4]
            axis = (u[1]-v[1], v[0]-u[0])
            pa = [x*axis[0]+y*axis[1] for x, y in a]
            pb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def collides(pose, bounds, size, obstacles):
    robot = corners(pose, bounds)
    if any(x <= 0 or y <= 0 or x >= size[0] or y >= size[1] for x, y in robot):
        return True
    return any(overlap(robot, corners((0, 0, 0), box)) for box in obstacles)


def advance(pose, velocity, dt):
    x, y, yaw = pose
    vx, vy, wz = velocity
    mid = yaw + wz*dt/2
    return [x+(vx*math.cos(mid)-vy*math.sin(mid))*dt,
            y+(vx*math.sin(mid)+vy*math.cos(mid))*dt,
            math.atan2(math.sin(yaw+wz*dt), math.cos(yaw+wz*dt))]


def segments(size, obstacles):
    result = []
    for box in ([0, 0, *size], *obstacles):
        points = corners((0, 0, 0), box)
        result.extend((points[i], points[(i+1) % 4]) for i in range(4))
    return result


def ray_distance(x, y, angle, edges, maximum=5.0):
    dx, dy = math.cos(angle), math.sin(angle)
    result = maximum
    for (ax, ay), (bx, by) in edges:
        sx, sy = bx-ax, by-ay
        denominator = dx*sy-dy*sx
        if abs(denominator) < 1e-10:
            continue
        distance = ((ax-x)*sy-(ay-y)*sx)/denominator
        fraction = ((ax-x)*dy-(ay-y)*dx)/denominator
        if distance >= 0 and 0 <= fraction <= 1:
            result = min(result, distance)
    return result
