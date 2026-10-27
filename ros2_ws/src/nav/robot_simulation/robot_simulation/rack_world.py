"""Planar rack contact model, independent of navigation and mission outcomes.

Empty robots collide with legs; a carried deck also collides with every other
deck. Vertical fit is intentionally NOT certified by this planar model.
"""
import math
from copy import deepcopy
from .world import corners, overlap


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def separation(a, b):
    if overlap(a, b):
        return 0.
    return min(point_segment(p, poly[i], poly[(i+1) % 4])
               for points, poly in ((a, b), (b, a)) for p in points for i in range(4))


def compose(pose, offset):
    x, y, yaw = pose
    u, v, theta = offset
    return [x+math.cos(yaw)*u-math.sin(yaw)*v,
            y+math.sin(yaw)*u+math.cos(yaw)*v, yaw+theta]


def relative(pose, target):
    dx, dy = target[0]-pose[0], target[1]-pose[1]
    return [math.cos(pose[2])*dx+math.sin(pose[2])*dy,
            -math.sin(pose[2])*dx+math.cos(pose[2])*dy,
            math.atan2(math.sin(target[2]-pose[2]), math.cos(target[2]-pose[2]))]


class RackWorld:
    def __init__(self, world, body_bounds):
        self.size = world['size']
        self.body = body_bounds
        self.spec = world['rack_geometry']
        self.racks = deepcopy(world['racks'])
        self.zone = world['destination_zone']
        self.static = [corners((0, 0, 0), b) for b in world['obstacles']]
        self.carried = None
        self.attachment = None
        self.moves = []
        self.outer = [-self.spec['depth']/2, -self.spec['width']/2,
                       self.spec['depth']/2, self.spec['width']/2]
        for r in self.racks:
            r['state'] = 'waiting'
        # Reject overlapping starting racks instead of silently drawing them.
        for i, a in enumerate(self.racks):
            for b in self.racks[i+1:]:
                if separation(corners(a['pose'], self.outer), corners(b['pose'], self.outer)) <= 0:
                    raise ValueError('Initial racks overlap')

    def rack_pose(self, rack, robot):
        return compose(robot, self.attachment) if rack['label'] == self.carried else rack['pose']

    def legs(self, rack, robot):
        x0, y0, x1, y1 = self.outer
        t = self.spec['leg']
        return [corners(self.rack_pose(rack, robot), [x, y, x+t, y+t])
                for x in (x0, x1-t) for y in (y0, y1-t)]

    def floor_polygons(self, robot):
        return self.static + [p for r in self.racks if r['label'] != self.carried for p in self.legs(r, robot)]

    def clearance(self, pose):
        body = corners(pose, self.body)
        moving = [body]
        pairs = [(body, p, 'body/rack_leg') for p in self.floor_polygons(pose)]
        if self.carried is not None:
            deck = corners(compose(pose, self.attachment), self.outer)
            moving.append(deck)
            pairs += [(deck, corners(r['pose'], self.outer), 'cargo/rack_'+r['label'])
                      for r in self.racks if r['label'] != self.carried]
            pairs += [(deck, p, 'cargo/static') for p in self.static]
        candidates = [(min(x, y, self.size[0]-x, self.size[1]-y), 'wall')
                      for poly in moving for x, y in poly]
        candidates.extend((separation(a, b), name) for a, b, name in pairs)
        return min(candidates)

    def swept_clearance(self, start, end, resolution=.002):
        """Conservative bound along linear centre/yaw interpolation.

        Each vertex moves at most resolution between samples. Subtracting half
        the per-step displacement covers points between samples (Lipschitz
        distance bound), including rotation; endpoints alone are insufficient.
        """
        dyaw = math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))
        radius = max(math.hypot(x, y) for x, y in corners((0, 0, 0), self.body))
        if self.carried:
            radius = max(radius, max(math.hypot(x, y) for x, y in corners(self.attachment, self.outer)))
        travel = math.hypot(end[0]-start[0], end[1]-start[1])+radius*abs(dyaw)
        count = max(1, math.ceil(travel/resolution))
        value, kind = min(self.clearance([start[0]+(end[0]-start[0])*i/count,
                        start[1]+(end[1]-start[1])*i/count, start[2]+dyaw*i/count])
                          for i in range(count+1))
        return value-travel/(2*count), kind

    def lift(self, pose, label):
        if self.carried is not None:
            raise ValueError('Cannot lift a second rack')
        rack = next(r for r in self.racks if r['label'] == label)
        delta = relative(pose, rack['pose'])
        if rack['state'] != 'waiting' or math.hypot(*delta[:2]) > .025 or abs(delta[2]) > .05:
            raise ValueError('Lift requires alignment with an undelivered rack (25 mm / 0.05 rad)')
        if self.clearance(pose)[0] <= .003:
            raise ValueError('Lift rejected: body contacts rack')
        self.carried, self.attachment = label, delta
        if self.clearance(pose)[0] <= .003:
            self.carried, self.attachment = None, None
            raise ValueError('Lifted rack would collide')
        rack['state'] = 'carried'
        self.moves.append({'event': 'attach', 'label': label, 'rack_pose': list(rack['pose']),
                           'robot_pose': list(pose), 'offset': list(delta)})

    def lower(self, pose, label):
        if self.carried != label:
            raise ValueError('Lower command does not match carried rack')
        rack = next(r for r in self.racks if r['label'] == label)
        placed = self.rack_pose(rack, pose)
        delta = relative(rack['destination'], placed)
        if math.hypot(*delta[:2]) > .04 or abs(delta[2]) > .08:
            raise ValueError('Rack is outside its allocated placement tolerance')
        polygon = corners(placed, self.outer)
        x0, y0, x1, y1 = self.zone
        if not all(x0 < x < x1 and y0 < y < y1 for x, y in polygon):
            raise ValueError('Entire rack must be inside the destination zone')
        if self.clearance(pose)[0] <= .003:
            raise ValueError('Lowering would overlap a rack or wall')
        # Keep the actual attachment transform. Never snap/teleport to a slot.
        old = rack['pose']
        rack['pose'] = placed
        self.carried = None
        if self.clearance(pose)[0] <= .003:
            self.carried = label
            rack['pose'] = old
            raise ValueError('Lowered legs would touch the robot')
        self.attachment = None
        rack['state'] = 'delivered'
        self.moves.append({'event': 'place', 'label': label, 'rack_pose': list(placed), 'robot_pose': list(pose)})

    def snapshot(self, pose):
        return [dict(r, pose=list(self.rack_pose(r, pose))) for r in self.racks]
