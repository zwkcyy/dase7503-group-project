"""Perspective image of one printed marker with a calibrated pinhole model."""
import math

import cv2
import numpy as np


def post_image(texture, marker, base_pose, mount, rpy, width=640, height=480, hfov=1.4):
    """Render the front of the card into a monochrome camera image in body FLU axes."""
    x, y, heading = base_pose
    c, s = math.cos(heading), math.sin(heading)
    mount_x, mount_y, mount_z = mount
    camera = np.array([x+c*mount_x-s*mount_y, y+s*mount_x+c*mount_y, mount_z])
    centre = np.array(marker['xyz'])
    yaw = marker['yaw']
    normal = np.array([math.cos(yaw), math.sin(yaw), 0.])
    blank = np.full((height, width), 230, dtype=np.uint8)
    if (camera-centre) @ normal <= 0:
        return blank
    right = np.array([-normal[1], normal[0], 0.])*marker['width']/2
    up = np.array([0., 0., marker['width']/2])
    corners = np.array([centre-right+up, centre+right+up, centre+right-up, centre-right-up])
    roll, pitch, yaw_offset = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    ch, sh = math.cos(heading+yaw_offset), math.sin(heading+yaw_offset)
    rotation = np.array([[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr],
                         [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr], [-sp, cp*sr, cp*cr]])
    forward, left, vertical = ((corners-camera) @ rotation).T
    if min(forward) <= .005:
        return blank
    focal = width/(2*math.tan(hfov/2))
    pixels = np.column_stack((width/2-focal*left/forward, height/2-focal*vertical/forward))
    size = texture.shape[0]-1
    transform = cv2.getPerspectiveTransform(np.float32([[0,0],[size,0],[size,size],[0,size]]),
                                            pixels.astype(np.float32))
    return cv2.warpPerspective(texture, transform, (width, height), flags=cv2.INTER_LINEAR,
                               borderValue=230)
