"""Deterministic image fixtures; not an optical camera or rack visibility model."""
import cv2
import numpy as np


def qr_image(payload, size=480):
    encoded = cv2.QRCodeEncoder_create().encode(payload)
    image = np.pad(encoded, 4, constant_values=255)
    return cv2.cvtColor(cv2.resize(image, (size, size), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)


def variants(payload):
    base = qr_image(payload)
    rotation = cv2.getRotationMatrix2D((240, 240), 15, .85)
    src = np.float32([[0, 0], [479, 0], [479, 479], [0, 479]])
    dst = np.float32([[35, 20], [440, 55], [470, 435], [15, 465]])
    perspective = cv2.getPerspectiveTransform(src, dst)
    occluded = base.copy()
    cv2.rectangle(occluded, (215, 215), (260, 260), (255, 255, 255), -1)
    multi = np.full((600, 900, 3), 255, np.uint8)
    multi[60:540, 20:500] = base
    multi[180:420, 620:860] = qr_image('UNRELATED', 240)
    yield 'clean', base, True
    yield 'rotate_90', cv2.rotate(base, cv2.ROTATE_90_CLOCKWISE), True
    yield 'rotate_15', cv2.warpAffine(base, rotation, (480, 480), borderValue=(255, 255, 255)), False
    yield 'perspective', cv2.warpPerspective(base, perspective, (480, 480), borderValue=(255, 255, 255)), False
    yield 'dim', (base.astype(np.float32)*.35).astype(np.uint8), False
    yield 'low_contrast', (base.astype(np.float32)*.25+120).astype(np.uint8), False
    yield 'blur_mild', cv2.GaussianBlur(base, (5, 5), 1.2), False
    yield 'blur_strong', cv2.GaussianBlur(base, (31, 31), 8.), False
    yield 'occlusion', occluded, False
    yield 'small', cv2.resize(base, (96, 96), interpolation=cv2.INTER_AREA), False
    yield 'multiple_codes', multi, False
