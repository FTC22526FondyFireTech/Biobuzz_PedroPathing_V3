import cv2
import numpy as np
import math

# Limelight 3A Snapscript pipeline - Biobuzz game piece detection.
# Detects red nectar, blue nectar and yellow pollen. For each color it reports
# the count and the distance / bearing of the NEAREST ball.
#
# llrobot (inputs from the robot, see PipelineConfigurables.toPythonInputs()):
#   each value = H * 65536 + S * 256 + V
#   [0] red low  [1] red high  [2] blue low  [3] blue high
#   [4] yellow low  [5] yellow high  [6],[7] reserved
#   If low H > high H the hue range wraps through 0 (red).
#   All zeros (robot not sending yet) -> the defaults below are used.
#
# llpython (outputs to the robot):
#   [0] red count    [1] red dist    [2] red angle
#   [3] blue count   [4] blue dist   [5] blue angle
#   [6] yellow count [7] yellow dist [8] yellow angle
#   [9] 1 if HSV ranges came from the robot, 0 if defaults were used
#   dist  = inches along the floor from the camera to the ball (0 if none seen)
#   angle = degrees from the camera's forward direction, + = ball to the right

# Geometry - keep in sync with PipelineConfigurables.java
CAMERA_HEIGHT_IN = 12.0
CAMERA_PITCH_DOWN_DEG = 20.0
NECTAR_DIAMETER_IN = 3.6
POLLEN_DIAMETER_IN = 2.8

# Limelight 3A field of view
HFOV_DEG = 54.5
VFOV_DEG = 42.0

# Contour filters (pixels at 640x480)
MIN_AREA = 150
MAX_AREA = 40000
MIN_CIRCULARITY = 0.40

# Defaults, used until the robot sends its own values: (lowH, lowS, lowV, highH, highS, highV)
DEFAULT_RED = (165, 130, 60, 10, 255, 255)
DEFAULT_BLUE = (100, 140, 50, 125, 255, 255)
DEFAULT_YELLOW = (18, 90, 80, 35, 255, 255)

KERNEL_CLOSE = np.ones((7, 7), np.uint8)
KERNEL_OPEN = np.ones((5, 5), np.uint8)

DRAW_BGR = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'yellow': (0, 255, 255)}


def unpack(value):
    v = int(value)
    return (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF


def read_range(llrobot, index, default):
    """Return (lowH, lowS, lowV, highH, highS, highV) from the packed inputs."""
    lo = llrobot[index]
    hi = llrobot[index + 1]
    if lo == 0 and hi == 0:
        return default
    lh, ls, lv = unpack(lo)
    hh, hs, hv = unpack(hi)
    return lh, ls, lv, hh, hs, hv


def make_mask(hsv, rng):
    lh, ls, lv, hh, hs, hv = rng
    if lh <= hh:
        mask = cv2.inRange(hsv, np.array([lh, ls, lv]), np.array([hh, hs, hv]))
    else:
        # hue wraps through 0 (red)
        m1 = cv2.inRange(hsv, np.array([lh, ls, lv]), np.array([180, hs, hv]))
        m2 = cv2.inRange(hsv, np.array([0, ls, lv]), np.array([hh, hs, hv]))
        mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_CLOSE)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_OPEN)
    return mask


def locate(cx, cy, w, h, diameter):
    """Ground distance (in) and bearing (deg) of a ball centre at pixel (cx, cy)."""
    fx = (w / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    fy = (h / 2.0) / math.tan(math.radians(VFOV_DEG / 2.0))
    x = (cx - w / 2.0) / fx          # right of centre
    y = (cy - h / 2.0) / fy          # below centre
    pitch = math.radians(CAMERA_PITCH_DOWN_DEG)

    # Ray in floor frame: forward = cos(p) - y*sin(p), down = sin(p) + y*cos(p)
    forward = math.cos(pitch) - y * math.sin(pitch)
    down = math.sin(pitch) + y * math.cos(pitch)
    if down <= 0.01 or forward <= 0:
        return None                  # ray never reaches the ball's height

    t = (CAMERA_HEIGHT_IN - diameter / 2.0) / down   # ball centre sits diameter/2 above floor
    ground_forward = t * forward
    lateral = t * x
    return math.hypot(ground_forward, lateral), math.degrees(math.atan2(lateral, ground_forward))


def find_balls(mask, w, h, diameter):
    """All valid balls in a mask as dicts with contour, distance, angle, centre."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    balls = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim < 10:
            continue
        if 4 * math.pi * area / (perim * perim) < MIN_CIRCULARITY:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        pos = locate(x + bw / 2.0, y + bh / 2.0, w, h, diameter)
        if pos is None:
            continue
        balls.append({'cnt': cnt, 'x': x, 'y': y, 'dist': pos[0], 'angle': pos[1]})
    return balls


def runPipeline(image, llrobot):
    h, w = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    from_robot = 1.0 if (llrobot[0] != 0 or llrobot[1] != 0) else 0.0
    ranges = {
        'red': read_range(llrobot, 0, DEFAULT_RED),
        'blue': read_range(llrobot, 2, DEFAULT_BLUE),
        'yellow': read_range(llrobot, 4, DEFAULT_YELLOW),
    }
    diameters = {'red': NECTAR_DIAMETER_IN, 'blue': NECTAR_DIAMETER_IN, 'yellow': POLLEN_DIAMETER_IN}

    out = []
    nearest = None
    for color in ('red', 'blue', 'yellow'):
        balls = find_balls(make_mask(hsv, ranges[color]), w, h, diameters[color])
        for b in balls:
            cv2.drawContours(image, [b['cnt']], -1, DRAW_BGR[color], 2)
        if balls:
            best = min(balls, key=lambda b: b['dist'])
            cv2.putText(image, "%.0fin %.0fdeg" % (best['dist'], best['angle']),
                        (best['x'], max(12, best['y'] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_BGR[color], 2)
            out += [float(len(balls)), float(best['dist']), float(best['angle'])]
            if nearest is None or best['dist'] < nearest['dist']:
                nearest = best
        else:
            out += [0.0, 0.0, 0.0]

    out.append(from_robot)

    # The nearest ball of any color becomes the tracked contour (Limelight tx/ty/ta)
    if nearest is not None:
        return nearest['cnt'], image, out
    return np.array([[]]), image, out
