
import numpy as np

TAG_SIZE_M = 0.1524
PADDING_PERCENTAGE = 0.31

CAMERA_MOUNT_ANGLE = 0.0
CAMERA_MOUNT_HEIGHT = 0.0

CAMERA_IDS = {
    '14442C10218CCCD200': {
        'name': 'OAK-D',
        'table': 'forward_localizer',
    },
    '18443010B1FA0C1300': {
        'name': 'OAK-D Lite',
        'table': 'forward_localizer',
    },
    '18443010110CC71200': {
        'name': 'OAK-D Pro W',
        'table': 'forward_localizer',
    },
    '1944301061F9721300': {
        'name': 'OAK-D Pro W PoE',
        'table': 'forward_localizer',
    },
    '1944301061F9721301': {
        'name': 'OAK-D Pro W PoE',
        'table': 'rear_localizer',
    },
}

CAMERA_PARAMS = {
    "OAK-D": {
        "rgb": {
            "hfov": 69.0,
            "vfov": 55.0
        },
        "mono": {
            "hfov": 72.0,
            "vfov": 50.0
        },
    },
    "OAK-D PoE": {
        "rgb": {
            "hfov": 69.0,
            "vfov": 55.0
        },
        "mono": {
            "hfov": 72.0,
            "vfov": 50.0
        },
    },
    "OAK-D Lite": {
        "rgb": {
            "hfov": 69.0,
            "vfov": 54.0
        },
        "mono": {
            "hfov": 73.0,
            "vfov": 58.0
        },
    },
    "OAK-D Pro W": {
    # "boardName": "OAK-D Pro W 120",
        "rgb": {
            "hfov": 95.0,
            "vfov": 70.0
        },
        "mono": {
            "hfov": 128.0,
            "vfov": 80.0,
            "rhfov": 97.0,
            "rvfov": 70.0
        },
    },
    "OAK-D Pro W PoE": {
    # "boardName": "OAK-D Pro W 120",
        "rgb": {
            "hfov": 95.0,
            "vfov": 70.0
        },
        "mono": {
            "hfov": 128.0,
            "vfov": 80.0,
            "rhfov": 97.0,
            "rvfov": 70.0
        },
    }
}

OPOINTS = np.array([
    -1, -1, 0,
     1, -1, 0,
     1,  1, 0,
    -1,  1, 0,
    -1, -1, -2 * 1,
     1, -1, -2 * 1,
     1,  1, -2 * 1,
    -1,  1, -2 * 1,
]).reshape(-1, 1, 3) * 0.5 * TAG_SIZE_M

EDGES = np.array([
    0, 1,
    1, 2,
    2, 3,
    3, 0,
    0, 4,
    1, 5,
    2, 6,
    3, 7,
    4, 5,
    5, 6,
    6, 7,
    7, 4
]).reshape(-1, 2)
