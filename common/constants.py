import cscore
import numpy as np

TAG_SIZE_M = 0.1524
PADDING_PERCENTAGE = 0.31

CAMERAS = {
    'Left_Localizers': {
        # X, Y Z
        'mount_offset': [0, 0, 0],
        # pitch, roll, yaw
        'mount_angle': [0, 0, 0],
        'nt_name': 'lLocalizer',
        'ids': {
            '14442C10218CCCD200': 'OAK-D',
            '18443010B1FA0C1300': 'OAK-D Lite',
            '18443010110CC71200': 'OAK-D Pro W',
            '1944301061F9721300': 'OAK-D Lite',
            'OV2311_1': 'OV2311',
        }
    },
    'Right_Localizers': {
        # X, Y Z
        'mount_offset': [0, 0, 0],
        # pitch, roll, yaw
        'mount_angle': [0, 0, 0],
        'nt_name': 'rLocalizer',
        'ids': {
            '1944301061F9721301': 'OAK-D Pro W PoE'
        }
    }
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
    },
    "OV2311": {
        "height": 1200,
        "width": 1600,
        "fps": 50,
        "pixelFormat": cscore.VideoMode.PixelFormat.kGray,
        "mono": {
            "hfov": 75.0,
            "vfov": 75.0, # TO BE DETERMINED
        },
        "camera_matrix": {
            "OV2311_1": np.array([
                [9.0497686115005104e+02, 0.0, 8.3740487457829317e+02],
                [0.0, 9.0747903505359182e+02, 6.7385900514896127e+02],
                [0.0, 0.0, 1.0]
            ]),
            "OV2311_6328": np.array([
                [8.7638714483680690e+02, 0.0, 8.4331555653705630e+02],
                [0.0, 8.7644598649542741e+02, 6.5641811849711667e+02],
                [0.0, 0.0, 1.0]
            ])
        }
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
