import json
import logging
import platform

from common.mathUtils import euler_from_quaternion

log = logging.getLogger(__name__)

File_Path = 'resources/2023-chargedup.json'

try:
    with open(File_Path) as json_file:
        TAG_DICTIONARY = json.load(json_file)
        for TAG in TAG_DICTIONARY['tags']:
            tagQuarternion = TAG['pose']['rotation']['quaternion']
            pitch, roll, yaw = euler_from_quaternion(tagQuarternion['X'], tagQuarternion['Y'], tagQuarternion['Z'], tagQuarternion['W'])
            TAG['pose']['rotation']['euler'] = {
                'pitch': pitch,
                'roll': roll,
                'yaw': yaw,
            }

        logging.debug(TAG_DICTIONARY)
except Exception as e:
    logging.error("Could not open tag dictionary!")
    TAG_DICTIONARY = {}

