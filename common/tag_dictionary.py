import json
import logging

log = logging.getLogger(__name__)

File_Path = 'resources/2023-chargedup.json'

try:
    with open(File_Path) as json_file:
        TAG_DICTIONARY = json.load(json_file)

        logging.debug(TAG_DICTIONARY)
except Exception as e:
    logging.error("Could not open tag dictionary!")
    TAG_DICTIONARY = {}
