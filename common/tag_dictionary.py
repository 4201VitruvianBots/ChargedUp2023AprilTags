import json

File_Path = './resources/2023-chargedup.json'

TAG_DICTIONARY = {}
with open(File_Path) as json_file:
    TAG_DICTIONARY = json.load(json_file)

print(TAG_DICTIONARY)
