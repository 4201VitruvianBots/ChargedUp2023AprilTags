import logging
import numpy as np

from common.cscoreServer import CSCoreServer

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.INFO)

testServer = CSCoreServer("test", ip='localhost', port=8085)

testFrame = np.ones((720, 1280), dtype=np.uint8)

while True:
    # print("Sending Test Frame")
    testServer.setFrame(testFrame)


