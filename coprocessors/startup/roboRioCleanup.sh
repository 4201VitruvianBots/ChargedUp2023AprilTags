#!/usr/bin/bash

DISK_SPACE=$(df -h / | grep -oP "^.* \K([0-9][0-9])")

if [[ ${DISK_SPACE} -ge 80 ]]; then
  rm -rf /home/lvuser/*.wpilog
fi