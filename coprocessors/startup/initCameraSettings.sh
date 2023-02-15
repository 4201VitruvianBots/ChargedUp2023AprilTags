#!/bin/bash

v4l2-ctrl --list-devices | grep "OV2311" -A1 | grep -v "OV2311" | while read -r line; do
  
done