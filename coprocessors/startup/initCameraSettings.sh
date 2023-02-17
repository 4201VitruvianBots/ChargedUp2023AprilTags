#!/bin/bash
#                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
#                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
#                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
#                            hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
# white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
#                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
#                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
#           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2
#      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
#                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
#         backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1
#                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
#              exposure_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157
#         exposure_auto_priority 0x009a0903 (bool)   : default=0 value=0

v4l2-ctl --list-devices | grep "OV2311" -A1 | grep -v "OV2311" | while read -r DEV;
  v4l2-ctl -d $DEV --set-fmt-video=width=1600,height=1200,pixelformat=MJPG
  v4l2-ctl -d $DEV -p 50
  v4l2-ctl -d $DEV -c brightness=0
  v4l2-ctl -d $DEV -c contrast=32
  v4l2-ctl -d $DEV -c saturation=64
  v4l2-ctl -d $DEV -c hue=0
  v4l2-ctl -d $DEV -c white_balance_temperature_auto=1
  v4l2-ctl -d $DEV -c gamma=100
  v4l2-ctl -d $DEV -c gain=0
  v4l2-ctl -d $DEV -c power_line_frequency=2
#  v4l2-ctl -d $DEV -c white_balance_temperature=4600
  v4l2-ctl -d $DEV -c sharpness=3
  v4l2-ctl -d $DEV -c backlight_compensation=0
  v4l2-ctl -d $DEV -c exposure_auto=1
  v4l2-ctl -d $DEV -c exposure_absolute=157
  v4l2-ctl -d $DEV -c exposure_auto_priority=0
done
