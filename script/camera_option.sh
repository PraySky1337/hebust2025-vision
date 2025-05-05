#!/bin/bash

DEVICE=/dev/video0

# 设置分辨率 + MJPG 格式 + 帧率
v4l2-ctl -d $DEVICE \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --set-parm=30

# 设置手动曝光、增益、白平衡等
v4l2-ctl -d $DEVICE -c exposure_auto=1
v4l2-ctl -d $DEVICE -c exposure_absolute=10
v4l2-ctl -d $DEVICE -c gain=1
v4l2-ctl -d $DEVICE -c white_balance_temperature_auto=0
v4l2-ctl -d $DEVICE -c white_balance_temperature=4500
