#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-127.0.0.1}"
PORT="${2:-5004}"

# если есть камера — v4l2src; иначе — файл (укажи свой путь)
if v4l2-ctl --list-devices >/dev/null 2>&1; then
  SRC="v4l2src ! videoconvert"
else
  SRC="filesrc location="/mnt/c/Users/Андрюша/Desktop/230225223505_00.mp4" ! decodebin ! videoconvert"
fi

gst-launch-1.0 -v \
  ${SRC} \
  ! x264enc tune=zerolatency bitrate=${BITRATE:-2000} speed-preset=veryfast \
  ! rtph264pay pt=96 \
  ! udpsink host="$HOST" port="$PORT"
