#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-5004}"

gst-launch-1.0 -v \
  udpsrc port="$PORT" caps="application/x-rtp,encoding-name=H264,payload=96" \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! autovideosink sync=false
