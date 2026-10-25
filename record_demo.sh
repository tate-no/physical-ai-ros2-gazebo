#!/usr/bin/env bash
set -euo pipefail
export DISPLAY=:0
export XAUTHORITY=/home/ubuntu/.Xauthority
mkdir -p /root/color_nav_ws/recordings
video_path="/root/color_nav_ws/recordings/color_nav_$(date +%Y%m%d_%H%M%S).mp4"
printf 'Recording: %s\nPress q to stop and wait for finalization.\n' "$video_path"
ffmpeg -n -f x11grab -framerate 15 -i :0.0 -an -c:v libx264 -preset ultrafast -crf 23 -vf 'pad=ceil(iw/2)*2:ceil(ih/2)*2' -pix_fmt yuv420p -movflags +faststart "$video_path"
printf 'Saved: %s\n' "$video_path"
