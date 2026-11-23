#!/usr/bin/env bash
# Draws every frame of a page and encodes the video.
#   video/make_video.sh PAGE OUT.mp4 [WORKERS]
# PAGE is web/gazebo.html or web/generated.html. web/data.js must exist
# (build_data.py), and for gazebo.html web/frames must point at the take's
# frames. Needs node with playwright, and ffmpeg.
set -e
here=$(cd "$(dirname "$0")" && pwd)
page=$1 out=$2 workers=${3:-3}
total=1770                                   # LENGTH (story.js) x 30 frames per second
tmp=$(mktemp -d "${TMPDIR:-/tmp}/roslean-video.XXXXXX")
step=$(( (total + workers - 1) / workers ))
for ((i = 0; i < workers; i++)); do
  node "$here/render.mjs" "$page" "$tmp" $((i * step)) $(((i + 1) * step)) &
done
wait
[ "$(ls "$tmp" | wc -l)" -eq "$total" ] || { echo "missing frames in $tmp" >&2; exit 1; }
ffmpeg -y -loglevel error -framerate 30 -i "$tmp/%05d.png" \
  -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -movflags +faststart "$out"
rm -r "$tmp"
echo "wrote $out"
