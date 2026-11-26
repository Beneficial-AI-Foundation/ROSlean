#!/usr/bin/env bash
# Draws every frame of a page and encodes the video.
#   video/make_video.sh PAGE OUT.mp4 [WORKERS]
# PAGE is web/gazebo.html or web/generated.html. web/data.js must exist
# (build_data.py), and for gazebo.html web/frames must point at the take's
# frames. Needs node with playwright, and ffmpeg.
# For a soundtrack, set AUDIO to a sound file and AUDIO_START to the second
# of it at which the video should begin (default 0). It fades in and out.
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
sound=()
if [ -n "${AUDIO:-}" ]; then
  [ -f "$AUDIO" ] || { echo "no such sound file: $AUDIO" >&2; exit 1; }
  len=$((total / 30))
  sound=(-ss "${AUDIO_START:-0}" -t "$len" -i "$AUDIO"
         -af "afade=t=in:d=1.5,afade=t=out:st=$((len - 4)):d=4"
         -c:a aac -b:a 192k -shortest)
fi
ffmpeg -y -loglevel error -framerate 30 -i "$tmp/%05d.png" "${sound[@]}" \
  -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -movflags +faststart "$out"
rm -r "$tmp"
echo "wrote $out"
