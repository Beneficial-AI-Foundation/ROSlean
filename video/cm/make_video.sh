#!/usr/bin/env bash
# Draws every frame of a page of the collision monitor video and encodes it,
# and writes the poster (OUT with .jpg) from the frame at POSTER seconds.
#   video/cm/make_video.sh PAGE OUT.mp4 [WORKERS] [POSTER]
# PAGE is web/generated.html or web/gazebo.html. web/data.js must exist
# (build_data.py), and for gazebo.html web/frames must point at the take's
# frames. Needs node with playwright (and the fonts, see video/README.md),
# and ffmpeg. The frames are drawn by ../render.mjs, as for the twist_mux
# video; the number of frames is the one build_data.py wrote.
set -e
here=$(cd "$(dirname "$0")" && pwd)
page=$1 out=$2 workers=${3:-3}
total=$(sed -n 's/.*"frameCount":\([0-9]*\).*/\1/p' "$here/web/data.js")
poster=${4:-$(sed -n 's/.*"slow":\([0-9.]*\).*/\1/p' "$here/web/data.js" | awk '{print $1 + 2.5}')}
tmp=$(mktemp -d "${TMPDIR:-/tmp}/roslean-video.XXXXXX")
step=$(( (total + workers - 1) / workers ))
for ((i = 0; i < workers; i++)); do
  node "$here/../render.mjs" "$page" "$tmp" $((i * step)) $(((i + 1) * step)) &
done
wait
[ "$(ls "$tmp" | wc -l)" -eq "$total" ] || { echo "missing frames in $tmp" >&2; exit 1; }
ffmpeg -y -loglevel error -framerate 30 -i "$tmp/%05d.png" \
  -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -movflags +faststart "$out"
frame=$(printf '%05d' "$(awk -v t="$poster" 'BEGIN { printf "%d", t * 30 }')")
ffmpeg -y -loglevel error -i "$tmp/$frame.png" -vf scale=1280:-1 -q:v 3 "${out%.mp4}.jpg"
rm -r "$tmp"
echo "wrote $out and ${out%.mp4}.jpg"
