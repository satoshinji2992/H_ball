#!/bin/sh
set -eu

url="${1:-rtsp://10.243.41.1:8554/live}"
out_dir="${2:-records}"
mkdir -p "$out_dir"
stamp="$(date +%Y%m%d_%H%M%S)"
out="$out_dir/H_test_$stamp.mkv"

echo "Recording $url -> $out"
echo "Press q to stop and finalize the video."
exec ffmpeg -rtsp_transport tcp -i "$url" -map 0:v:0 -c copy -metadata title="H ball balance test $stamp" "$out"
