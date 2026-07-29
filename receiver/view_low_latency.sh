#!/bin/sh
set -eu
url="${1:-rtsp://10.243.41.1:8554/live}"
exec ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay -framedrop "$url"
