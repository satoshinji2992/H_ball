#!/bin/sh

app_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
bundled_model="$app_dir/models/stainless_ball.mud"

if [ -n "${1:-}" ]; then
    model="$1"
elif [ -f "$bundled_model" ]; then
    model="$bundled_model"
else
    model=/root/models/stainless_ball.mud
fi
uart_port="${2:-/dev/ttyS0}"
config="${3:-$app_dir/balance_calibration.cfg}"
recordings_dir="${4:-$app_dir/recordings}"

exec "$app_dir/h_ball_balance" "$model" "$uart_port" "$config" "$recordings_dir"
