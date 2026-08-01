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
network_config="${4:-$app_dir/network.cfg}"

exec "$app_dir/h_ball_balance" "$model" "$uart_port" "$config" "$network_config"
