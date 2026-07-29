#!/bin/sh
set -eu
if [ "$#" -ne 1 ]; then echo "usage: $0 records/H_test_*.mkv"; exit 2; fi
exec ffplay "$1"
