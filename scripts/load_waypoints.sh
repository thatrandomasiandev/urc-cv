#!/usr/bin/env bash
set -e

usage() {
  echo "Usage: $0 lat1,lon1 [lat2,lon2 ...]"
  echo "  Example: $0 40.123456,-111.654321 40.123700,-111.654100"
  exit 1
}

if [[ $# -lt 1 ]] || [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
  usage
fi

json="["
sep=""
for pair in "$@"; do
  if [[ "$pair" != *,* ]]; then
    echo "error: expected lat,lon but got: $pair" >&2
    usage
  fi
  lat="${pair%%,*}"
  lon="${pair#*,}"
  json+="${sep}[${lat},${lon}]"
  sep=","
done
json+="]"

exec ros2 run urc_autonomy waypoint_loader --ros-args -p "waypoints:=${json}"
