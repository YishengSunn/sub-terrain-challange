#!/usr/bin/env bash

set -euo pipefail

rviz2_bin="$(command -v rviz2)"

for lib in \
  "/opt/ros/jazzy/lib/liboctomap.so" \
  "/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9" \
  "/usr/lib/x86_64-linux-gnu/liboctomap.so" \
  ""
do
  [[ -z "${lib}" || -f "${lib}" ]] && break
done

if [[ -n "${lib}" ]]; then
  exec env LD_PRELOAD="${lib}${LD_PRELOAD:+:$LD_PRELOAD}" "${rviz2_bin}" "$@"
fi

exec "${rviz2_bin}" "$@"
