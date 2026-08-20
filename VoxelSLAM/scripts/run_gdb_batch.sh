#!/usr/bin/env bash

set -o pipefail

gdb_log="${VOXELSLAM_GDB_LOG:-/tmp/voxelslam_gdb.log}"
export ASAN_OPTIONS="${ASAN_OPTIONS:-abort_on_error=1:detect_leaks=0:disable_coredump=0}"

exec gdb -q -batch \
  -ex "set pagination off" \
  -ex "set confirm off" \
  -ex "set print thread-events off" \
  -ex "set logging file ${gdb_log}" \
  -ex "set logging overwrite on" \
  -ex "set logging on" \
  -ex "handle SIGPIPE nostop noprint pass" \
  -ex "run" \
  -ex "thread apply all bt 30" \
  --args "$@"
