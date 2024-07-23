#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/tmp_catkin_ws/devel/setup.bash

OUTPUT_DIR=$1
LOG_FILENAME=$2
TIMELIMIT_IN_SEC=$3

# We need unbufferd piping since it seems the
# segfault from openrave puts the buffers in
# a bad state and as a result an incomplete
# log is captured.
timeout $TIMELIMIT_IN_SEC \
    stdbuf -o0 \
    python TMP.py "${@:4}" \
    &> "$OUTPUT_DIR"/"$LOG_FILENAME"

# Capture the return value of this command.
# We will return it later. This is the
# value that matters.
return_value=$?

# Make everything readable and writable.
chmod -R ugo+rw $OUTPUT_DIR/*

exit $return_value
