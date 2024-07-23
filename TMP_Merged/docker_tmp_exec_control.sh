#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/tmp_catkin_ws/devel/setup.bash

DOMAIN=$1
OUTPUT_DIR=$2
PROBLEM_FILENAME=$3
LOG_FILENAME=$4
TIMELIMIT_IN_SEC=$5
ACTION_NAME=$6
POLICY_FILENAME=$7
ENV_PATH=$8

combined_filepath=$OUTPUT_DIR/combined.pddl
policy_filepath=$OUTPUT_DIR/$POLICY_FILENAME
log_filepath=$OUTPUT_DIR/$LOG_FILENAME

cat test_domains/$DOMAIN/Tasks/domain.pddl > $combined_filepath
cat $OUTPUT_DIR/$PROBLEM_FILENAME >> $combined_filepath

# We need unbufferd piping since it seems the
# segfault from openrave puts the buffers in
# a bad state and as a result an incomplete
# log is captured.
timeout $TIMELIMIT_IN_SEC \
    stdbuf -o0 \
    ./planners/mdp-lib/simulate_action.out $combined_filepath p01 $ACTION_NAME $policy_filepath \
    &> $log_filepath

echo "" >> $log_filepath

# Capture the return value of this command.
# We will return it later. This is the
# value that matters.
return_value=$?

timeout $TIMELIMIT_IN_SEC \
    stdbuf -o0 \
    python TMP.py --domain $DOMAIN --assume-refinable --store-policy-tree --store-simulated-executions --problem $OUTPUT_DIR/$PROBLEM_FILENAME --ll-file $OUTPUT_DIR/"$PROBLEM_FILENAME".ll.pkl --policy-file $policy_filepath --output-dir $OUTPUT_DIR --env-path $ENV_PATH \
    &>> $log_filepath

# Make everything readable and writable.
chmod -R ugo+rw $OUTPUT_DIR/*

exit $return_value
