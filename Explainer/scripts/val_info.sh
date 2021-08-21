#!/bin/bash

# TODO make the dependency on versions of VAL that have this exact output explicit
prec_missing="has an unsatisfied"
prec_negated="to false"
goal_missing="The goal is not satisfied"

# path to Validate binary
VAL_PATH=$4

# validate plan given domain and problem
${VAL_PATH} -v $1 $2 $3 > val_out.txt
output_prec=$(cat val_out.txt | grep "${prec_missing}" | wc -l)
output_goal=$(cat val_out.txt | grep "${goal_missing}" | wc -l)
if [ ${output_prec} -gt 0 ]; then
    failed_action=$(cat val_out.txt | grep "${prec_missing}" | awk -F "${prec_missing}" '{print $1}')
    actual_pred=`cat val_out.txt | grep -A 2 "${prec_missing}" | awk NF | tail -n 1 | sed 's/(Set .*(/(/' | sed 's/).*)/)/'`
    failed_step=`cat val_out.txt | grep "${prec_missing}" | tail -n 1 | awk '{print $NF}'`
    output_negated=$(cat val_out.txt | grep "${prec_negated}" | wc -l)
    if [ ${output_negated} -gt 0 ]; then
        echo "negated-precondition@${failed_step}@${failed_action}@${actual_pred}"
    else
        echo "precondition@${failed_step}@${failed_action}@${actual_pred}"
    fi
elif [ ${output_goal} -gt 0 ]; then
    # TODO assuming all goals are positive because negative goals are not parsed correctly
    failed_goal=$(cat val_out.txt | grep "(Set" | grep " to true)" | sed 's/(Set //' | sed 's/ to true)//' | tr '\n' '#')
    echo "goal@${failed_goal}"
else
    echo "False"
fi

rm val_out.txt
