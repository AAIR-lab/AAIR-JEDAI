#!/bin/bash

# path to Validate binary
VAL_PATH=$4

# validate plan given domain and problem
output=$(${VAL_PATH} $1 $2 $3 | grep "Successful plans:"| wc -l)

if [ ${output} -gt 0 ]; then
    echo "True"
else
    echo "False"
fi
