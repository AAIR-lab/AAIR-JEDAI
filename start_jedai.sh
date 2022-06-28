#!/bin/bash

./stop_jedai.sh
echo "Starting new JEDAI server"
python3 manage.py runserver > out.txt 2>&1 &
PID=$(echo $!)
echo "$PID" > pid.txt
sleep 3s
ADDR=$(cat out.txt | grep "Starting development server at" | tr -s ' ' | cut -d ' ' -f 5)
xdg-open "$ADDR" &
echo "Started JEDAI server with process ID $PID."
