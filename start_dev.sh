#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/tmp_catkin_ws/devel/setup.bash

./stop_jedai.sh
echo "Starting new JEDAI server"
python3 manage.py runserver $(ifconfig eth0 | perl -ne 'print $1 if /inet\s.*?(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\b/'):8000 | tee out.txt &
PID=$(echo $!)
echo "$PID" > pid.txt
sleep 3s
ADDR=$(cat out.txt | grep "Starting development server at" | tr -s ' ' | cut -d ' ' -f 5)
xdg-open "$ADDR" &
echo "Started JEDAI server with process ID $PID."
