#!/bin/bash

./stop_jedai.sh
echo "Starting new JEDAI server"
python3 manage.py runserver > temp_file.log &
sleep 3s
ADDR=$(cat temp_file.log | grep "Starting development server at" | tr -s ' ' | cut -d ' ' -f 5)
xdg-open "$ADDR" &
echo "Started JEDAI server."
