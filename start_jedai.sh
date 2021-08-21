#!/bin/bash

./stop_jedai.sh
echo "Starting new JEDAI server"
python3 manage.py runserver > temp_file.log &
sleep 3s
xdg-open "http://127.0.0.1:8000"
echo "Started JEDAI server."
