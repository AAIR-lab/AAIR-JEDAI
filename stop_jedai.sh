#!/bin/bash

PID=$(cat pid.txt | tr -s ' ')
kill "$PID"
pkill -f manage.py
pkill -f tmp.py
echo "Already running JEDAI server stopped."
rm pid.txt
rm out.txt
