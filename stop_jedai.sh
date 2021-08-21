#!/bin/bash

pkill -f manage.py
pkill -f tmp.py
echo "Already running JEDAI server stopped."
rm temp_file.log
