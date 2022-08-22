Frequently Asked Questions
===================================

|

**1. I keep getting the error ``OSError: [Errno 98] Address already in use``. What to do?**

This error comes when you run the start script more than once without using the stop script in between. Please kill the already running process
using the following command in the terminal.

.. code-block :: bash

    pkill -f manage.py

|
|

