# Building the webpage.

This note has been tested on Ubuntu 18.04.6 LTS

## Installation of required software

```
apt install python3-sphinx sphinx-bootstrap-theme python3-sphinx-copybutton
```

Navigate to the `docs/` directory using `cd docs`

You can clean the `build` directory by running `rm -fr build`

Now, run `make html` to generate all html files

Finally, you can view the changes by executing `pushd ../ && ./start_jedai.sh && popd`
