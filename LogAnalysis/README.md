


### Remote `./log_downloader`

If the flight controller is connected to a `racebian` companion computer, the
shell script `log_downloader/get_logs.sh` can be used to mirror the logfiles
present on the flight controller to a local directory. This is more stable 
and convenient than dealing with SD cards directly.

See `./log_downloader/get_logs.sh --help` for more.


### Analysis with Betaflight Blackbox Explorer

Supported on Linux only (Ubuntu 22.04 LTS tested).

Download the special `indi` version from <https://github.com/tudelft/blackbox-log-viewer/releases>. It will open both Betaflight and Indiflight logs

The `blackbox-explorer-workspace.json` file can be drag-and-dropped into the
Betaflight Blackbox Viewer and has some usefult workspaces to analyse
Indiflight logs.

NOTE: All coordinate frames are either Forward-Right-Down or North-East-Down.


### Decoding with `./indiflightLogTools`

The python module `indiflightLogTools` provides a class that can decode 
Betaflight/Indiflight logs and convert them to SI units.
This way, you can build your own data analysis tools. 

Conversion to `csv` is also possible, using the script `bfl2csv.py`.

Install python requirements first:

    pip install -r requirements.txt

NOTE: All coordinate frames are either Forward-Right-Down or North-East-Down.
