# Flying a Quadrotor with Unkown Actuators and Sensor Configuration

To reproduce the figures from the data logged during the experiment of our 
IMAV 2024 paper "Flying a Quadrotor with Unkown Actuators and Sensor Configuration",
run the commands below.

**NOTE**: Only tested on Ubuntu 22.04.


## Steps to reproduce Simulation

Build container (this also compiles the correct version of `indiflight`)

    docker build . -t pyndiflight-imav2024 -f Simulation/sil-githash.Dockerfile    \
        --build-arg COMMIT=b165ae127                                               \
        --build-arg BUILD_CONFIG=Documentation/Papers/IMAV2024/IndiflightConfig.mk

Run container with the python simulation configuration. Open <http://localhost:5000>:

    docker run -it -p 5000:5000                                                \
        -v ./Documentation/Papers/IMAV2024/simOveractuated.py:/sim.py          \
        -v ./Documentation/Papers/IMAV2024/IndiflightProfile.txt:/profile.txt  \
        pyndiflight-imav2024 --sil-log --throw --learn

Copy logs from most recently run container:

    docker cp $(docker ps -alq):/logs/. ./logs-simulation


## Steps to reproduce plots from dataset or simulation output

Create a new python venv, just to be sure about dependencies

    python -m venv ~/.indiflight-venv
    source ~/.indiflight-venv/bin/activate

Install dependencies:

    pip install --upgrade pip
    pip install -r LogAnalysis/indiflightLogTools/requirements.txt
    pip install sympy

Download the dataset <LINK> on some location on your machine.

Generate plots (they will appear in `Documentation/Papers/IMAV2024/figures`):

    python3 Documentation/Papers/IMAV2024/generateFigures.py /path/to/dataset

*NOTE*: the original plots were generated with `"text.usetex": True`, but that requires
a Latex installation. For portability, this is turned off by default here.
*NOTE*: running this will create cache files of the data, so they can be imported faster in the future. To delete the cache, run `python3 -c "from LogAnalysis.indiflightLogTools import IndiflightLog; IndiflightLog.clearCache()"`
