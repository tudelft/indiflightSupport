# Flying a Quadrotor with Unkown Actuators and Sensor Configuration

To reproduce the figures from the data logged during the experiment of our 
IMAV 2024 paper "Flying a Quadrotor with Unkown Actuators and Sensor Configuration",
run the commands below.

**NOTE**: Only tested on Ubuntu 22.04.

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
