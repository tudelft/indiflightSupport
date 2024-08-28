# Flying a Quadrotor with Unkown Actuators and Sensor Configuration

To reproduce the figures from the data logged during the experiment of our 
IMAV 2024 paper "Flying a Quadrotor with Unkown Actuators and Sensor Configuration",
run the commands below.

**NOTE**: Only tested on Ubuntu 22.04.

Install dependencies:

    pip install -r LogAnalysis/indiflightLogTools/requirements.txt

Generate plots (they will appear in `LogAnalysis/IMAV2024/figures`):

    python3 LogAnalysis/IMAV2024/generateFigures.py

*NOTE*: the original plots were generated with `"text.usetex": True`, but that requires
a Latex installation. For portability, this is turned off by default here.
