# PyNDIflight -- Simulation environment for Indiflight

## (partial) Software in the Loop simulation

This variant runs part of the Indiflight code on your local workstation, with 
the purpose of testing and debugging estimation and control code, but not
debugging device drivers, schedulers, or interfaces. We call such a build a 
`MOCKUP`, as it's not the whole package.

### Quickstart using Docker

(tested on Ubuntu 22.04, install docker like https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

Run all commands from this directory (`./Simulation`).

#### Step 1 -- Build the container:

    docker build . -t pyndiflight -f githash.Dockerfile \
        --build-arg COMMIT=f582a29 \
        --build-arg BUILD_CONFIG=./config/IndiflightConfig.mk

- downloads an `indiflight` commit
- builds the `MOCKUP` library using a build configuration (aka. `local.mk` file)
- provides facilities to pass in an `indiflight` configuration profile and python simulation script

#### Step 2 -- Run the container (sorry for the long arguments...) and open http://localhost:5000

    docker run -it -p 5000:5000                                \
        -v ./exampleQuadSim.py:/sim.py                         \
        -v ./config/exampleINDIflightProfile.txt:/profile.txt  \
        pyndiflight --throw --learn --sil-log

Explanation of arguments:

```
-it                                                    # ensure proper printing of status bar
-p 5000:5000                                           # Map ports for visualisation
-v ./exampleQuadSim.py:/sim.py                         # Map simulation script into container
-v ./config/exampleINDIflightProfile.txt:/profile.txt  # Map runtime config into container
pyndiflight                                            # Container tag (see build command)
--throw --learn --sil-log                              # see ./exampleQuadSim.py --help
```

#### Step 3 -- Get log from latest container

    docker cp $(docker ps -alq):/logs/ ./logs/

NB: logs are not persistent (unless you mount another volume to `:/logs`)


### Dev setup using Docker

#### Step 1 -- Build indiflight docker builder

Clone the indiflight repo (https://github.com/tudelft/indiflight), and build 
the builder container:

    docker build . -t indiflight -f <path/to/indiflight>/Dockerfile

(rebuild only when indiflight build pre-requisites change)

#### Step 2 -- Build local pyndiflight docker container

    docker build . -t pyndiflight-local -f local.Dockerfile \
        --build-arg BUILD_CONFIG=./config/IndiflightConfig.mk

(rebuild when `indiflight` build container or `PyNDIflight` simulation code changes)

#### Step 3 -- Run the container and open http://localhost:5000

    docker run -it -p 5000:5000                               \
        -v ./exampleQuadSim.py:/sim.py                        \
        -v ./config/exampleINDIflightProfile.txt:/profile.txt \
        -v <path/to/indiflight>:/indiflight                   \
        pyndiflight-local --throw --learn --sil-log

Explanation of arguments:

```
-it                                                    # ensure proper printing of status bar
-p 5000:5000                                           # Map ports for visualisation
-v ./exampleQuadSim.py:/sim.py                         # Map simulation script into container
-v ./config/exampleINDIflightProfile.txt:/profile.txt  # Map runtime config into container
-v <path/to/indiflight>:/indiflight                    # Map indiflight into the container
pyndiflight-local                                      # Container tag (see build command)
--throw --learn --sil-log                              # see ./exampleQuadSim.py --help
```

#### Step 4 -- Get log from latest container

    docker cp $(docker ps -alq):/logs/ ./logs/

NB: logs are not persistent (unless you mount another volume to `:/logs`)


### Software architecture and limitations

Interfacing is done using direct C API and the simulation is advanced using 
a `tick` function. The only parts of Indiflight that are run here are the 
following tasks:
- `FILTER` (gyro filtering) at every `tick()`
- `INNER_LOOP` (attitude/rate control) at every `tick()`
- `ACCEL` (accelerometer processing) at every second `tick()`
- `ATTITUDE` (attitude and position estimation using complementary filter) at every second `tick()`
- `EKF` at every second `tick()`
- `POS_CTL` at every second `tick()`



## Hardware in the Loop simulation (docs WIP)

With HIL simulation, Indiflight runs on the target hardware as it would during 
flight. However, a serial communications link provides simulated sensor values
in and accepts the computed actuator commands, instead of sending them to the 
real ESCs.

This can be used to test all components except the IMU and DSHOT drivers, but
requires a full Flight Controller + Remote Control setup, and additionally 
an FTDI serial-to-usb connection.

**Limitation: only up to 4 actuators for now**
**Limitation: no one-in-all dockerfile**

### Setup

#### Step 1 -- Build indiflight docker builder (same as Step 1 of Dev Setup for SIL)

Clone the indiflight repo (https://github.com/tudelft/indiflight), and build 
the builder container:

    docker build . -t indiflight -f <path/to/indiflight>/Dockerfile

(rebuild only when indiflight build pre-requisites change)


#### Step 2 -- Compile for target hardware

Provide a `make/local.mk` for your target hardware in the cloned indiflight
repo (see its README) and make sure to add `EXTRA_FLAGS += -DHIL_BUILD`.
This disables the dshot drivers and enables the HIL drivers.

baudrate).

Now compile and flash the resulting binary (also check README):

    docker run --privileged -v </path/to/indiflight>:/indiflight indiflight DEBUG=GDB dfu_flash


#### Step 3 -- Configure using indiflight-configurator

Setup the HIL interfaces in the Ports tab of the Configurator (use `auto`
Compile and flash INDIflight with `EXTRA_FLAGS += -DHIL_BUILD` in the build
configuration.


#### Step 4 -- Build the hil container

    docker build . -t pyndiflight-hil -f hil.Dockerfile


#### Step 5 -- Run the container and open http://localhost:5000

    docker run --privileged -it -p 5000:5000                     \
        -v ./exampleQuadSim.py:/sim.py                           \
        pyndiflight-hil --hil /dev/ttyUSB0 --mocap 10.0.0.5 5005 \
        --throw

