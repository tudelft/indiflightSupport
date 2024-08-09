# Indiflight Support Package

- log analyzer
- cinerat instructions and config files
- groundstation code (i.e. optitrack script, setpoint stuff)
- racebian instructions and pi-cross compiler
- information on layout
- information on controllers / papers


submodules and instructions
- configurator
- blackbox

# Docker image for Software in the Loop simulation

Tested on x86 running Ubuntu 22.04.




Build docker image for the randomized simulation:

    docker buildx build -f randomize.Dockerfile --tag=randomizedSim .


Run it 

    docker run -it -p 5000:5000 randomizedSim

Additional arguments that can be appended to the above command:

```
  --no-vis              Do not launch visualization webserver (default: False)
  --no-real-time        Run as fast as possible (default: False)
  --verbose             Print debugging output from INDIflight (default: False)
```

Copy logfiles from the last-run container:

    docker cp $(docker ps -alq):/logs/ ./docker-simulation-logs
