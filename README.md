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


docker for the randomized simulation:

    docker buildx build -f randomize.Dockerfile --tag=randomizedSim .

    docker run -it -p 5000:5000 randomizedSim


To copy logfiles from the last-run container:

    docker cp $(docker ps -alq):/logs/ ./docker-simulation-logs