# Base image
FROM ubuntu:22.04

# Update sources and install boost and build tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        libboost-all-dev \
        build-essential \
        cmake \
    # Remove apt cache
    && rm -rf /var/lib/apt/lists/*

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'CLIENTS=udp' .. && make

# Add the entrypoint script
ADD ./dockerfiles/udp_entrypoint.sh .
RUN chmod +x ./udp_entrypoint.sh
ENTRYPOINT ["./udp_entrypoint.sh"]
