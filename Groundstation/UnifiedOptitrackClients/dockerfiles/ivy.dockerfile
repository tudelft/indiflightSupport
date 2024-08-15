# Base image
FROM ubuntu:22.04

# Update sources and install boost and build tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        libboost-all-dev \
        software-properties-common \
        build-essential \
        cmake \
    # Remove apt cache
    && rm -rf /var/lib/apt/lists/*

# Install Ivy
ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN add-apt-repository -y ppa:paparazzi-uav/ppa && apt-get update && apt-get install -y ivy-c-dev \
    && rm -rf /var/lib/apt/lists/*

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'CLIENTS=ivy' .. && make

# Add the entrypoint script
ADD ./dockerfiles/ivy_entrypoint.sh .
RUN chmod +x ./ivy_entrypoint.sh
ENTRYPOINT ["./ivy_entrypoint.sh"]
