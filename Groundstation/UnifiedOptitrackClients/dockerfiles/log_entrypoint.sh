#!/bin/bash

# Check if --filename or -o argument contains "/"
found_argument=
for arg in "$@"; do
    if [[ -n "$found_argument" || "$arg" == "-o"* || "$arg" == "--filename"* ]]; then
        if [[ $arg == *"/"* ]]; then
            echo "Error: when running with docker, --filename/-o has to be just a filename, no path allowed"
            exit 1
        fi
    fi

    if [[ "$arg" == "-o" || "$arg" == "--filename" ]]; then
        found_argument=t
    fi
done

cd /data
/home/build/mocap2log "$@"
