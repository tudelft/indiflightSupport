#!/bin/bash

# example command to run this in a new terminal
#   alacritty -o "window.startup_mode=Maximized" -e ./dashboard.sh 1003 
#

# Watch out! Stop all commands before closing the terminal, catching that still
# doesnt work

echo_help_and_exit() {
    echo "usage: $0 <rigid_body_id> [--test] [--help/-h]"
    exit 1
}

# check for arguments
if [[ $# -gt 2 ]] || [[ $# -lt 1 ]]; then
    echo_help_and_exit
fi

while test $# != 0
do
    left=$(echo "$1" | cut -d "=" -f 1)
    right=$(echo "$1" | cut -d "=" -f 2)
    case "$left" in
    --test) TEST_FLAG=--test ;;
    --help) echo_help_and_exit ;;
    -h) echo_help_and_exit ;;
    *) RB_ID=$left ;;
    esac
    shift
done

if [[ $RB_ID -lt 0 ]] || [[ -z $RB_ID ]]; then
    echo "Invalid Rigid Body ID"
    echo_help_and_exit
fi

# define ssh stuff
ON_PI="/usr/bin/sshpass -p pi ssh -o StrictHostKeyChecking=no pi@10.0.0.1"

# make sure program exits properly
trap "exit" INT TERM
trap "kill 0" EXIT

# setup tmux
session="dashboard"
tmux kill-server
tmux start-server
tmux new-session -d -s $session
tmux set mouse on

# make panes like this 
# +-----+------+
# | .0  |  .2  |
# +-----|------+
# | .1  |  .3  |
# +-----+------+
#
tmux splitw -h -t "$session:0.0"
tmux splitw -v -t "$session:0.0"
tmux splitw -v -t "$session:0.2"

# start uart comms. UPDATE: now assumed that relay.service is running, check README
#tmux send-keys -t $session:0.2 "$ON_PI sudo killall relay\;"
#tmux send-keys -t $session:0.2 "$ON_PI /home/pi/relay/build-aarch64-linux-gnu/relay" ENTER
tmux send-keys -t $session:0.2 "$ON_PI sudo systemctl status relay.service" ENTER

# start natnet2udp.py in udp mode
tmux send-keys -t $session:0.0 "./optitrack_forwarder/natnet2udp.py -ac $RB_ID 0 -f 20 -le right -an far -xs right -up y_up -udp $TEST_FLAG" ENTER

# setpoints
tmux send-keys -t $session:0.1 './setpoint_sender/setpointSender.py --pos 0 0 1.0 --yaw 0' ENTER
#tmux send-keys -t $session:0.1 'sudo python3 setpoint_sender/track_test.py' ENTER

# ping
tmux send-keys -t $session:0.3 'ping 10.0.0.1' ENTER

# periodically get stats by sending USER 1 signal to process
#$ON_PI "while [[1]]; do; kill -USE2 `pidof connect`; sleep 5; done" &

tmux attach-session -t $session

