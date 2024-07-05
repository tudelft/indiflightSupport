Install

    pip install -r requirements.txt

Run (I think Linux only right now, if using the IndiflightHIL interface)

    ./sim.py --hil /dev/ttyUSB0 --mocap 10.0.0.1 5005

See `./sim.py --help` for all interface config options.
Modify `./sim.py:runSim()` to suit your multicopter model needs

Open browser to http://localhost:5000
