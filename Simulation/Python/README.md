Install

    pip install -r requirements.txt

Run (I think Linux only right now, if using the IndiflightHIL interface)

    ./quadSimExample.py --hil /dev/ttyUSB0 --mocap 10.0.0.1 5005

If you have a compiled Indiflight TARGET=MOCKUP library, you can run that SIL:

    ./quadSimExample.py --sil

See `./quadSimExample.py --help` for all interface config options.
Modify `./quadSimExample.py` to suit your multicopter model needs

Open browser to http://localhost:5000
