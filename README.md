# Rocky Rockets

a python project built to simulate 2d rocket flights, weather conditions, and stabilization systems from scratch.

## what is inside

this repo contains an iterative progression of python simulation scripts, starting from a basic physics calculator and working up to a full flight and landing loop:

* **`rock.py`** - the starting script. calculates basic vertical ascent, gravity, and apogee.
* **`rock2.py`** - adds max q (maximum dynamic pressure) and transonic drag spikes around the sound barrier.
* **`rock3.py`** - adds a full atmosphere model (temperature, humidity, and virtual temperature) plus wind shear.
* **`rock4.py`** - introduces rotational dynamics, a PID controller, and active thrust vector control (TVC) to keep the rocket stable against crosswinds.
* **`rock5.py`** - the final version. handles the full flight sequence: ascent, coast, descent, and a retro-thrust landing burn.

## how to run

make sure you have python installed, open your terminal in the project folder, and run any of the scripts:

```powershell
python rock5.py
