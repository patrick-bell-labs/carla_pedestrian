# CARLA dataset generation to capture pedestrians from multiple cameras

## Setup and Install CARLA UE 5

refer to https://carla-ue5.readthedocs.io/en/latest/start_quickstart/

you should download CARLA with tag `0.10.0`

## Start CARLA server in headless mode

Start a terminal, and make sure to change venv path to match your venv (set up during install), then start the server with

```
./start_carla_server_headless.sh
```

You should see this on the terminal, not other messages which could suggest errors.
```
Shutdown handler: initialize.
5.5.0-0+UE5 1013 0
Disabling core dumps.
```
This CARLA should be using Unreal Engine version 5.5, as seen above.

## Start client script to simulate and capture frames

Start another terminal, activate venv, and run the script with this example command

```
source venv/bin/activate
python3 sim_pedestrains.py -p 600 -f 45 -c 1.0 -s normal -w afternoon --fps 30 -n 10
```

Here is the usage.

```
usage: sim_pedestrains.py [-h] [-p POINTS] [-f FOV] [-c CAMERA_HEIGHT] [-s {slow,normal,fast}]
                          [-w {morning,afternoon,late_afternoon,evening}] [--fps FPS] [-n NUM_FRAMES]

A CARLA simulator client that spawns simulated pedestrians and vehicles and captures frames.

options:
  -h, --help            show this help message and exit
  -p POINTS, --points POINTS
                        Total number of pedestrain spawn points, final number of spawned pedestrains with AI controllers
                        will be approximately half
  -f FOV, --fov FOV     camera field of view e.g., 25, 35, 45
  -c CAMERA_HEIGHT, --camera_height CAMERA_HEIGHT
                        camera height from ground e.g., 1.0, 2.0, 3.0
  -s {slow,normal,fast}, --speed {slow,normal,fast}
                        Choose pedestrain walking speed.
  -w {morning,afternoon,late_afternoon,evening}, --weather {morning,afternoon,late_afternoon,evening}
                        Choose weather based on time of day; they have different lighting conditions
  --fps FPS             CARLA world tick FPS
  -n NUM_FRAMES, --num_frames NUM_FRAMES
                        Number of frames captured by sensors
```
## Captured frames saved to storage

Default output folder is `camera_capture_output` and filename is structured as camera_X_YYYYYY.png where X is camera index (starting from 1) and Y is frame index of the capture from camera X.

## Stop the server after simulation is done

Because the states and actors from the previous simulation may not be cleaned properly, on the server terminal, press Ctrl+C to stop the server process. If you need to run another simulation, restart the server again.




