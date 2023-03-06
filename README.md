# uad_software
Underwater Autonomous Drone project, aka "Bullet Gill" part of ECE 494/495 at the University of Alberta in Fall 2022 - Winter 2023

# Dependencies
The device relies on libraries for implementing PID controllers and reading from the inertial measurement unit.
```sh
sudo apt-get update
sudo pip install sparkfun-qwiic-icm20948
sudo pip install simple-pid
```
The RPi.GPIO module is also used, which comes pre-installed on Raspbian.
