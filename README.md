# ME41125: Introduction to Engineering Research
### Name: Nikhil Sethi
### Student number: 5711428

## Project goal
This repository contains code for reproducing the results of the paper titled "A self-healing algorithm for adaptive formation control with drone swarms". The software is capable of producing a simulation of multiple drones which can heal themselves when some drones are lost as part of the formation.

## Requirements
The code has been tested with the following platforms. There may be additional setup required for your own platform.
- Ubuntu 22.04
- Python 3.8.16 

## Install

A handy installation script is provided which should ideally install everything for you. There are some links below which can be useful to debug any problems. Feel free to open an issue if you face problems.

```bash
# clone this repository
git clone git@github.com:nikhil-sethi/formation-healing.git
cd formation-healing

# setup the requirements
chmod +x install.sh
./install.sh

```

[Ardupilot software setup](https://ardupilot.org/dev/docs/building-setup-linux.html)

## Test
The above process should build the Ardupilot software successfully and you can run the following command to test a basic simulation:
```
sim_vehicle.py -v ArduCopter --map -- console
```
You should see something like the following:


## Run
Close any testing sessions if you have open and run the following commands in a new terminal inside this projects repository.
```
python3 manager.py --gcs --server --simulation
```
If everything went well, there should

## Licensing
This project is licensed under the [GNU GPLv3 License](https://gist.github.com/kn9ts/cbe95340d29fc1aaeaa5dd5c059d2e60) - see the [LICENSE.txt](LICENSE.txt) file for details

## Authors

  - **Nikhil Sethi** 
    [Github](https://github.com/nikhil-sethi)


## Acknowledgements
This work would not be possible without Ardupilot's comprehensive framework for testing drones in simulation.