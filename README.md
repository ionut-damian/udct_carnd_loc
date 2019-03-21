# Overview
This repository contains the Kidnapped Vehicle Project of the udacity self driving car nanodegree. It uses a particle filter to localize a vehicle in a 2d map.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.
Visual Studio is also supported using vcpckg. 

### gcc

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Visual Studio

For setting up the environment for Visual Studio under Windows, follow these steps (credits go to [fkeidel](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/blob/master/VisualStudio/README.md)):

1. Install cmake
    * Download and run windows installer (see https://cmake.org/download/)

2. Install make
    * Download setup from   http://gnuwin32.sourceforge.net/packages/make.htm
    * Select 'Complete package, except sources - Setup'
    * Run downloaded setup

3. Clone and install vcpkg
    * The install script used in the next step will asume that you installed vckpgk in c:\\vcpkg. You can choose another location, but then you have to adapt VcPkgDir in line 13 in install-windows.bat
    * cd c:\\
    * git clone https://github.com/Microsoft/vcpkg.git
    * cd vcpkg
    * call bootstrap-vcpkg.bat

4. Adapt and call the install script for windows
    * cd to directory ide_profiles\\VisualStudio
    * Open install-windows.bat and adjust lines 5 to 7 to the   settings you will use when building your Visual Studio project    (platform, toolset, buildtype)
    * You could also pass these settings as command line arguments  to install-windows.bat
    * If you have more than one toolset installed, comment line 14  and uncomment line 15
    * call install-windows.bat
    * the install scipt will
        * set the build parameters for the libraries to install     (platform, toolset, buildtype)
        * use vcpkg to download, build and install uWebSockets
            * it will download the latest version of uWebSockets

5. Open solution and adapt toolset settings
    * Open Localisation.sln
    * Open project properties
    * Adapt target platform version and platform toolset (use the   same setting that you used in the install script)

6. Build project in Visual Studio
    * Build the project for the platform and buildtype you used in the install script

## Protocol

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

# The Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.
