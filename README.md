# MotionCaptureJointCalibration

[![Build Status](https://travis-ci.org/JuliaRobotics/MotionCaptureJointCalibration.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MotionCaptureJointCalibration.jl) [![codecov.io](http://codecov.io/github/JuliaRobotics/MotionCaptureJointCalibration.jl/coverage.svg?branch=master)](http://codecov.io/github/JuliaRobotics/MotionCaptureJointCalibration.jl?branch=master)

MotionCaptureJointCalibration provides functionality for kinematic calibration of robots, given measurements of the positions of motion capture markers attached to the robot's links and positions of the robot's joints in a number of poses. It does so by solving a nonlinear program (NLP) with (weighted) square error between measured and predicted marker locations as the objective to minimize.

MotionCaptureJointCalibration is a small Julia library built on top of [JuMP](https://github.com/JuliaOpt/JuMP.jl) and [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl). JuMP makes it possible to choose between various NLP solvers. [Ipopt](https://github.com/JuliaOpt/Ipopt.jl) appears to perform fairly well for the problems formulated by this package.

## News
* October 18, 2017: [tagged version 0.0.1](https://github.com/JuliaRobotics/MotionCaptureJointCalibration.jl/releases/tag/v0.0.1).
* August 4, 2017: the package is under initial construction.

## Features
Features include:
* handling of occlusions
* handling of measurements of the body-fixed locations of only a subset of the markers attached to the robot (the unknown marker positions will be solved for, given rough bounds)
* handling of measurements of only a subset of a robot's joint positions (the unknown joint positions will be solved for, given rough bounds)
* proper handling of quaternion-parameterized floating joints (unit norm constraints for quaternions)
* visualization of calibration results using [RigidBodyTreeInspector](https://github.com/rdeits/RigidBodyTreeInspector.jl)

Currently, MotionCaptureJointCalibration can only estimate constant offsets between measured and actual joint positions.

## Installation
To install, simply run

```julia
Pkg.add("MotionCaptureJointCalibration")
```

This will install MotionCaptureJointCalibration and its required dependencies. RigidBodyTreeInspector.jl is an optional dependency and can be used to visualize the calibration results (`Pkg.add("RigidBodyTreeInspector")`). You'll also need an NLP solver that interfaces with JuMP, e.g. Ipopt (`Pkg.add("Ipopt")`).

## Usage
See [the demo notebook](https://github.com/JuliaRobotics/MotionCaptureJointCalibration.jl/blob/master/notebook/Demo.ipynb) for usage.

## Acknowledgements
A variant of the NLP formulation used in this package is due to Michael Posa.
