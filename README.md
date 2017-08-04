# MotionCaptureJointCalibration

[![Build Status](https://travis-ci.org/tkoolen/MotionCaptureJointCalibration.jl.svg?branch=master)](https://travis-ci.org/tkoolen/MotionCaptureJointCalibration.jl) [![codecov.io](http://codecov.io/github/tkoolen/MotionCaptureJointCalibration.jl/coverage.svg?branch=master)](http://codecov.io/github/tkoolen/MotionCaptureJointCalibration.jl?branch=master)

MotionCaptureJointCalibration provides functionality for kinematic calibration of robots given measurements of the positions of motion capture markers attached to the robot's links and positions of the robot's joints in a number of poses. It does so by solving a nonlinear program (NLP) with weighted square error between measured and predicted marker locations as the objective to minimize.

MotionCaptureJointCalibration is a Julia library built on top of [JuMP](https://github.com/JuliaOpt/JuMP.jl) and [RigidBodyDynamics.jl](https://github.com/tkoolen/RigidBodyDynamics.jl). JuMP makes it possible to choose between various NLP solvers. [Ipopt](https://github.com/JuliaOpt/Ipopt.jl) appears to perform fairly well for the problems formulated by this package.

## Features
Features include:
* handling of occlusions
* handling of measurements of the body-fixed locations of only a subset of the markers attached to the robot (the unknown marker positions will be solved for, given rough bounds)
* handling of measurements of only a subset of a robot's joint positions (the unknown joint positions will be solved for, given rough bounds)
* proper handling of quaternion-parameterized floating joints (unit norm constraints for quaternions)

Currently, MotionCaptureJointCalibration can only estimate constant offsets between measured and actual joint positions.

## News
* August 4, 2017: the package is under initial construction.

## Installation
MotionCaptureJointCalibration is not yet registered, and currently requires the master branches of Rotations and RigidBodyDynamics. Please see the [.travis.yml file](https://github.com/tkoolen/MotionCaptureJointCalibration.jl/blob/master/.travis.yml) for installation instructions on Linux and OSX.

## Usage
For now, see [the test suite](https://github.com/tkoolen/MotionCaptureJointCalibration.jl/blob/master/test/runtests.jl) for usage and to get a rough idea of the required quality of the input data.

## Acknowledgements
A variant of the NLP formulation used in this package is due to Michael Posa.
