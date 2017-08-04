# MotionCaptureJointCalibration

[![Build Status](https://travis-ci.org/tkoolen/MotionCaptureJointCalibration.jl.svg?branch=master)](https://travis-ci.org/tkoolen/MotionCaptureJointCalibration.jl) [![codecov.io](http://codecov.io/github/tkoolen/MotionCaptureJointCalibration.jl/coverage.svg?branch=master)](http://codecov.io/github/tkoolen/MotionCaptureJointCalibration.jl?branch=master)

MotionCaptureJointCalibration provides functionality for kinematic calibration of robots given a series of measurements of (a subset of) joint positions along with motion capture marker locations. It does so by solving a nonlinear program (NLP) with weighted square error between measured and predicted marker locations as the objective to minimize.

MotionCaptureJointCalibration is a Julia library built on top of [JuMP](https://github.com/JuliaOpt/JuMP.jl) and [RigidBodyDynamics](https://github.com/tkoolen/RigidBodyDynamics.jl). JuMP makes it possible to choose between various NLP solvers. [Ipopt](https://github.com/JuliaOpt/Ipopt.jl) appears to perform fairly well for the problems formulated by this package.

## News
* August 4, 2017: the package is currently under initial construction.

## Installation
MotionCaptureJointCalibration is not yet registered, and currently requires the master branches of Rotations and RigidBodyDynamics. Please see the [.travis.yml file](https://github.com/tkoolen/MotionCaptureJointCalibration.jl/blob/master/.travis.yml) for installation instructions on Linux and OSX.

## Usage
For now, see [the test suite](https://github.com/tkoolen/MotionCaptureJointCalibration.jl/blob/master/test/runtests.jl) for usage.

## Acknowledgements
A variant of the NLP formulation used in this package is due to Michael Posa.
