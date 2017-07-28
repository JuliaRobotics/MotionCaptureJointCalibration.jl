using MotionCaptureJointCalibration
using Base.Test
using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using Parameters

import MotionCaptureJointCalibration: Point3DS

include("synthetic_data_generation.jl")

T = Float64

val = Valkyrie()
mechanism = val.mechanism
state = MechanismState{T}(mechanism)

foot = findbody(mechanism, "leftFoot")
pelvis = findbody(mechanism, "pelvis")
markerbodies = [pelvis; foot]

p = path(mechanism, pelvis, foot)
joints = collect(p)

srand(1)
ground_truth_marker_positions, measured_marker_positions = generate_marker_positions(markerbodies)
ground_truth_pose_data, measured_pose_data = generate_pose_data(state, ground_truth_marker_positions)



