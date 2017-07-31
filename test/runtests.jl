using MotionCaptureJointCalibration
using Base.Test
using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using Parameters

import MotionCaptureJointCalibration: Point3DS, reconstruct!, deconstruct, _marker_residual
import DataStructures: OrderedDict

include(joinpath(@__DIR__, "synthetic_data_generation.jl"))

T = Float64

val = Valkyrie()
mechanism = val.mechanism
state = MechanismState{T}(mechanism)

foot = findbody(mechanism, "leftFoot")
pelvis = findbody(mechanism, "pelvis")
markerbodies = [pelvis; foot]
scales = Dict(pelvis => 100., foot => 1.)

p = path(mechanism, pelvis, foot)
joints = collect(p)

srand(1)
ground_truth_marker_positions, measured_marker_positions = generate_marker_positions(markerbodies)
ground_truth_pose_data, measured_pose_data = generate_pose_data(state, ground_truth_marker_positions)

@testset "deconstruct/reconstruct!" begin
    q = zeros(num_positions(mechanism))
    num_markers = Dict(b => length(markers) for (b, markers) in measured_marker_positions)
    marker_positions_body = OrderedDict{RigidBody{T}, Vector{Point3DS{T}}}(b => [Point3D(default_frame(b), 0., 0., 0.) for i = 1 : num_markers[b]] for b in markerbodies)
    reconstruct!(q, marker_positions_body, deconstruct(configuration(state), ground_truth_marker_positions)...)
    @test all(q .== configuration(state))
    for (body, positions) in ground_truth_marker_positions
        @test all(marker_positions_body[body] .== positions)
    end
end

@testset "_marker_residual" begin
    for data in measured_pose_data
        set_configuration!(state, data.configuration)
        _marker_residual(state, data.marker_positions, ground_truth_marker_positions, scales)
        # TODO: test something
    end
end
