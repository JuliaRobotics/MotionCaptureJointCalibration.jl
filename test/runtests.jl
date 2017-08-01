using MotionCaptureJointCalibration
using Base.Test
using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using Parameters
using ForwardDiff

import MotionCaptureJointCalibration: Point3DS, reconstruct!, deconstruct, _marker_residual, _∇marker_residual
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
num_poses = length(ground_truth_pose_data)

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

@testset "marker_residual" begin
    for data in measured_pose_data
        set_configuration!(state, data.configuration)
        _marker_residual(state, data.marker_positions, ground_truth_marker_positions, scales)
        # TODO: test something
    end
end

@testset "marker_residual gradient" begin
    function marker_residual_inefficient(x::X...) where {X}
        M = eltype(mechanism)
        state = MechanismState{X}(mechanism)
        marker_positions_body = OrderedDict{RigidBody{M}, Vector{Point3DS{X}}}(
            b => [Point3D(default_frame(b), zero(X), zero(X), zero(X)) for i = 1 : length(measured_marker_positions[b])] for b in markerbodies
        )
        reconstruct!(configuration(state), marker_positions_body, x...)
        setdirty!(state)
        _marker_residual(state, measured_pose_data[1].marker_positions, marker_positions_body, scales)
    end

    data = measured_pose_data[1]
    set_configuration!(state, data.configuration)
    J = _∇marker_residual(state, data.marker_positions, ground_truth_marker_positions, scales)

    f(args) = [marker_residual_inefficient(args...)]
    Jcheck = ForwardDiff.jacobian(f, deconstruct(data.configuration, ground_truth_marker_positions))
    @test isapprox(J, Jcheck, atol = 1e-14)
end
