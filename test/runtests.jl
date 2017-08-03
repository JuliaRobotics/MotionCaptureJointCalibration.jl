using MotionCaptureJointCalibration
using Base.Test
using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using Parameters
using ForwardDiff

import MotionCaptureJointCalibration: Point3DS, reconstruct!, deconstruct, _marker_residual, _∇marker_residual!
import DataStructures: OrderedDict

include(joinpath(@__DIR__, "synthetic_data_generation.jl"))

T = Float64

val = Valkyrie()
mechanism = val.mechanism
remove_fixed_tree_joints!(mechanism)
state = MechanismState{T}(mechanism)

foot = findbody(mechanism, "leftFoot")
pelvis = findbody(mechanism, "pelvis")
markerbodies = [pelvis; foot]
scales = Dict(pelvis => 1., foot => 1.) # be careful with these; having them be different orders of magnitiude can lead to numerical issues
p = path(mechanism, pelvis, foot)
correction_joints = collect(p)
free_joints = filter(isfloating, tree_joints(mechanism))
correction_param_bounds = Dict(j => [(-0.05, 0.05)] for j in correction_joints)

srand(1)
ground_truth_marker_positions, measured_marker_positions = generate_marker_positions(markerbodies)
ground_truth_offsets = generate_joint_offsets(correction_joints, 1e-2)
ground_truth_pose_data, measured_pose_data = generate_pose_data(state, ground_truth_marker_positions, ground_truth_offsets, free_joints)
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
    data = measured_pose_data[1]
    marker_positions_world = data.marker_positions

    function marker_residual_inefficient(x::X...) where {X}
        M = eltype(mechanism)
        state = MechanismState{X}(mechanism)
        marker_positions_body = OrderedDict{RigidBody{M}, Vector{Point3DS{X}}}(
            b => [Point3D(default_frame(b), zero(X), zero(X), zero(X)) for i = 1 : length(measured_marker_positions[b])] for b in markerbodies
        )
        reconstruct!(configuration(state), marker_positions_body, x...)
        setdirty!(state)
        _marker_residual(state, marker_positions_world, marker_positions_body, scales)
    end

    f(args) = [marker_residual_inefficient(args...)]

    for i = 1 : 100
        q = rand(num_positions(mechanism)) # needs to work for non-normalized quaternions as well
        Jcheck = ForwardDiff.jacobian(f, deconstruct(q, ground_truth_marker_positions))

        set_configuration!(state, q)
        g = zeros(length(Jcheck))
        _∇marker_residual!(g, state, marker_positions_world, ground_truth_marker_positions, scales)
        J = g'

        @test isapprox(J, Jcheck, atol = 1e-14)
    end
end
