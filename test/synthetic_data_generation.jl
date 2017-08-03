using MotionCaptureJointCalibration
using RigidBodyDynamics
using StaticArrays
using Parameters

import MotionCaptureJointCalibration: Point3DS

@with_kw struct MarkerPositionGenerationOptions
    marker_measurement_stddev::Float64 = 1e-5
    marker_offset_max::Float64 = 0.1
    num_markers::Int = 4
    num_measured_markers::Int = 3#4
end

function generate_marker_positions(bodies::AbstractVector{<:RigidBody}, options::MarkerPositionGenerationOptions = MarkerPositionGenerationOptions())
    # Marker positions in body frame
    B = eltype(bodies)
    T = Float64
    ground_truth_marker_positions = Dict{B, Vector{Point3DS{T}}}(b => Vector{Point3DS{T}}() for b in markerbodies)
    measured_marker_positions = Dict{B, Vector{Point3DS{T}}}(b => Vector{Point3DS{T}}() for b in markerbodies)
    for body in markerbodies
        frame = default_frame(body) # TODO: use some other frame to improve test coverage
        measured_marker_inds = randperm(options.num_markers)[1 : options.num_measured_markers]
        for i = 1 : options.num_markers
            ground_truth = Point3D(frame, (rand(SVector{3}) - 0.5) * 2 * options.marker_offset_max)
            push!(ground_truth_marker_positions[body], ground_truth)
            measured = if i ∈ measured_marker_inds
                measurement_error = FreeVector3D(frame, options.marker_measurement_stddev * randn(SVector{3}))
                ground_truth + measurement_error
            else
                Point3D(frame, fill(NaN, SVector{3}))
            end
            push!(measured_marker_positions[body], measured)
        end
    end
    ground_truth_marker_positions, measured_marker_positions
end

function generate_joint_offset(joint::Joint, max_offset::Number)
    (rand(num_positions(joint)) .- 0.5) .* max_offset .* 2
end

function generate_joint_offsets(joints::AbstractVector{<:Joint}, max_offset::Number)
    Dict(j => generate_joint_offset(j, max_offset) for j in joints)
end

@with_kw struct PoseDataGenerationOptions
    num_poses::Int = 25
    motion_capture_noise_stddev::Float64 = 1e-6
    joint_configuration_noise_stddev::Float64 = 1e-4
end

function generate_pose_data(
        state::MechanismState{X, M, C},
        ground_truth_marker_positions::Associative{<:RigidBody{M}, <:AbstractVector{Point3DS{T}}},
        ground_truth_offsets::Associative{<:Joint{M}, <:AbstractVector{T}},
        free_joints::AbstractVector{<:Joint{M}},
        options::PoseDataGenerationOptions = PoseDataGenerationOptions()) where {X, M, C, T}
    ground_truth_pose_data = Vector{PoseData{C}}()
    measured_pose_data = Vector{PoseData{C}}()
    mechanism = state.mechanism
    for i = 1 : options.num_poses
        rand!(state)

        # Joint configurations
        q_ground_truth = copy(configuration(state))
        q_measured = copy(q_ground_truth)
        for (joint, offset) in ground_truth_offsets
            @views q_measured[configuration_range(state, joint)] .+= offset
        end
        for joint in free_joints
            qjoint = view(q_measured, configuration_range(state, joint))
            zero_configuration!(qjoint, joint)
        end
        for joint in setdiff(tree_joints(mechanism), free_joints)
            q_measured[configuration_range(state, joint)] .+= options.joint_configuration_noise_stddev * randn(num_positions(joint))
        end

        # Markers
        S = promote_type(C, T)
        markerbodies = keys(ground_truth_marker_positions)
        ground_truth_marker_data = Dict(b => Vector{Point3DS{S}}() for b in markerbodies)
        measured_marker_data = Dict(b => Vector{Point3DS{S}}() for b in markerbodies)
        for body in markerbodies
            toworld = transform_to_root(state, body)
            for (j, marker_body) in enumerate(ground_truth_marker_positions[body])
                marker_world = transform(marker_body, toworld)
                push!(ground_truth_marker_data[body], marker_world)
                noise = FreeVector3D(marker_world.frame, options.motion_capture_noise_stddev * randn(SVector{3}))
                push!(measured_marker_data[body], marker_world + noise)
                # TODO: add occlusions
            end
        end
        push!(ground_truth_pose_data, PoseData(q_ground_truth, ground_truth_marker_data))
        push!(measured_pose_data, PoseData(q_measured, measured_marker_data))
    end
    ground_truth_pose_data, measured_pose_data
end

