module SyntheticDataGeneration

export
    MarkerPositionGenerationOptions,
    PoseDataGenerationOptions,
    generate_marker_positions,
    generate_joint_offset,
    generate_pose_data,
    generate_calibration_problem

using MotionCaptureJointCalibration
using RigidBodyDynamics
using StaticArrays
using Parameters
using Base.Iterators

using Random: randperm, rand!
using MotionCaptureJointCalibration: Point3DS

@with_kw struct MarkerPositionGenerationOptions
    marker_measurement_stddev::Float64 = 1e-5
    marker_offset_max::Float64 = 0.1
    num_markers::Int = 4
    num_measured_markers::Int = 3
end

@with_kw struct PoseDataGenerationOptions
    num_poses::Int = 25
    motion_capture_noise_stddev::Float64 = 1e-6
    joint_configuration_noise_stddev::Float64 = 1e-4
    occlusion_probability::Float64 = 0.025
end

function generate_marker_positions(bodies::AbstractVector{<:RigidBody},
        options::MarkerPositionGenerationOptions = MarkerPositionGenerationOptions())
    # Marker positions in body frame
    B = eltype(bodies)
    T = Float64
    ground_truth_marker_positions = Dict(b => Vector{Point3DS{T}}() for b in bodies)
    measured_marker_positions = Dict(b => Vector{Tuple{Point3DS{T}, Point3DS{T}}}() for b in bodies)
    for body in bodies
        frame = default_frame(body) # TODO: use some other frame to improve test coverage
        measured_marker_inds = randperm(options.num_markers)[1 : options.num_measured_markers]
        for i = 1 : options.num_markers
            ground_truth = Point3D(frame, (rand(SVector{3}) - 0.5) * 2 * options.marker_offset_max)
            push!(ground_truth_marker_positions[body], ground_truth)
            bounds = if i ∈ measured_marker_inds
                measurement_error = FreeVector3D(frame, options.marker_measurement_stddev * randn(SVector{3}))
                measured = ground_truth + measurement_error
                measured, measured
            else
                lower = Point3D(frame, fill(-0.2, SVector{3}))
                upper = Point3D(frame, fill(0.2, SVector{3}))
                lower, upper
            end
            push!(measured_marker_positions[body], bounds)
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

function generate_pose_data(
        state::MechanismState{X, M, C},
        ground_truth_marker_positions::AbstractDict{<:RigidBody{M}, <:AbstractVector{Point3DS{T}}},
        ground_truth_offsets::AbstractDict{<:Joint{M}, <:AbstractVector{T}},
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
            qjoint = q_measured[joint]
            qjoint .+= offset
        end
        for joint in free_joints
            qjoint = q_measured[joint]
            zero_configuration!(qjoint, joint)
        end
        for joint in setdiff(tree_joints(mechanism), free_joints)
            qjoint = q_measured[joint]
            qjoint .+= options.joint_configuration_noise_stddev * randn(num_positions(joint))
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
                occluded = rand() < options.occlusion_probability
                measured = if occluded
                    Point3D(marker_world.frame, fill(S(NaN), SVector{3}))
                else
                    noise = FreeVector3D(marker_world.frame, options.motion_capture_noise_stddev * randn(SVector{3}))
                    marker_world + noise
                end
                push!(measured_marker_data[body], measured)
            end
        end
        push!(ground_truth_pose_data, PoseData(q_ground_truth, ground_truth_marker_data))
        push!(measured_pose_data, PoseData(q_measured, measured_marker_data))
    end
    ground_truth_pose_data, measured_pose_data
end

function generate_calibration_problem(state::MechanismState{T}, body_weights::Dict{RigidBody{T}, T};
        marker_options::MarkerPositionGenerationOptions = MarkerPositionGenerationOptions(),
        pose_options::PoseDataGenerationOptions = PoseDataGenerationOptions()) where {T}
    bodies = collect(keys(body_weights))
    mechanism = state.mechanism
    correction_joints = unique(flatten([collect(RigidBodyDynamics.path(mechanism, body1, body2)) for (body1, body2) in product(bodies, bodies)]))
    calibration_param_bounds = Dict{Joint{T}, Vector{Tuple{T, T}}}(j => fill((-0.05, 0.05), num_positions(j)) for j in correction_joints)
    free_joint_configuration_bounds = Dict{Joint{T}, Vector{Tuple{T, T}}}(
        j => fill((-1., 1.), num_positions(j)) for j in tree_joints(mechanism) if isfloating(j))
    free_joints = collect(keys(free_joint_configuration_bounds))
    ground_truth_marker_positions, measured_marker_positions = generate_marker_positions(bodies, marker_options)
    ground_truth_offsets = Dict{Joint{T}, Vector{T}}(j => generate_joint_offset(j, 1e-2) for j in correction_joints)
    ground_truth_pose_data, measured_pose_data =
        generate_pose_data(state, ground_truth_marker_positions, ground_truth_offsets, free_joints, pose_options)
    problem = CalibrationProblem(
        mechanism,
        calibration_param_bounds,
        free_joint_configuration_bounds,
        measured_marker_positions,
        measured_pose_data, body_weights)
    configurations = [data.configuration for data in ground_truth_pose_data]
    ground_truth = CalibrationResult(:Optimal, 0., ground_truth_offsets, configurations, ground_truth_marker_positions)
    problem, ground_truth
end

end # module
