function add_revolute_joint_noise!(q::AbstractVector, state::MechanismState, max_offset::Number)
    for joint in filter(x -> x.jointType isa Revolute, tree_joints(state.mechanism))
        @views q[configuration_range(state, joint)] .+= (rand(num_positions(joint)) .- 0.5) .* max_offset * 2
    end
end

@with_kw struct MarkerPositionGenerationOptions
    marker_measurement_stddev::Float64 = 1e-5
    marker_offset_max::Float64 = 0.1
    num_markers::Int = 4
    num_measured_markers::Int = 3
end

function generate_marker_positions(bodies::AbstractVector{<:RigidBody}, options::MarkerPositionGenerationOptions = MarkerPositionGenerationOptions())
    # Marker positions in body frame
    ground_truth_marker_positions = Dict(b => Vector{Point3DS{T}}() for b in markerbodies)
    measured_marker_positions = Dict(b => Vector{Point3DS{T}}() for b in markerbodies)
    for body in markerbodies
        frame = default_frame(body)
        measured_marker_inds = randperm(options.num_markers)[1 : options.num_measured_markers]
        for i = 1 : options.num_markers
            ground_truth = Point3D(frame, (rand(SVector{3}) - 0.5) * options.marker_offset_max)
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

@with_kw struct PoseDataGenerationOptions
    num_poses::Int = 10
    motion_capture_noise_stddev::Float64 = 1e-6
    revolute_joint_offset_max::Float64 = 1e-2;
end

function generate_pose_data(
        state::MechanismState{X, M, C},
        ground_truth_marker_positions::Dict{<:RigidBody{M}, Vector{Point3DS{T}}},
        options::PoseDataGenerationOptions = PoseDataGenerationOptions()) where {X, M, C, T}
    ground_truth_pose_data = Vector{PoseData}()
    measured_pose_data = Vector{PoseData}()
    for i = 1 : options.num_poses
        rand!(state)

        # Joint configurations
        q_ground_truth = copy(configuration(state))
        q_measured = copy(q_ground_truth)
        add_revolute_joint_noise!(q_measured, state, options.revolute_joint_offset_max)

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

