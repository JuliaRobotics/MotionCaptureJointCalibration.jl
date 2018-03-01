struct PoseData{T}
    configuration::TreeJointSegmentedVector{T}
    marker_positions::Dict{RigidBody{T}, Vector{Point3DS{T}}}
end

mutable struct CalibrationProblem{T}
    mechanism::Mechanism{T}
    calibration_param_bounds::Dict{<:Joint{T}, Vector{Tuple{T, T}}}
    free_joint_configuration_bounds::Dict{<:Joint{T}, Vector{Tuple{T, T}}}
    marker_location_bounds::Dict{RigidBody{T}, Vector{Tuple{Point3DS{T}, Point3DS{T}}}}
    pose_data::Vector{PoseData{T}}
    body_weights::Dict{RigidBody{T}, T}
    ordered_marker_bodies::Vector{RigidBody{T}}

    function CalibrationProblem(
            mechanism::Mechanism{T},
            calibration_param_bounds::Dict{<:Joint{T}, Vector{Tuple{T, T}}},
            free_joint_configuration_bounds::Dict{<:Joint{T}, Vector{Tuple{T, T}}},
            marker_location_bounds::Dict{RigidBody{T}, Vector{Tuple{Point3DS{T}, Point3DS{T}}}},
            pose_data::Vector{PoseData{T}},
            body_weights::Dict{RigidBody{T}, T} = Dict(b => one(T) for b in keys(marker_location_bounds))) where {T}
        @assert isempty(symdiff(keys(marker_location_bounds), keys(body_weights)))
        canonicalize!(marker_location_bounds)
        ordered_marker_bodies = intersect(bodies(mechanism), keys(body_weights))
        new{T}(mechanism, calibration_param_bounds, free_joint_configuration_bounds,
            marker_location_bounds, pose_data, body_weights, ordered_marker_bodies)
    end
end

num_poses(problem::CalibrationProblem) = length(problem.pose_data)
num_calibration_params(problem::CalibrationProblem) = sum(length, values(problem.calibration_param_bounds))
num_calibration_params(problem::CalibrationProblem, joint::Joint) = length(problem.calibration_param_bounds[joint])
num_markers(problem::CalibrationProblem) = sum(length, values(problem.marker_location_bounds))
num_markers(problem::CalibrationProblem, body::RigidBody) = length(problem.marker_location_bounds[body])
RigidBodyDynamics.num_bodies(problem::CalibrationProblem) = length(problem.body_weights)
calibration_joints(problem::CalibrationProblem) = keys(problem.calibration_param_bounds)
free_joints(problem::CalibrationProblem) = keys(problem.free_joint_configuration_bounds)

function Base.show(io::IO, problem::CalibrationProblem{T}) where {T}
    msg = """CalibrationProblem{$T} with:
    * $(num_poses(problem)) poses
    * $(num_calibration_params(problem)) calibration parameters
    * $(num_markers(problem)) markers attached to $(num_bodies(problem)) bodies
    """
    println(io, msg)
end
