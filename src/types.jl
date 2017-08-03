struct PoseData{T}
    configuration::Vector{T}
    marker_positions::Dict{RigidBody{T}, Vector{Point3DS{T}}}
end

type CalibrationProblem{T}
    mechanism::Mechanism{T}
    calibration_param_bounds::Dict{GenericJoint{T}, Vector{Tuple{T, T}}}
    free_joint_configuration_bounds::Dict{GenericJoint{T}, Vector{Tuple{T, T}}}
    marker_location_measurements::Dict{RigidBody{T}, Vector{Point3DS{T}}}
    pose_data::Vector{PoseData{T}}
    body_weights::Dict{RigidBody{T}, T}
    ordered_marker_bodies::Vector{RigidBody{T}}

    function CalibrationProblem(
            mechanism::Mechanism{T},
            calibration_param_bounds::Dict{GenericJoint{T}, Vector{Tuple{T, T}}},
            free_joint_configuration_bounds::Dict{GenericJoint{T}, Vector{Tuple{T, T}}},
            marker_location_measurements::Dict{RigidBody{T}, Vector{Point3DS{T}}},
            pose_data::Vector{PoseData{T}},
            body_weights::Dict{RigidBody{T}, T} = Dict(b => one(T) for b in keys(marker_location_measurements))) where {T}
        @assert isempty(symdiff(keys(marker_location_measurements), keys(body_weights)))
        canonicalize_body_fixed_points!(marker_location_measurements)
        ordered_marker_bodies = intersect(bodies(mechanism), keys(body_weights))
        new{T}(mechanism, calibration_param_bounds, free_joint_configuration_bounds,
            marker_location_measurements, pose_data, body_weights, ordered_marker_bodies)
    end
end
