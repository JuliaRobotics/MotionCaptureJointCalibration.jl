struct CalibrationResult{T}
    status::Symbol
    residual::T
    calibration_params::Dict{<:Joint{T}, Vector{T}}
    configurations::Vector{TreeJointSegmentedVector{T}}
    marker_positions::Dict{RigidBody{T}, Vector{Point3DS{T}}}
end

num_poses(result::CalibrationResult) = length(result.configurations)
num_calibration_params(result::CalibrationResult) = sum(length, values(result.calibration_params))
num_markers(result::CalibrationResult) = sum(length, values(result.marker_positions))
RigidBodyDynamics.num_bodies(result::CalibrationResult) = length(result.marker_positions)

function Base.show(io::IO, result::CalibrationResult{T}) where {T}
    println(io, "CalibrationResult{$T}: $(string(result.status)), residual = $(result.residual). Calibration parameters:")
    num_joints = length(result.calibration_params)
    n = 0
    for (joint, params) in result.calibration_params
        print(io, "$(joint.name): $params")
        (n += 1) < num_joints && println(io)
    end
end
