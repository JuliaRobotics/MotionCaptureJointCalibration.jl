struct CalibrationResult{T}
    status::Symbol
    residual::T
    calibration_params::Dict{GenericJoint{T}, Vector{T}}
    configurations::Vector{Vector{T}}
    marker_positions::Dict{RigidBody{T}, Vector{Point3DS{T}}}
end

function Base.show(io::IO, result::CalibrationResult{T}) where {T}
    println(io, "CalibrationResult{$T}: $(string(result.status)), residual = $(result.residual). Calibration parameters:")
    num_joints = length(result.calibration_params)
    n = 0
    for (joint, params) in result.calibration_params
        print(io, "$(joint.name): $params")
        (n += 1) < num_joints && println()
    end
end