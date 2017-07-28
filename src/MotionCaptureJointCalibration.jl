__precompile__()

module MotionCaptureJointCalibration

# types
export
    MarkerGroup,
    PoseData

using StaticArrays
using RigidBodyDynamics
using RigidBodyTreeInspector
import RigidBodyDynamics: GenericJoint

const Point3DS{T} = Point3D{SVector{3, T}}

struct PoseData
    configuration::Vector{Float64}
    marker_positions::Dict{RigidBody{Float64}, Vector{Point3DS{Float64}}}
end

struct MarkerGroup
    num_markers::Int
    scale::Float64
end

struct CalibrationProblem{M<:Number, C}
    mechanism::Mechanism{M}
    joints::Vector{GenericJoint{M}}
    correctionfun::C
    markergroups::Dict{RigidBody{M}, MarkerGroup}
end

end # module
