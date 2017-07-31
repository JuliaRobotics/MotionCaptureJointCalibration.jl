__precompile__()

module MotionCaptureJointCalibration

# types
export
    PoseData

using StaticArrays
import DataStructures: OrderedDict
using RigidBodyDynamics
import RigidBodyDynamics: GenericJoint

const Point3DS{T} = Point3D{SVector{3, T}}

struct PoseData
    configuration::Vector{Float64}
    marker_positions::OrderedDict{RigidBody{Float64}, Vector{Point3DS{Float64}}}
end

function deconstruct(
    q::AbstractVector,
    marker_positions_body::OrderedDict{<:RigidBody, <:AbstractVector{<:Point3D}})
    x = copy(q)
    for (body, positions) in marker_positions_body
        for j = 1 : length(positions)
            append!(x, positions[j].v)
        end
    end
    x
end

function reconstruct!(q::AbstractVector, marker_positions_body::OrderedDict{<:RigidBody, <:AbstractVector{<:Point3D}}, x...)
    index = 1
    for i = 1 : length(q)
        q[i] = x[index]
        index += 1
    end
    for (body, positions) in marker_positions_body
        frame = default_frame(body)
        for j = 1 : length(positions)
            positions[j] = Point3D(frame, x[index], x[index + 1], x[index + 2])
            index += 3
        end
    end
end

end # module
