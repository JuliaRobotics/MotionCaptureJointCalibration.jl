__precompile__()

module MotionCaptureJointCalibration

# types
export
    PoseData

using StaticArrays
import DataStructures: OrderedDict
using RigidBodyDynamics
import RigidBodyDynamics: GenericJoint

include("util.jl")

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

function _marker_residual(
        state::MechanismState{X, M, C},
        marker_positions_world::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        marker_positions_body::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        scales::Associative{RigidBody{M}, Float64}) where {X, M, C}
    residual = zero(C)
    for (body, scale) in scales
        tf = transform_to_root(state, body)
        positions_body = marker_positions_body[body]
        positions_world = marker_positions_world[body]
        for i in eachindex(positions_body, positions_world)
            p = tf * positions_body[i]
            pmeas = positions_world[i]
            residual += scale * mapreduce(x -> zero_nans(x)^2, +, (p - pmeas).v)
        end
    end
    residual
end

# NOTE: ∇residual_v maps the joint velocity vector to the time derivative of the marker residual, rd:
#
#   rd = <∇residual_v , v>
#
# The desired gradient g is the mapping from the time derivative of the joint configuration vector to
# the time derivative of the marker residual:
#
#   rd = <g , qd>
#
# qd and v are related by an invertible linear map (see e.g. Port-based modeling and control for
# efficient bipedal walking robots, Definition 2.9):
#
#   v = v_Q qd
#
# so we have
#
#   rd = <∇residual_v , v> = <∇residual_v , v_Q qd> = <v_Qᵀ ∇residual_v , qd>
#
# which shows that g = v_Qᵀ ∇residual_v
function _∇marker_residual!(g::AbstractVector{C},
        state::MechanismState{X, M, C},
        marker_positions_world::OrderedDict{RigidBody{M}, <:AbstractVector{<:Point3D}},
        marker_positions_body::OrderedDict{RigidBody{M}, <:AbstractVector{<:Point3D}},
        scales::Associative{RigidBody{M}, Float64}) where {X, M, C}
    nq = num_positions(state)
    ∇residual_q = view(g, 1 : nq) # TODO: allocates
    ∇residual_ps = view(g, nq + 1 : length(g))
    ∇residual_v = zeros(C, num_velocities(state)) # TODO: allocates
    mechanism = state.mechanism
    nv = num_velocities(state)
    p_index = 0
    for body in keys(marker_positions_body)
        scale = scales[body]
        tf = transform_to_root(state, body)
        path_to_root = path(mechanism, root_body(mechanism), body) #TODO: allocates
        J = geometric_jacobian(state, path_to_root) # TODO: allocates
        positions_body = marker_positions_body[body]
        positions_world = marker_positions_world[body]
        for i in eachindex(positions_body, positions_world)
            p = tf * positions_body[i]
            pmeas = positions_world[i]
            e = zero_nans.((p - pmeas).v)
            for j = 1 : nv
                ω = SVector(J.angular[1, j], J.angular[2, j], J.angular[3, j])
                v = SVector(J.linear[1, j], J.linear[2, j], J.linear[3, j])
                ∇residual_v[j] += 2 * scale * dot(e, ω × p.v + v)
            end
            ∇residual_p = 2 * scale * e' * rotation(tf)
            ∇residual_ps[p_index += 1] = ∇residual_p[1]
            ∇residual_ps[p_index += 1] = ∇residual_p[2]
            ∇residual_ps[p_index += 1] = ∇residual_p[3]
        end
    end
    configuration_derivative_to_velocity_adjoint!(∇residual_q, state, ∇residual_v)
    g
end

end # module
