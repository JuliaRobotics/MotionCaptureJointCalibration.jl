function _marker_residual(state::MechanismState{X, M, C},
        ordered_marker_bodies::AbstractVector{<:RigidBody},
        marker_positions_world::AbstractDict{RigidBody{M}, <:AbstractVector{<:Point3D}},
        marker_positions_body::AbstractDict{RigidBody{M}, <:AbstractVector{<:Point3D}},
        body_weights::AbstractDict{RigidBody{M}, Float64}) where {X, M, C}
    normalize_configuration!(state)
    residual = zero(C)
    for body in ordered_marker_bodies
        weight = body_weights[body]
        tf = transform_to_root(state, body)
        positions_body = marker_positions_body[body]
        positions_world = marker_positions_world[body]
        for i in eachindex(positions_body, positions_world)
            p = tf * positions_body[i]
            pmeas = positions_world[i]
            residual += weight * mapreduce(x -> zero_nans(x)^2, +, (p - pmeas).v)
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
function _∇marker_residual!(g::AbstractVector{T},
        state::MechanismState{T},
        ordered_marker_bodies::AbstractVector{RigidBody{T}},
        marker_positions_world::AbstractDict{RigidBody{T}, <:AbstractVector{Point3DS{T}}},
        marker_positions_body::AbstractDict{RigidBody{T}, <:AbstractVector{Point3DS{T}}},
        body_weights::AbstractDict{RigidBody{T}, T},
        jacobians::Dict{RigidBody{T}, Pair{TreePath{RigidBody{T}, Joint{T}}, GeometricJacobian{Matrix{T}}}}) where {T}
    normalize_configuration!(state)
    nq = num_positions(state)
    ∇residual_q = SegmentedVector(view(g, 1 : nq), tree_joints(state.mechanism), num_positions) # TODO: allocates
    ∇residual_ps = view(g, nq + 1 : length(g)) # TODO: allocates
    ∇residual_v = zero(velocity(state)) # TODO: allocates
    mechanism = state.mechanism
    nv = num_velocities(state)
    p_index = 0
    for body in ordered_marker_bodies
        weight = body_weights[body]
        tf = transform_to_root(state, body)
        path_to_root, J = jacobians[body]
        geometric_jacobian!(J, state, path_to_root)
        positions_body = marker_positions_body[body]
        positions_world = marker_positions_world[body]
        for i in eachindex(positions_body, positions_world)
            p = tf * positions_body[i]
            pmeas = positions_world[i]
            e = zero_nans.((p - pmeas).v)
            for j = 1 : nv
                ω = SVector(J.angular[1, j], J.angular[2, j], J.angular[3, j])
                v = SVector(J.linear[1, j], J.linear[2, j], J.linear[3, j])
                ∇residual_v[j] += 2 * weight * dot(e, ω × p.v + v)
            end
            ∇residual_p = 2 * weight * e' * rotation(tf)
            ∇residual_ps[p_index += 1] = ∇residual_p[1]
            ∇residual_ps[p_index += 1] = ∇residual_p[2]
            ∇residual_ps[p_index += 1] = ∇residual_p[3]
        end
    end
    configuration_derivative_to_velocity_adjoint!(∇residual_q, state, ∇residual_v)
    g
end
