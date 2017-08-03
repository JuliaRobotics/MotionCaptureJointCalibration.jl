__precompile__()

module MotionCaptureJointCalibration

# types
export
    PoseData,
    CalibrationProblem,
    solve

using StaticArrays
using RigidBodyDynamics
using JuMP
import RigidBodyDynamics: GenericJoint
using MathProgBase: SolverInterface.AbstractMathProgSolver

include("util.jl")

const Point3DS{T} = Point3D{SVector{3, T}}

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

function deconstruct(ordered_marker_bodies::AbstractVector{<:RigidBody}, q::AbstractVector,
        marker_positions_body::Associative{<:RigidBody, <:AbstractVector{<:Point3D}}) # TODO: PoseData? it's in body frame though.
    x = copy(q)
    for body in ordered_marker_bodies
        positions = marker_positions_body[body]
        for j = 1 : length(positions)
            append!(x, positions[j].v)
        end
    end
    x
end

function reconstruct!(ordered_marker_bodies::AbstractVector{<:RigidBody}, q::AbstractVector,
        marker_positions_body::Associative{<:RigidBody, <:AbstractVector{<:Point3D}}, x...)
    index = 1
    for i = 1 : length(q)
        q[i] = x[index]
        index += 1
    end
    for body in ordered_marker_bodies
        positions = marker_positions_body[body]
        frame = default_frame(body)
        for j = 1 : length(positions)
            positions[j] = Point3D(frame, x[index], x[index + 1], x[index + 2])
            index += 3
        end
    end
end

function _marker_residual(state::MechanismState{X, M, C},
        ordered_marker_bodies::AbstractVector{<:RigidBody},
        marker_positions_world::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        marker_positions_body::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        body_weights::Associative{RigidBody{M}, Float64}) where {X, M, C}
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
function _∇marker_residual!(g::AbstractVector{C},
        state::MechanismState{X, M, C},
        ordered_marker_bodies::AbstractVector{<:RigidBody},
        marker_positions_world::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        marker_positions_body::Associative{RigidBody{M}, <:AbstractVector{<:Point3D}},
        body_weights::Associative{RigidBody{M}, Float64}) where {X, M, C}
    nq = num_positions(state)
    ∇residual_q = view(g, 1 : nq) # TODO: allocates
    ∇residual_ps = view(g, nq + 1 : length(g))
    ∇residual_v = zeros(C, num_velocities(state)) # TODO: allocates
    mechanism = state.mechanism
    nv = num_velocities(state)
    p_index = 0
    for body in ordered_marker_bodies
        weight = body_weights[body]
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

function solve(problem::CalibrationProblem{T}, solver::AbstractMathProgSolver) where {T}
    # unpack
    mechanism = problem.mechanism
    calibration_param_bounds = problem.calibration_param_bounds
    marker_bodies = problem.ordered_marker_bodies
    marker_location_measurements = problem.marker_location_measurements
    pose_data = problem.pose_data
    body_weights = problem.body_weights

    state = MechanismState{T}(mechanism)
    calibration_joints = collect(keys(problem.calibration_param_bounds))
    free_joints = collect(keys(problem.free_joint_configuration_bounds))
    num_callibration_params = Dict(j => length(problem.calibration_param_bounds[j]) for j in calibration_joints)
    num_markers = Dict(b => length(markers) for (b, markers) in marker_location_measurements)
    num_poses = length(pose_data)

    m = Model(solver = solver)

    # variables
    calibration_params = Dict(j => @variable(m, [1 : num_callibration_params[j]], basename="c_$(j.name)", start = 0.) for j in calibration_joints)
    configurations = [@variable(m, [1 : num_positions(mechanism)], basename="q$i") for i = 1 : num_poses]
    marker_positions = Dict(b => [Point3D(default_frame(b), @variable(m, [1 : 3], basename="m_$(b.name)_$i")) for i = 1 : num_markers[b]] for b in marker_bodies)
    @variable(m, pose_residuals[1 : num_poses])

    # calibration param constraints
    for joint in calibration_joints
        lower = first.(calibration_param_bounds[joint])
        upper = last.(calibration_param_bounds[joint])
        params = calibration_params[joint]
        @constraint(m, lower .<= params .<= upper)
    end

    # joint configuration constraints and initial values
    for i = 1 : num_poses
        q = configurations[i]
        qmeasured = pose_data[i].configuration

        # initial value
        setvalue.(q, qmeasured)

        # free joints
        for j in free_joints
            if joint_type(j) isa QuaternionFloating
                qj = q[configuration_range(state, j)]
                qrotj = qj[1 : 4] # TODO: method for getting rotation part
                @constraint(m, -1 .<= qj .<= 1) # TODO: have user provide bounds; note: these are essential
                @NLconstraint(m, qrotj[1]^2 + qrotj[2]^2 + qrotj[3]^2 + qrotj[4]^2 == 1) # Unit norm constraint
            end
        end

        # offsets
        for j in calibration_joints
            range = configuration_range(state, j)
            qmeasured_j = qmeasured[range]
            qj = q[range]
            cj = calibration_params[j]
            @constraint(m, qmeasured_j .== qj .+ cj)
        end

        # other joints
        for j in setdiff(tree_joints(mechanism), [free_joints; calibration_joints])
            range = configuration_range(state, j)
            qmeasured_j = qmeasured[range]
            qj = q[range]
            @constraint(m, qj .== qmeasured_j)
        end
    end

    # marker position constraints and initial values
    for body in keys(marker_location_measurements)
        for (position, measured_position) in zip(marker_positions[body], marker_location_measurements[body])
            @framecheck position.frame measured_position.frame
            for (coord, measured_coord) in zip(position.v, measured_position.v)
                if !isnan(measured_coord)
                    @constraint(m, coord == measured_coord)
                    setvalue(coord, measured_coord)
                else
                    @constraint(m, -0.2 <= coord <= 0.2) # TODO: don't hardcode
                end
            end
        end
    end

    # objective: sum of marker residuals
    marker_positions_body = Dict{RigidBody{T}, Vector{Point3DS{T}}}(b => [Point3D(default_frame(b), 0., 0., 0.) for i = 1 : num_markers[b]] for b in marker_bodies)
    for i = 1 : num_poses
        marker_residual = (args::Float64...) -> begin
            reconstruct!(marker_bodies, configuration(state), marker_positions_body, args...)
            setdirty!(state)
            _marker_residual(state, marker_bodies, pose_data[i].marker_positions, marker_positions_body, body_weights)
        end

        ∇marker_residual! = (g, args::Float64...) -> begin
            reconstruct!(marker_bodies, configuration(state), marker_positions_body, args...)
            setdirty!(state)
            _∇marker_residual!(g, state, marker_bodies, pose_data[i].marker_positions, marker_positions_body, body_weights)
        end

        marker_residual_args = deconstruct(marker_bodies, configurations[i], marker_positions)
        num_marker_residual_args = length(marker_residual_args) # actually the same for all poses
        fun = Symbol("marker_residual_", i)
        JuMP.register(m, fun, num_marker_residual_args, marker_residual, ∇marker_residual!, autodiff=false)
        arg_expressions = [:($(marker_residual_args[i])) for i = 1 : num_marker_residual_args]
        JuMP.addNLconstraint(m, :($(pose_residuals[i]) == $(fun)($(arg_expressions...))))
    end
    JuMP.setNLobjective(m, :Min, :(+($(pose_residuals...)) / $num_poses))

    JuMP.solve(m)

    # TODO: do something with results
end

end # module
