function solve(problem::CalibrationProblem{T}, solver::AbstractMathProgSolver) where {T}
    # unpack
    mechanism = problem.mechanism
    calibration_param_bounds = problem.calibration_param_bounds
    marker_bodies = problem.ordered_marker_bodies
    marker_location_measurements = problem.marker_location_measurements
    free_joint_configuration_bounds = problem.free_joint_configuration_bounds
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
    for (joint, bounds) in calibration_param_bounds
        lower = first.(bounds)
        upper = last.(bounds)
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
        for (joint, bounds) in free_joint_configuration_bounds # note: bounds on joint configurations appear to be essential for Ipopt
            qjoint = q[configuration_range(state, joint)]
            lower = first.(bounds)
            upper = last.(bounds)
            if joint_type(joint) isa QuaternionFloating
                qrot = qjoint[1 : 4] # TODO: method for getting rotation part
                rotlower = max.(lower[1 : 4], -1)
                rotupper = min.(upper[1 : 4], 1)
                @constraint(m, rotlower .<= qrot .<= rotupper)
                @NLconstraint(m, qrot[1]^2 + qrot[2]^2 + qrot[3]^2 + qrot[4]^2 == 1) # Unit norm constraint
                qtrans = qjoint[5 : 7]
                translower = lower[5 : 7]
                transupper = upper[5 : 7]
                @constraint(m, translower .<= qtrans .<= transupper)
            else
                @constraint(m, lower .<= qjoint .<= upper)
            end
        end

        # offsets
        for joint in calibration_joints
            range = configuration_range(state, joint)
            qmeasured_joint = qmeasured[range]
            qjoint = q[range]
            cjoint = calibration_params[joint]
            @constraint(m, qmeasured_joint .== qjoint .+ cjoint)
        end

        # other joints
        for joint in setdiff(tree_joints(mechanism), [free_joints; calibration_joints])
            range = configuration_range(state, joint)
            qmeasured_joint = qmeasured[range]
            qjoint = q[range]
            @constraint(m, qjoint .== qmeasured_joint)
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

    status = JuMP.solve(m)

    calibration_params_sol = Dict(j => getvalue.(c) for (j, c) in calibration_params)
    configurations_sol = getvalue.(configurations)
    marker_positions_sol = Dict(b => (p -> Point3D(p.frame, SVector{3}(getvalue.(p.v)))).(positions) for (b, positions) in marker_positions)

    CalibrationResult(status, getobjectivevalue(m), calibration_params_sol, configurations_sol, marker_positions_sol)
end
