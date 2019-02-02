function solve(problem::CalibrationProblem{T}, solver::AbstractMathProgSolver) where {T}
    # unpack
    mechanism = problem.mechanism
    calibration_param_bounds = problem.calibration_param_bounds
    marker_bodies = problem.ordered_marker_bodies
    marker_location_bounds = problem.marker_location_bounds
    free_joint_configuration_bounds = problem.free_joint_configuration_bounds
    pose_data = problem.pose_data
    body_weights = problem.body_weights
    num_poses = MotionCaptureJointCalibration.num_poses(problem)

    state = MechanismState{T}(mechanism)
    m = Model(solver = solver)

    # variables
    calibration_params = Dict(j => @variable(m, [1 : num_calibration_params(problem, j)], basename="c_$(j.name)")
        for j in calibration_joints(problem))
    configurations = [copyto!(similar(configuration(state), JuMP.Variable),
        @variable(m, [1 : num_positions(mechanism)], basename="q$i")) for i = 1 : num_poses]
    marker_positions = Dict(b => [Point3D(default_frame(b), @variable(m, [1 : 3], basename="m_$(b.name)_$i"))
        for i = 1 : num_markers(problem, b)] for b in marker_bodies)
    @variable(m, pose_residuals[1 : num_poses])

    # calibration param constraints
    for (joint, bounds) in calibration_param_bounds
        params = calibration_params[joint]
        setlowerbound.(params, first.(bounds))
        setupperbound.(params, last.(bounds))
        setvalue.(params, 0)
    end

    # joint configuration constraints and initial values
    for i = 1 : num_poses
        q = configurations[i]
        qmeasured = pose_data[i].configuration
        setvalue.(q, qmeasured)
        for joint in tree_joints(mechanism) # to fix the order
            qjoint = q[joint]
            if joint ∈ free_joints(problem)
                # free joint configuration bounds
                bounds = free_joint_configuration_bounds[joint]
                lower = first.(bounds)
                upper = last.(bounds)
                if joint_type(joint) isa QuaternionFloating
                    # TODO: method for getting rotation part
                    qrot = qjoint[1 : 4]
                    @NLconstraint(m, qrot[1]^2 + qrot[2]^2 + qrot[3]^2 + qrot[4]^2 == 1) # Unit norm constraint
                    lower[1 : 4] .= max.(lower[1 : 4], -1)
                    upper[1 : 4] .= min.(upper[1 : 4], +1)
                end
                setlowerbound.(qjoint, lower)
                setupperbound.(qjoint, upper)
            elseif joint ∈ calibration_joints(problem)
                # calibration joint model
                # TODO: generalize to handle not just offsets but also other models
                # TODO: add redundant bounds on qjoint?
                qjoint_measured = qmeasured[joint]
                cjoint = calibration_params[joint]
                @constraint(m, qjoint_measured .== qjoint .+ cjoint)
            else
                # other joints: fix at measured position
                JuMP.fix.(qjoint, qmeasured[joint])
            end
        end
    end

    # marker position constraints and initial values
    for body in keys(marker_location_bounds)
        for (position, bounds) in zip(marker_positions[body], marker_location_bounds[body])
            lower, upper = bounds
            @framecheck position.frame lower.frame
            @framecheck position.frame upper.frame
            for (coord, lower_coord, upper_coord) in zip(position.v, lower.v, upper.v)
                if lower_coord == upper_coord
                    JuMP.fix(coord, lower_coord)
                else
                    setlowerbound(coord, lower_coord)
                    setupperbound(coord, upper_coord)
                end
            end
        end
    end

    # objective: sum of marker residuals
    marker_positions_body = Dict(
        b => [Point3D(default_frame(b), 0., 0., 0.) for i = 1 : num_markers(problem, b)] for b in marker_bodies)
    paths_to_root = Dict(b => RigidBodyDynamics.path(mechanism, root_body(mechanism), b) for b in marker_bodies)
    jacobians = Dict(b => (p => geometric_jacobian(state, p)) for (b, p) in paths_to_root)
    for i = 1 : num_poses
        marker_residual = (args::Float64...) -> begin
            reconstruct!(marker_bodies, configuration(state), marker_positions_body, args...)
            normalize_configuration!(state)
            setdirty!(state)
            _marker_residual(state, marker_bodies, pose_data[i].marker_positions, marker_positions_body, body_weights)
        end

        ∇marker_residual! = (g, args::Float64...) -> begin
            reconstruct!(marker_bodies, configuration(state), marker_positions_body, args...)
            normalize_configuration!(state)
            setdirty!(state)
            _∇marker_residual!(g, state, marker_bodies, pose_data[i].marker_positions, marker_positions_body, body_weights, jacobians)
        end

        marker_residual_args = deconstruct(marker_bodies, configurations[i], marker_positions)
        num_marker_residual_args = length(marker_residual_args) # actually the same for all poses
        fun = Symbol("marker_residual_", i)
        JuMP.register(m, fun, num_marker_residual_args, marker_residual, ∇marker_residual!, autodiff = false)
        arg_expressions = [:($(marker_residual_args[i])) for i = 1 : num_marker_residual_args]
        JuMP.addNLconstraint(m, :($(pose_residuals[i]) == $(fun)($(arg_expressions...))))
    end
    @NLobjective(m, Min, sum(pose_residuals[i] for i = 1 : num_poses) / num_poses)
    @objective m Min 0

    status = JuMP.solve(m)

    calibration_params_sol = Dict(j => getvalue.(c) for (j, c) in calibration_params)
    configurations_sol = [copyto!(similar(configuration, T), getvalue.(configuration)) for configuration in configurations]
    marker_positions_sol = Dict(b => (p -> Point3D(p.frame, SVector{3}(
            getvalue.(p.v)))).(positions) for (b, positions) in marker_positions)

    CalibrationResult(status, getobjectivevalue(m), calibration_params_sol, configurations_sol, marker_positions_sol)
end
