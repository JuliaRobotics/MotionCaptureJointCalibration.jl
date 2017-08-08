using MotionCaptureJointCalibration
using MotionCaptureJointCalibration.SyntheticDataGeneration
using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using ForwardDiff
using Ipopt
using RigidBodyTreeInspector
using Base.Test

import MotionCaptureJointCalibration: Point3DS, reconstruct!, deconstruct, _marker_residual, _∇marker_residual!

T = Float64

val = Valkyrie()
mechanism = val.mechanism
vis = Visualizer()[:valkyrie]
setgeometry!(vis, mechanism, parse_urdf(ValkyrieRobot.urdfpath(), mechanism; package_path = [ValkyrieRobot.packagepath()]))
remove_fixed_tree_joints!(mechanism)
state = MechanismState{T}(mechanism)

foot = findbody(mechanism, "leftFoot")
pelvis = findbody(mechanism, "pelvis")
markerbodies = [pelvis; foot]
scales = Dict(pelvis => 5., foot => 1.) # be careful with these; having them be different orders of magnitude can lead to numerical issues
p = path(mechanism, pelvis, foot)
correction_joints = collect(p)
calibration_param_bounds = Dict(j => [(-0.05, 0.05)] for j in correction_joints)
free_joint_configuration_bounds = Dict{GenericJoint{T}, Vector{Tuple{Float64, Float64}}}()
for joint in filter(isfloating, tree_joints(mechanism))
    free_joint_configuration_bounds[joint] = fill((-1., 1.), num_positions(joint))
end
free_joints = collect(keys(free_joint_configuration_bounds))

srand(1)
marker_options = MarkerPositionGenerationOptions()
pose_options = PoseDataGenerationOptions()
ground_truth_marker_positions, measured_marker_positions = generate_marker_positions(markerbodies, marker_options)
ground_truth_offsets = Dict(j => generate_joint_offset(j, 1e-2) for j in correction_joints)
ground_truth_pose_data, measured_pose_data = generate_pose_data(state, ground_truth_marker_positions, ground_truth_offsets, free_joints, pose_options)

@testset "deconstruct/reconstruct!" begin
    q = zeros(num_positions(mechanism))
    num_markers = Dict(b => length(markers) for (b, markers) in measured_marker_positions)
    marker_positions_body = Dict{RigidBody{T}, Vector{Point3DS{T}}}(b => [Point3D(default_frame(b), 0., 0., 0.) for i = 1 : num_markers[b]] for b in markerbodies)
    reconstruct!(markerbodies, q, marker_positions_body, deconstruct(markerbodies, configuration(state), ground_truth_marker_positions)...)
    @test all(q .== configuration(state))
    for (body, positions) in ground_truth_marker_positions
        @test all(marker_positions_body[body] .== positions)
    end
end

@testset "marker_residual gradient" begin
    data = measured_pose_data[1]
    marker_positions_world = data.marker_positions

    function marker_residual_inefficient(x::X...) where {X}
        M = eltype(mechanism)
        state = MechanismState{X}(mechanism)
        marker_positions_body = Dict{RigidBody{M}, Vector{Point3DS{X}}}(
            b => [Point3D(default_frame(b), zero(X), zero(X), zero(X)) for i = 1 : length(measured_marker_positions[b])] for b in markerbodies
        )
        reconstruct!(markerbodies, configuration(state), marker_positions_body, x...)
        setdirty!(state)
        _marker_residual(state, markerbodies, marker_positions_world, marker_positions_body, scales)
    end

    f(args) = [marker_residual_inefficient(args...)]

    for i = 1 : 100
        q = rand(num_positions(mechanism)) # needs to work for non-normalized quaternions as well
        Jcheck = ForwardDiff.jacobian(f, deconstruct(markerbodies, q, ground_truth_marker_positions))

        set_configuration!(state, q)
        g = zeros(length(Jcheck))
        paths_to_root = Dict(b => path(mechanism, root_body(mechanism), b) for b in markerbodies)
        jacobians = Dict(b => (p => geometric_jacobian(state, p)) for (b, p) in paths_to_root)
        _∇marker_residual!(g, state, markerbodies, marker_positions_world, ground_truth_marker_positions, scales, jacobians)
        J = g'

        @test isapprox(J, Jcheck, atol = 1e-14)
    end
end

@testset "problem" begin
    problem = CalibrationProblem(mechanism, calibration_param_bounds, free_joint_configuration_bounds, measured_marker_positions, measured_pose_data)
    @test num_poses(problem) == pose_options.num_poses
    @test num_calibration_params(problem) == length(correction_joints)
    @test num_markers(problem) == marker_options.num_markers * length(markerbodies)
    @test num_bodies(problem) == length(markerbodies)
    show(DevNull, problem)
end

@testset "solve" begin
    # NLopt SLSQP works well with up to 10 poses, free floating joint configurations and two unmeasured markers
    # solver = NLoptSolver(algorithm = :LD_SLSQP)

    solver = IpoptSolver(print_level = 4, max_iter = 10000, derivative_test = "first-order", check_derivatives_for_naninf = "yes", tol = 1e-10)
    # other useful options:
    # hessian_approximation = "limited-memory"

    problem = CalibrationProblem(mechanism, calibration_param_bounds, free_joint_configuration_bounds, measured_marker_positions, measured_pose_data)
    result = solve(problem, solver)
    @test result.status == :Optimal

    @test num_poses(problem) == num_poses(result)
    @test num_calibration_params(problem) == num_calibration_params(result)
    @test num_markers(problem) == num_markers(result)
    @test num_bodies(problem) == num_bodies(result)

    # check calibration parameters
    calibration_joints = keys(calibration_param_bounds)
    for joint in calibration_joints
        sol = result.calibration_params[joint]
        truth = ground_truth_offsets[joint]
        @test isapprox(sol, truth; atol = 1e-3)
    end

    # check configurations
    solution_state = MechanismState{T}(mechanism)
    ground_truth_state = MechanismState{T}(mechanism)
    for i = 1 : num_poses(problem)
        set_configuration!(solution_state, result.configurations[i])
        set_configuration!(ground_truth_state, ground_truth_pose_data[i].configuration)
        for body in bodies(mechanism)
            @test isapprox(transform_to_root(solution_state, body), transform_to_root(ground_truth_state, body); atol = 2e-3)
        end
    end

    # make sure printing doesn't error
    show(result)

    # visualization (just to make sure the code doesn't error)
    println()
    inspect!(state, vis, problem, result)
    nothing
end
