using MotionCaptureJointCalibration
using DrakeVisualizer
using RigidBodyTreeInspector
using Interact

using MotionCaptureJointCalibration.SyntheticDataGeneration

using RigidBodyDynamics
using StaticArrays
using ValkyrieRobot
using ForwardDiff
using Ipopt
using NBInclude
using Test
using LinearAlgebra

using MotionCaptureJointCalibration: Point3DS, reconstruct!, deconstruct, _marker_residual, _∇marker_residual!
using Random: seed!, rand!

T = Float64
val = Valkyrie()
mechanism = val.mechanism
remove_fixed_tree_joints!(mechanism)
state = MechanismState{T}(mechanism)

markerbodies = findbody.(Ref(mechanism), ["leftFoot", "pelvis"])
seed!(1)
body_weights = Dict(b => rand() for b in markerbodies)
marker_options = MarkerPositionGenerationOptions()
pose_options = PoseDataGenerationOptions()
problem, groundtruth = generate_calibration_problem(state, body_weights, marker_options = marker_options, pose_options = pose_options)

@testset "deconstruct/reconstruct!" begin
    q = zeros(num_positions(mechanism))
    marker_positions_body = Dict(b => [Point3D(default_frame(b), 0., 0., 0.) for i = 1 : num_markers(problem, b)] for b in markerbodies)
    reconstruct!(markerbodies, q, marker_positions_body, deconstruct(markerbodies, configuration(state), groundtruth.marker_positions)...)
    @test all(q .== configuration(state))
    for (body, positions) in groundtruth.marker_positions
        @test all(marker_positions_body[body] .== positions)
    end
end

@testset "marker_residual gradient" begin
    data = problem.pose_data[1]
    marker_positions_world = data.marker_positions

    function marker_residual_inefficient(x::X...) where {X}
        M = eltype(mechanism)
        state = MechanismState{X}(mechanism)
        marker_positions_body = Dict{RigidBody{M}, Vector{Point3DS{X}}}(
            b => [Point3D(default_frame(b), zero(X), zero(X), zero(X)) for i = 1 : num_markers(problem, b)] for b in markerbodies
        )
        reconstruct!(markerbodies, configuration(state), marker_positions_body, x...)
        normalize_configuration!(state)
        setdirty!(state)
        _marker_residual(state, markerbodies, marker_positions_world, marker_positions_body, body_weights)
    end

    f(args) = [marker_residual_inefficient(args...)]

    for i = 1 : 100
        rand!(state)
        q = configuration(state)
        Jcheck = ForwardDiff.jacobian(f, deconstruct(markerbodies, q, groundtruth.marker_positions))

        set_configuration!(state, q)
        g = zeros(length(Jcheck))
        paths_to_root = Dict(b => RigidBodyDynamics.path(mechanism, root_body(mechanism), b) for b in markerbodies)
        jacobians = Dict(b => (p => geometric_jacobian(state, p)) for (b, p) in paths_to_root)
        _∇marker_residual!(g, state, markerbodies, marker_positions_world, groundtruth.marker_positions, body_weights, jacobians)
        J = g'

        @test isapprox(J, Jcheck, atol = 1e-14)
    end
end

@testset "problem" begin
    @test num_poses(problem) == pose_options.num_poses
    @test num_calibration_params(problem) == 6
    @test num_markers(problem) == marker_options.num_markers * length(markerbodies)
    @test num_bodies(problem) == length(markerbodies)
    show(devnull, problem)
end

@testset "solve" begin
    # NLopt SLSQP works well with up to 10 poses, free floating joint configurations and two unmeasured markers
    # solver = NLoptSolver(algorithm = :LD_SLSQP)

    solver = IpoptSolver(print_level = 4, max_iter = 10000, derivative_test = "first-order", check_derivatives_for_naninf = "yes", tol = 1e-10)
    # other useful options:
    # hessian_approximation = "limited-memory"

    result = solve(problem, solver)
    @test result.status == :Optimal

    @test num_poses(problem) == num_poses(result)
    @test num_calibration_params(problem) == num_calibration_params(result)
    @test num_markers(problem) == num_markers(result)
    @test num_bodies(problem) == num_bodies(result)

    # check calibration parameters
    for joint in calibration_joints(problem)
        @test isapprox(result.calibration_params[joint], groundtruth.calibration_params[joint]; atol = 1e-3)
    end

    # check configurations
    solutionstate = MechanismState{T}(mechanism)
    groundtruthstate = MechanismState{T}(mechanism)
    for i = 1 : num_poses(problem)
        set_configuration!(solutionstate, result.configurations[i])
        set_configuration!(groundtruthstate, groundtruth.configurations[i])
        for body in bodies(mechanism)
            @test isapprox(transform_to_root(solutionstate, body), transform_to_root(groundtruthstate, body); atol = 2e-3)
        end
    end

    # printing and visualization (just to make sure the code doesn't error)
    show(devnull, result)

    vis = Visualizer()[:valkyrie]
    geometry = visual_elements(mechanism, URDFVisuals(ValkyrieRobot.urdfpath(); package_path = [ValkyrieRobot.packagepath()]))
    setgeometry!(vis, mechanism, geometry)
    inspect!(state, vis, problem, result)
end

using RigidBodyTreeInspector

@testset "example notebooks" begin
    notebookdir = joinpath(@__DIR__, "..", "notebooks")
    excludedirs = [".ipynb_checkpoints"]
    excludefiles = String[]
    for (root, dir, files) in walkdir(notebookdir)
        basename(root) in excludedirs && continue
        for file in files
            file in excludefiles && continue
            name, ext = splitext(file)
            lowercase(ext) == ".ipynb" || continue
            path = joinpath(root, file)
            @eval module $(gensym()) # Each notebook is run in its own module.
            using Test
            using NBInclude
            @testset "Notebook: $($name)" begin
                # Note: use #NBSKIP in a cell to skip it during tests.
                @nbinclude($path; regex = r"^((?!\#NBSKIP).)*$"s)
            end
            end # module
        end
    end
end
