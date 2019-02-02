module MotionCaptureJointCalibration

export
    PoseData,
    CalibrationProblem,
    CalibrationResult,
    solve,
    calibration_joints,
    free_joints,
    num_poses,
    num_calibration_params,
    num_markers

using Requires
using StaticArrays
using RigidBodyDynamics
using JuMP
using LinearAlgebra

using MathProgBase: SolverInterface.AbstractMathProgSolver
using RigidBodyDynamics.Graphs: TreePath

include("util.jl")
include("problem.jl")
include("result.jl")
include("deconstruct.jl")
include("residual.jl")
include("solve.jl")
include("synthetic.jl")

function __init__()
    @require DrakeVisualizer="49c7015b-b8db-5bc5-841b-fcb31c578176" begin
        @require RigidBodyTreeInspector="82daab19-8fc9-5c1e-9f69-37d6aaa0269b" begin
            @require Interact="c601a237-2ae4-5e1e-952c-7a85b0c7eef1" begin
                include("visualization.jl")
            end
        end
    end
end

end # module
