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

# TODO: clean up after https://github.com/MikeInnes/Requires.jl/issues/54 has been resolved.
req_rbti = false
req_interact = false
req_geometrytypes = false
req_colortypes = false

function __init__()
    @require RigidBodyTreeInspector="82daab19-8fc9-5c1e-9f69-37d6aaa0269b" begin
        global req_rbti = true
        req_rbti && req_interact && req_geometrytypes && req_colortypes && include("visualization.jl")
    end
    @require Interact="c601a237-2ae4-5e1e-952c-7a85b0c7eef1" begin
        global req_interact = true
        req_rbti && req_interact && req_geometrytypes && req_colortypes && include("visualization.jl")
    end
    @require GeometryTypes="4d00f742-c7ba-57c2-abde-4428a4b178cb" begin
        global req_geometrytypes = true
        req_rbti && req_interact && req_geometrytypes && req_colortypes && include("visualization.jl")
    end
    @require ColorTypes="3da002f7-5984-5a60-b8a6-cbb66c0b333f" begin
        global req_colortypes = true
        req_rbti && req_interact && req_geometrytypes && req_colortypes && include("visualization.jl")
    end
end

end # module
